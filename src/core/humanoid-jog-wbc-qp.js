import {
    buildHardConstraintRows,
    finiteNumber,
    inferDimension,
    matVec,
    maxHalfspaceViolation,
    normalizeTaskBlocks,
    positiveViolationNorm,
    solveConstrainedTaskQP,
    taskResidualNorm,
    vectorNorm
} from './humanoid-jog-qp-controller-utils.js';
import { buildQPNumericSolveOptions } from './vfi-utils.js';

function qAppliedFrom(qActive, dq) {
    if (!Array.isArray(qActive)) return null;
    return qActive.map((q, i) => finiteNumber(q, 0) + finiteNumber(dq?.[i], 0));
}

function buildTrackingBlocks(blocks, previousStep, previousTaskVelocity, gains = {}) {
    const kp = Math.max(0, finiteNumber(gains.kp, 1));
    const kd = Math.max(0, finiteNumber(gains.kd, 0));
    const ki = Math.max(0, finiteNumber(gains.ki, 0));
    const out = [];
    const robustTaskIntegral = [];
    let offset = 0;
    for (const block of blocks) {
        const prevVelocity = Array.isArray(previousStep) && previousStep.length > 0
            ? matVec(block.J, previousStep)
            : new Array(block.J.length).fill(0);
        const target = block.target.map((value, i) => {
            const desiredVelocity = Array.isArray(previousTaskVelocity)
                ? finiteNumber(previousTaskVelocity[offset + i], 0)
                : 0;
            const robustIntegral = ki * desiredVelocity;
            robustTaskIntegral.push(robustIntegral);
            return kp * finiteNumber(value, 0) - kd * finiteNumber(prevVelocity[i], 0) - robustIntegral;
        });
        out.push({
            ...block,
            target
        });
        offset += block.J.length;
    }
    return { taskBlocks: out, robustTaskIntegral };
}

function buildRobustHardConstraints(n, hardConstraints, previousStep, previousConstraintVelocity, gains = {}) {
    const baseRows = buildHardConstraintRows(n, hardConstraints || {});
    const constraintKi = Math.max(0, finiteNumber(gains.constraintKi, 0));
    if (constraintKi <= 0 || baseRows.A.length === 0) {
        return {
            hardConstraints: hardConstraints || {},
            baseRows,
            robustConstraintIntegral: new Array(baseRows.A.length).fill(0)
        };
    }

    const fallbackVelocity = matVec(baseRows.A, Array.isArray(previousStep) ? previousStep : new Array(n).fill(0));
    const b = baseRows.b.map((rhs, i) => {
        const previousVelocity = Array.isArray(previousConstraintVelocity)
            ? finiteNumber(previousConstraintVelocity[i], finiteNumber(fallbackVelocity[i], 0))
            : finiteNumber(fallbackVelocity[i], 0);
        const robustIntegral = constraintKi * Math.max(0, previousVelocity);
        const tightened = finiteNumber(rhs, 0) - robustIntegral;
        return rhs >= 0 ? Math.max(0, tightened) : tightened;
    });
    const robustConstraintIntegral = baseRows.b.map((rhs, i) => {
        return Math.max(0, finiteNumber(rhs, 0) - finiteNumber(b[i], 0));
    });
    return {
        hardConstraints: {
            A: baseRows.A.map((row) => row.slice()),
            b
        },
        baseRows,
        robustConstraintIntegral
    };
}

export function solveHumanoidJogKinematicWBCQP(options = {}) {
    const n = inferDimension(options);
    if (n <= 0) {
        return {
            solver: 'wbc_qp',
            converged: true,
            reason: 'empty_problem',
            dq: [],
            qApplied: Array.isArray(options.qActive) ? options.qActive.slice() : null,
            activeResidualNorm: 0,
            maxViolation: 0,
            iterations: 0,
            stages: [],
            qp_status: 'empty',
            tracking_norm: 0,
            constraint_violation: 0,
            nextState: { dqLast: [], taskVelocity: [] }
        };
    }

    const rawBlocks = normalizeTaskBlocks(options.taskBlocks, n);
    const previousStep = Array.isArray(options.previousStep)
        ? options.previousStep.slice(0, n).map((value) => finiteNumber(value, 0))
        : (Array.isArray(options.state?.dqLast)
            ? options.state.dqLast.slice(0, n).map((value) => finiteNumber(value, 0))
            : new Array(n).fill(0));
    while (previousStep.length < n) previousStep.push(0);
    const previousTaskVelocity = Array.isArray(options.state?.taskVelocity)
        ? options.state.taskVelocity.map((value) => finiteNumber(value, 0))
        : [];
    const previousConstraintVelocity = Array.isArray(options.state?.constraintVelocity)
        ? options.state.constraintVelocity.map((value) => finiteNumber(value, 0))
        : [];
    const gains = options.gains || {};
    const { taskBlocks, robustTaskIntegral } = buildTrackingBlocks(rawBlocks, previousStep, previousTaskVelocity, gains);
    const robustConstraints = buildRobustHardConstraints(
        n,
        options.hardConstraints || {},
        previousStep,
        previousConstraintVelocity,
        gains
    );
    const damping = Math.max(0, finiteNumber(options.damping, 1e-6));
    const qpOptions = options.qpOptions || buildQPNumericSolveOptions({});
    const solved = solveConstrainedTaskQP({
        n,
        taskBlocks,
        hardConstraints: robustConstraints.hardConstraints,
        damping,
        x0: previousStep,
        qpOptions
    });
    const { A, b } = buildHardConstraintRows(n, robustConstraints.hardConstraints || {});
    const taskVelocity = taskBlocks.flatMap((block) => matVec(block.J, solved.dq));
    const constraintVelocity = matVec(robustConstraints.baseRows.A, solved.dq);
    const maxViolation = maxHalfspaceViolation(A, b, solved.dq);
    const trackingNorm = taskResidualNorm(taskBlocks, solved.dq);
    const constraintViolation = positiveViolationNorm(A, b, solved.dq);
    const robustTaskIntegralNorm = vectorNorm(robustTaskIntegral);
    const robustConstraintIntegralNorm = vectorNorm(robustConstraints.robustConstraintIntegral);
    const tolerance = Math.max(0, finiteNumber(options.tolerance, 1e-8));
    const qpStatus = solved.result?.converged === false ? 'failed' : 'solved';

    return {
        solver: 'wbc_qp',
        converged: qpStatus === 'solved' && maxViolation <= tolerance,
        reason: qpStatus !== 'solved' ? 'qp_not_converged' : (maxViolation <= tolerance ? 'converged' : 'constraint_violation'),
        dq: solved.dq,
        qApplied: qAppliedFrom(options.qActive, solved.dq),
        activeResidualNorm: trackingNorm,
        maxViolation,
        iterations: finiteNumber(solved.result?.iterations, 0),
        stages: [{
            label: 'kinematic-wbc-qp',
            rows: taskBlocks.reduce((sum, block) => sum + block.J.length, 0),
            trackingNorm,
            constraintViolation
        }],
        qp_status: qpStatus,
        qpStatus,
        tracking_norm: trackingNorm,
        trackingNorm,
        constraint_violation: constraintViolation,
        constraintViolation,
        robust_task_integral_norm: robustTaskIntegralNorm,
        robustTaskIntegralNorm,
        robust_constraint_integral_norm: robustConstraintIntegralNorm,
        robustConstraintIntegralNorm,
        active_rows: A.length,
        slack_norm: solved.slackNorm,
        qp_objective: Number.isFinite(solved.result?.objective) ? solved.result.objective : null,
        qp_message: solved.result?.solverMessage || null,
        nextState: {
            dqLast: solved.dq.slice(),
            taskVelocity,
            constraintVelocity
        },
        stepNorm: vectorNorm(solved.dq)
    };
}
