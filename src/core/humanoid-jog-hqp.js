import {
    addEqualityRows,
    buildHardConstraintRows,
    buildTaskQP,
    finiteNumber,
    inferDimension,
    matVec,
    maxHalfspaceViolation,
    normalizeLevels,
    positiveViolationNorm,
    solveConstrainedTaskQP,
    taskResidualNorm,
    vectorNorm
} from './humanoid-jog-qp-controller-utils.js';
import {
    buildQPNumericSolveOptions,
    solveQPNumericActiveSet
} from './vfi-utils.js';

function flattenBlocks(blocks) {
    const out = [];
    for (const block of Array.isArray(blocks) ? blocks : []) out.push(block);
    return out;
}

function qAppliedFrom(qActive, dq) {
    if (!Array.isArray(qActive)) return null;
    return qActive.map((q, i) => finiteNumber(q, 0) + finiteNumber(dq?.[i], 0));
}

export function solveHumanoidJogHQP(options = {}) {
    const n = inferDimension(options);
    if (n <= 0) {
        return {
            solver: 'hqp',
            converged: true,
            reason: 'empty_problem',
            dq: [],
            qApplied: Array.isArray(options.qActive) ? options.qActive.slice() : null,
            activeResidualNorm: 0,
            maxViolation: 0,
            iterations: 0,
            stages: [],
            hqp_levels: 0,
            active_rows: 0,
            slack_norm: 0,
            qp_status: 'empty'
        };
    }

    const levels = normalizeLevels({ ...options, n });
    const hardConstraints = options.hardConstraints || {};
    const baseRows = buildHardConstraintRows(n, hardConstraints);
    const equalityTolerance = Math.max(0, finiteNumber(options.equalityTolerance, 1e-8));
    const tolerance = Math.max(0, finiteNumber(options.tolerance, 1e-8));
    const feasibleAcceptanceTolerance = Math.max(
        tolerance,
        Math.max(0, finiteNumber(options.feasibleAcceptanceTolerance, 1e-8))
    );
    const damping = Math.max(0, finiteNumber(options.damping, 1e-6));
    const qpOptions = options.qpOptions || buildQPNumericSolveOptions({});

    if (levels.length === 0) {
        const qp = solveConstrainedTaskQP({
            n,
            taskBlocks: [],
            hardConstraints,
            damping,
            x0: new Array(n).fill(0),
            qpOptions
        });
        const acceptedFeasible = qp.result?.converged === false && qp.maxViolation <= feasibleAcceptanceTolerance;
        const message = acceptedFeasible
            ? `${qp.result?.solverMessage ? `${qp.result.solverMessage}; ` : ''}accepted_feasible_warning(tol=${feasibleAcceptanceTolerance.toExponential(1)})`
            : (qp.result?.solverMessage || null);
        return {
            solver: 'hqp',
            converged: qp.maxViolation <= feasibleAcceptanceTolerance,
            reason: qp.maxViolation <= feasibleAcceptanceTolerance ? 'no_task' : 'constraint_violation',
            dq: qp.dq,
            qApplied: qAppliedFrom(options.qActive, qp.dq),
            activeResidualNorm: 0,
            maxViolation: qp.maxViolation,
            iterations: finiteNumber(qp.result?.iterations, 0),
            stages: [],
            hqp_levels: 0,
            active_rows: qp.A.length,
            slack_norm: qp.slackNorm,
            qp_status: qp.result?.converged === false && !acceptedFeasible ? 'failed' : 'solved',
            qp_message: message
        };
    }

    let dq = new Array(n).fill(0);
    let totalIterations = 0;
    const stages = [];
    const protectedJ = [];
    const protectedTarget = [];
    let finalA = baseRows.A.map((row) => row.slice());
    let finalB = baseRows.b.slice();
    let finalResult = null;
    let hierarchyLimited = false;
    let hierarchyLimitMessage = null;
    let primaryLimited = false;

    for (let levelIndex = 0; levelIndex < levels.length; levelIndex++) {
        const level = levels[levelIndex];
        const A = baseRows.A.map((row) => row.slice());
        const b = baseRows.b.slice();
        if (protectedJ.length > 0) {
            addEqualityRows(A, b, protectedJ, protectedTarget, equalityTolerance);
        }

        const { H, f } = buildTaskQP(n, level.taskBlocks, damping);
        const previousDq = dq.slice();
        const result = solveQPNumericActiveSet(H, f, A, b, dq, qpOptions);
        const candidateDq = Array.isArray(result?.x)
            ? result.x.slice(0, n).map((v) => finiteNumber(v, 0))
            : previousDq;
        totalIterations += finiteNumber(result?.iterations, 0);
        const residualNorm = taskResidualNorm(level.taskBlocks, candidateDq);
        const violation = maxHalfspaceViolation(A, b, candidateDq);
        const slackNorm = positiveViolationNorm(A, b, candidateDq);
        const levelFeasible = violation <= feasibleAcceptanceTolerance;
        const levelConverged = levelFeasible;
        stages.push({
            level: levelIndex + 1,
            label: level.label,
            rows: level.taskBlocks.reduce((sum, block) => sum + block.J.length, 0),
            residualNorm,
            maxViolation: violation,
            slackNorm,
            qpConverged: levelConverged
        });

        finalA = A;
        finalB = b;
        finalResult = result;
        if (!levelFeasible) {
            dq = previousDq;
            hierarchyLimited = true;
            primaryLimited = levelIndex === 0;
            const tag = primaryLimited ? 'primary_limited' : 'lower_priority_limited';
            hierarchyLimitMessage = `${tag}(level=${levelIndex + 1},slack=${slackNorm.toExponential(2)})`;
            break;
        }

        dq = candidateDq;
        for (const block of flattenBlocks(level.taskBlocks)) {
            const achieved = matVec(block.J, dq);
            protectedJ.push(...block.J.map((row) => row.slice()));
            protectedTarget.push(...achieved);
        }
    }

    const allBlocks = levels.flatMap((level) => level.taskBlocks);
    const activeResidualNorm = taskResidualNorm(allBlocks, dq);
    const maxViolation = maxHalfspaceViolation(finalA, finalB, dq);
    const slackNorm = positiveViolationNorm(finalA, finalB, dq);
    const acceptedFeasible = finalResult?.converged === false && maxViolation <= feasibleAcceptanceTolerance;
    const hardFeasible = maxViolation <= feasibleAcceptanceTolerance;
    const solverConverged = hardFeasible && !primaryLimited;
    let qpMessage = acceptedFeasible
        ? `${finalResult?.solverMessage ? `${finalResult.solverMessage}; ` : ''}accepted_feasible_warning(tol=${feasibleAcceptanceTolerance.toExponential(1)})`
        : (finalResult?.solverMessage || null);
    if (hierarchyLimited && hierarchyLimitMessage) {
        qpMessage = qpMessage ? `${qpMessage}; ${hierarchyLimitMessage}` : hierarchyLimitMessage;
    }

    return {
        solver: 'hqp',
        converged: solverConverged,
        reason: hierarchyLimited
            ? (primaryLimited ? 'primary_limited' : 'lower_priority_limited')
            : (solverConverged ? 'converged' : (maxViolation > feasibleAcceptanceTolerance ? 'constraint_violation' : 'qp_not_converged')),
        dq,
        qApplied: qAppliedFrom(options.qActive, dq),
        activeResidualNorm,
        maxViolation,
        iterations: totalIterations,
        stages,
        hqp_levels: levels.length,
        active_rows: finalA.length,
        slack_norm: slackNorm,
        qp_status: primaryLimited ? 'failed' : (hierarchyLimited ? 'limited' : (finalResult?.converged === false && !acceptedFeasible ? 'failed' : 'solved')),
        qp_objective: Number.isFinite(finalResult?.objective) ? finalResult.objective : null,
        qp_message: qpMessage,
        stepNorm: vectorNorm(dq)
    };
}
