import {
    buildHardConstraintRows,
    finiteNumber,
    inferDimension,
    matVec,
    maxHalfspaceViolation,
    normalizeTaskBlocks,
    positiveViolationNorm,
    taskResidualNorm,
    vectorNorm
} from './humanoid-jog-qp-controller-utils.js';
import {
    buildQPNumericSolveOptions,
    solveQPNumericActiveSet
} from './vfi-utils.js';

function qAppliedFrom(qActive, dq) {
    if (!Array.isArray(qActive)) return null;
    return qActive.map((q, i) => finiteNumber(q, 0) + finiteNumber(dq?.[i], 0));
}

function clamp01(value) {
    return Math.max(0, Math.min(1, finiteNumber(value, 0)));
}

function addDenseLeastSquaresRow(H, f, coeff, target, weight) {
    const w = Math.max(0, finiteNumber(weight, 1));
    if (w <= 0) return;
    const zDim = H.length;
    const rhs = finiteNumber(target, 0);
    for (let i = 0; i < zDim; i++) {
        const ci = finiteNumber(coeff?.[i], 0);
        if (!ci) continue;
        f[i] -= w * ci * rhs;
        for (let j = 0; j < zDim; j++) {
            H[i][j] += w * ci * finiteNumber(coeff?.[j], 0);
        }
    }
}

function splitCommandAndSecondaryBlocks(blocks) {
    const commandBlocks = blocks.filter((block) => block.label === 'command');
    if (commandBlocks.length === 0) {
        return {
            commandBlocks: blocks,
            secondaryBlocks: []
        };
    }
    return {
        commandBlocks,
        secondaryBlocks: blocks.filter((block) => block.label !== 'command')
    };
}

function targetScaleForBlock(block, minTargetScale = 0.02) {
    return Math.max(vectorNorm(Array.isArray(block?.target) ? block.target : []), minTargetScale);
}

function buildPredictiveRowsForStep({ row, target, n, horizon, stepIndex, scaleIndex, scaleMultiplier }) {
    const coeff = new Array(n * horizon + 1).fill(0);
    for (let k = 0; k <= stepIndex; k++) {
        for (let j = 0; j < n; j++) {
            coeff[k * n + j] += finiteNumber(row?.[j], 0);
        }
    }
    coeff[scaleIndex] = -scaleMultiplier * finiteNumber(target, 0);
    return coeff;
}

function buildPerStepHardConstraints(n, horizon, baseRows, scaleIndex) {
    const zDim = n * horizon + 1;
    const A = [];
    const b = [];
    for (let k = 0; k < horizon; k++) {
        for (let r = 0; r < baseRows.A.length; r++) {
            const row = new Array(zDim).fill(0);
            for (let j = 0; j < n; j++) row[k * n + j] = finiteNumber(baseRows.A[r]?.[j], 0);
            A.push(row);
            b.push(finiteNumber(baseRows.b?.[r], 0));
        }
    }
    const lower = new Array(zDim).fill(0);
    lower[scaleIndex] = -1;
    A.push(lower);
    b.push(0);
    const upper = new Array(zDim).fill(0);
    upper[scaleIndex] = 1;
    A.push(upper);
    b.push(1);
    return { A, b };
}

function computeScaledTrackingNorm(commandBlocks, secondaryBlocks, dq, scale) {
    const residuals = [];
    for (const block of commandBlocks) {
        const achieved = matVec(block.J, dq);
        for (let i = 0; i < achieved.length; i++) {
            residuals.push(finiteNumber(achieved[i], 0) - scale * finiteNumber(block.target?.[i], 0));
        }
    }
    for (const block of secondaryBlocks) {
        const achieved = matVec(block.J, dq);
        for (let i = 0; i < achieved.length; i++) {
            residuals.push(finiteNumber(achieved[i], 0) - finiteNumber(block.target?.[i], 0));
        }
    }
    return vectorNorm(residuals);
}

export function solveHumanoidJogPredictiveIK(options = {}) {
    const n = inferDimension(options);
    if (n <= 0) {
        return {
            solver: 'predictive_ik',
            converged: true,
            reason: 'empty_problem',
            dq: [],
            qApplied: Array.isArray(options.qActive) ? options.qActive.slice() : null,
            activeResidualNorm: 0,
            maxViolation: 0,
            iterations: 0,
            stages: [],
            task_scale: 1,
            predictive_horizon: 0,
            tracking_norm: 0,
            constraint_violation: 0,
            qp_status: 'empty'
        };
    }

    const blocks = normalizeTaskBlocks(options.taskBlocks, n);
    const { commandBlocks, secondaryBlocks } = splitCommandAndSecondaryBlocks(blocks);
    const horizon = Math.max(1, Math.min(5, Math.floor(finiteNumber(options.horizonSteps, 3))));
    const scaleColumn = n * horizon;
    const zDim = n * horizon + 1;
    const commandWeight = Math.max(0, finiteNumber(options.commandWeight, 1e5));
    const secondaryWeight = Math.max(0, finiteNumber(options.secondaryWeight, 1));
    const scaleWeight = Math.max(0, finiteNumber(options.scaleWeight, 100));
    const smoothnessWeight = Math.max(0, finiteNumber(options.smoothnessWeight, 0.01));
    const minTargetScale = Math.max(1e-9, finiteNumber(options.minTargetScale, 0.02));
    const secondaryAcrossHorizon = options.secondaryAcrossHorizon !== false;
    const damping = Math.max(1e-6, finiteNumber(options.damping, 1e-6));
    const H = new Array(zDim).fill(0).map(() => new Array(zDim).fill(0));
    const f = new Array(zDim).fill(0);
    let secondaryHorizonRows = 0;

    for (let i = 0; i < n * horizon; i++) H[i][i] += damping;
    H[scaleColumn][scaleColumn] += Math.max(damping, 1e-9);
    addDenseLeastSquaresRow(H, f, (() => {
        const coeff = new Array(zDim).fill(0);
        coeff[scaleColumn] = 1;
        return coeff;
    })(), 1, scaleWeight);

    for (let k = 0; k < horizon; k++) {
        const scaleMultiplier = k + 1;
        for (const block of commandBlocks) {
            const blockWeight = commandWeight * Math.max(0, finiteNumber(block.weight, 1));
            const targetScale = targetScaleForBlock(block, minTargetScale);
            for (let r = 0; r < block.J.length; r++) {
                const coeff = buildPredictiveRowsForStep({
                    row: block.J[r],
                    target: block.target[r],
                    n,
                    horizon,
                    stepIndex: k,
                    scaleIndex: scaleColumn,
                    scaleMultiplier
                });
                for (let c = 0; c < coeff.length; c++) coeff[c] /= targetScale;
                addDenseLeastSquaresRow(H, f, coeff, 0, blockWeight);
            }
        }

        if (secondaryAcrossHorizon || k === 0) {
            for (const block of secondaryBlocks) {
                const blockWeight = secondaryWeight * Math.max(0, finiteNumber(block.weight, 1));
                for (let r = 0; r < block.J.length; r++) {
                    const coeff = new Array(zDim).fill(0);
                    const lastStep = secondaryAcrossHorizon ? k : 0;
                    for (let step = 0; step <= lastStep; step++) {
                        for (let j = 0; j < n; j++) {
                            coeff[step * n + j] += finiteNumber(block.J[r]?.[j], 0);
                        }
                    }
                    addDenseLeastSquaresRow(H, f, coeff, finiteNumber(block.target?.[r], 0), blockWeight);
                    secondaryHorizonRows += 1;
                }
            }
        }
    }

    for (let k = 1; k < horizon; k++) {
        for (let j = 0; j < n; j++) {
            const coeff = new Array(zDim).fill(0);
            coeff[k * n + j] = 1;
            coeff[(k - 1) * n + j] = -1;
            addDenseLeastSquaresRow(H, f, coeff, 0, smoothnessWeight);
        }
    }

    const baseRows = buildHardConstraintRows(n, options.hardConstraints || {});
    const { A, b } = buildPerStepHardConstraints(n, horizon, baseRows, scaleColumn);
    const x0 = new Array(zDim).fill(0);
    x0[scaleColumn] = 0.5;
    const qpOptions = options.qpOptions || buildQPNumericSolveOptions({});
    const result = solveQPNumericActiveSet(H, f, A, b, x0, qpOptions);
    const z = Array.isArray(result?.x) ? result.x.slice(0, zDim).map((v) => finiteNumber(v, 0)) : x0;
    while (z.length < zDim) z.push(0);
    const dq = z.slice(0, n).map((v) => finiteNumber(v, 0));
    const taskScale = clamp01(z[scaleColumn]);
    const scales = Array.from({ length: horizon }, () => taskScale);
    const maxViolation = maxHalfspaceViolation(baseRows.A, baseRows.b, dq);
    const constraintViolation = positiveViolationNorm(baseRows.A, baseRows.b, dq);
    const trackingNorm = computeScaledTrackingNorm(commandBlocks, secondaryBlocks, dq, taskScale);
    const activeResidualNorm = taskResidualNorm(blocks, dq);
    const tolerance = Math.max(0, finiteNumber(options.tolerance, 1e-8));
    const acceptedFeasible = result?.converged === false && maxViolation <= tolerance;
    const qpStatus = result?.converged === false && !acceptedFeasible ? 'failed' : 'solved';
    const converged = qpStatus === 'solved' && maxViolation <= tolerance;

    return {
        solver: 'predictive_ik',
        converged,
        reason: converged ? 'converged' : (maxViolation > tolerance ? 'constraint_violation' : 'qp_not_converged'),
        dq,
        qApplied: qAppliedFrom(options.qActive, dq),
        activeResidualNorm,
        maxViolation,
        iterations: finiteNumber(result?.iterations, 0),
        stages: [{
            label: 'predictive-ik-task-scaling',
            horizonSteps: horizon,
            rows: blocks.reduce((sum, block) => sum + block.J.length, 0),
            taskScale,
            trackingNorm,
            constraintViolation
        }],
        task_scale: taskScale,
        taskScale,
        scale_sequence: scales,
        scaleSequence: scales,
        predictive_horizon: horizon,
        predictiveHorizon: horizon,
        tracking_norm: trackingNorm,
        trackingNorm,
        constraint_violation: constraintViolation,
        constraintViolation,
        active_rows: baseRows.A.length,
        predictive_rows: A.length,
        scale_bound_rows: 2,
        secondary_horizon_rows: secondaryHorizonRows,
        min_target_scale: minTargetScale,
        slack_norm: constraintViolation,
        qp_status: qpStatus,
        qpStatus,
        qp_objective: Number.isFinite(result?.objective) ? result.objective : null,
        qp_message: result?.solverMessage || null,
        stepNorm: vectorNorm(dq)
    };
}
