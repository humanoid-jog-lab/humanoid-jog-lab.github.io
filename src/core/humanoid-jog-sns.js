import {
    addEqualityRows,
    buildTaskQP,
    buildHardConstraintRows,
    finiteNumber,
    inferDimension,
    matVec,
    maxHalfspaceViolation,
    normalizeTaskBlocks,
    positiveViolationNorm,
    taskResidualNorm,
    vectorNorm,
    dot
} from './humanoid-jog-qp-controller-utils.js';
import {
    buildQPNumericSolveOptions,
    solveQPNumericActiveSet
} from './vfi-utils.js';

function qAppliedFrom(qActive, dq) {
    if (!Array.isArray(qActive)) return null;
    return qActive.map((q, i) => finiteNumber(q, 0) + finiteNumber(dq?.[i], 0));
}

function splitPrimaryAndSecondaryBlocks(taskBlocks) {
    const commandBlocks = taskBlocks.filter((block) => block.label === 'command');
    if (commandBlocks.length === 0) {
        return {
            primaryBlocks: taskBlocks,
            secondaryBlocks: []
        };
    }
    return {
        primaryBlocks: commandBlocks,
        secondaryBlocks: taskBlocks.filter((block) => block.label !== 'command')
    };
}

function appendProtectedTaskRows(A, b, blocks, dq, tolerance) {
    for (const block of blocks) {
        addEqualityRows(A, b, block.J, matVec(block.J, dq), tolerance);
    }
}

function solveLinearSystem(A, b) {
    const n = Array.isArray(A) ? A.length : 0;
    if (n === 0) return [];
    const M = A.map((row, i) => {
        const dense = new Array(n + 1).fill(0);
        for (let j = 0; j < n; j++) dense[j] = finiteNumber(row?.[j], 0);
        dense[n] = finiteNumber(b?.[i], 0);
        return dense;
    });
    for (let col = 0; col < n; col++) {
        let pivotRow = col;
        let pivotMag = Math.abs(finiteNumber(M[col]?.[col], 0));
        for (let row = col + 1; row < n; row++) {
            const mag = Math.abs(finiteNumber(M[row]?.[col], 0));
            if (mag > pivotMag) {
                pivotMag = mag;
                pivotRow = row;
            }
        }
        if (pivotMag < 1e-12) return null;
        if (pivotRow !== col) [M[col], M[pivotRow]] = [M[pivotRow], M[col]];
        const pivot = finiteNumber(M[col][col], 1);
        for (let j = col; j <= n; j++) M[col][j] /= pivot;
        for (let row = 0; row < n; row++) {
            if (row === col) continue;
            const factor = finiteNumber(M[row][col], 0);
            if (Math.abs(factor) < 1e-18) continue;
            for (let j = col; j <= n; j++) M[row][j] -= factor * M[col][j];
        }
    }
    return M.map((row) => finiteNumber(row[n], 0));
}

function flattenTaskBlocks(blocks, n) {
    const J = [];
    const target = [];
    for (const block of Array.isArray(blocks) ? blocks : []) {
        const rows = Array.isArray(block?.J) ? block.J : [];
        const rhs = Array.isArray(block?.target) ? block.target : [];
        const weight = Math.sqrt(Math.max(0, finiteNumber(block?.weight, 1)));
        if (weight <= 0) continue;
        for (let r = 0; r < rows.length; r++) {
            const row = new Array(n).fill(0);
            for (let c = 0; c < n; c++) row[c] = weight * finiteNumber(rows[r]?.[c], 0);
            J.push(row);
            target.push(weight * finiteNumber(rhs[r], 0));
        }
    }
    return { J, target };
}

function extractComponentBounds(n, hardConstraints = {}) {
    const { A, b } = buildHardConstraintRows(n, hardConstraints);
    const lower = new Array(n).fill(Number.NEGATIVE_INFINITY);
    const upper = new Array(n).fill(Number.POSITIVE_INFINITY);
    const singletonRows = new Set();
    const eps = 1e-12;

    for (let rowIndex = 0; rowIndex < A.length; rowIndex++) {
        const row = A[rowIndex];
        let nz = 0;
        let idx = -1;
        let coeff = 0;
        for (let j = 0; j < n; j++) {
            const value = finiteNumber(row?.[j], 0);
            if (Math.abs(value) > eps) {
                nz += 1;
                idx = j;
                coeff = value;
                if (nz > 1) break;
            }
        }
        if (nz !== 1 || idx < 0) continue;
        const bound = finiteNumber(b?.[rowIndex], 0) / coeff;
        if (coeff > 0) upper[idx] = Math.min(upper[idx], bound);
        else lower[idx] = Math.max(lower[idx], bound);
        singletonRows.add(rowIndex);
    }

    return { A, b, lower, upper, singletonRows };
}

function solveEnabledLeastNorm(J, target, enabled, fixed, damping) {
    const n = Array.isArray(fixed) ? fixed.length : 0;
    const enabledIndices = [];
    for (let i = 0; i < n; i++) {
        if (enabled[i]) enabledIndices.push(i);
    }
    const out = fixed.slice();
    if (J.length === 0 || enabledIndices.length === 0) return out;

    const residual = new Array(J.length).fill(0);
    for (let r = 0; r < J.length; r++) {
        residual[r] = finiteNumber(target?.[r], 0) - dot(J[r], fixed);
    }

    const gram = Array.from({ length: J.length }, () => new Array(J.length).fill(0));
    const lambda = Math.max(0, finiteNumber(damping, 1e-9));
    for (let r = 0; r < J.length; r++) {
        for (let c = 0; c < J.length; c++) {
            let value = r === c ? lambda : 0;
            for (const idx of enabledIndices) {
                value += finiteNumber(J[r]?.[idx], 0) * finiteNumber(J[c]?.[idx], 0);
            }
            gram[r][c] = value;
        }
    }
    const alpha = solveLinearSystem(gram, residual);
    if (!Array.isArray(alpha)) return out;
    for (const idx of enabledIndices) {
        let value = 0;
        for (let r = 0; r < J.length; r++) {
            value += finiteNumber(J[r]?.[idx], 0) * finiteNumber(alpha[r], 0);
        }
        out[idx] = value;
    }
    return out;
}

function interpolateAffine(offset, direction, scale) {
    return offset.map((value, i) => finiteNumber(value, 0) + scale * finiteNumber(direction?.[i], 0));
}

function computeBoxScale(offset, direction, lower, upper, enabled, tolerance = 1e-10) {
    let scale = 1;
    let criticalJoint = null;
    let saturationValue = null;
    let side = null;
    for (let i = 0; i < offset.length; i++) {
        if (enabled && !enabled[i]) continue;
        const qi = finiteNumber(offset[i], 0);
        const vi = finiteNumber(direction[i], 0);
        const qAtOne = qi + vi;
        let candidate = null;
        let candidateValue = null;
        let candidateSide = null;
        if (Number.isFinite(upper?.[i]) && qAtOne > upper[i] + tolerance && Math.abs(vi) > tolerance) {
            candidate = (upper[i] - qi) / vi;
            candidateValue = upper[i];
            candidateSide = 'upper';
        } else if (Number.isFinite(lower?.[i]) && qAtOne < lower[i] - tolerance && Math.abs(vi) > tolerance) {
            candidate = (lower[i] - qi) / vi;
            candidateValue = lower[i];
            candidateSide = 'lower';
        }
        if (candidate == null) continue;
        candidate = Math.max(0, Math.min(1, candidate));
        if (candidate < scale) {
            scale = candidate;
            criticalJoint = i;
            saturationValue = candidateValue;
            side = candidateSide;
        }
    }
    return { scale, criticalJoint, saturationValue, side };
}

function isWithinBox(x, lower, upper, tolerance = 1e-8) {
    for (let i = 0; i < x.length; i++) {
        const value = finiteNumber(x[i], 0);
        if (Number.isFinite(lower?.[i]) && value < lower[i] - tolerance) return false;
        if (Number.isFinite(upper?.[i]) && value > upper[i] + tolerance) return false;
    }
    return true;
}

function solveSingleTaskSNS({ J, target, lower, upper, damping, tolerance }) {
    const n = Array.isArray(lower) ? lower.length : 0;
    const enabled = new Array(n).fill(true);
    const fixed = new Array(n).fill(0);
    const saturatedJoints = [];
    const stages = [];
    let bestScale = 0;
    let bestDq = new Array(n).fill(0);
    let bestCriticalJoint = null;
    const maxIterations = n + 1;

    for (let iter = 0; iter < maxIterations; iter++) {
        const unitTarget = target.slice();
        const zeroTarget = target.map(() => 0);
        const offset = solveEnabledLeastNorm(J, zeroTarget, enabled, fixed, damping);
        const full = solveEnabledLeastNorm(J, unitTarget, enabled, fixed, damping);
        const direction = full.map((value, i) => finiteNumber(value, 0) - finiteNumber(offset[i], 0));
        const scaleInfo = computeBoxScale(offset, direction, lower, upper, enabled, tolerance);
        const scaled = interpolateAffine(offset, direction, scaleInfo.scale);
        const scaledTarget = target.map((value) => scaleInfo.scale * finiteNumber(value, 0));
        const scaledResidual = taskResidualNorm([{ J, target: scaledTarget }], scaled);
        const scaledOffsetResidual = taskResidualNorm([{ J, target: scaledTarget }], offset);
        const residualTolerance = Math.max(tolerance, 1e-8);
        const scaledTracksRequestedTask = scaledResidual <= residualTolerance
            || scaledResidual < scaledOffsetResidual - residualTolerance;
        if (scaledTracksRequestedTask && (scaleInfo.scale > bestScale + 1e-12 || (scaleInfo.scale === 0 && iter === 0))) {
            bestScale = scaleInfo.scale;
            bestDq = scaled;
            bestCriticalJoint = scaleInfo.criticalJoint;
        }

        stages.push({
            label: 'sns-saturation',
            iteration: iter + 1,
            taskScale: scaleInfo.scale,
            criticalJoint: scaleInfo.criticalJoint,
            saturatedJoints: saturatedJoints.slice()
        });

        const fullResidual = taskResidualNorm([{ J, target }], full);
        const offsetResidual = taskResidualNorm([{ J, target }], offset);
        const improvesTask = fullResidual < offsetResidual - residualTolerance;
        if (isWithinBox(full, lower, upper, tolerance) && (fullResidual <= residualTolerance || improvesTask)) {
            return {
                dq: full,
                taskScale: 1,
                saturatedJoints,
                criticalJoint: saturatedJoints.length > 0 ? saturatedJoints[saturatedJoints.length - 1] : null,
                iterations: iter + 1,
                stages,
                feasibleFullTask: true
            };
        }
        if (isWithinBox(full, lower, upper, tolerance)) break;

        if (scaleInfo.criticalJoint == null || !enabled[scaleInfo.criticalJoint]) break;
        const j = scaleInfo.criticalJoint;
        enabled[j] = false;
        fixed[j] = finiteNumber(scaleInfo.saturationValue, 0);
        saturatedJoints.push(j);
        stages[stages.length - 1].saturatedJoint = j;
        stages[stages.length - 1].saturationSide = scaleInfo.side;

        if (!enabled.some(Boolean)) break;
    }

    return {
        dq: bestDq,
        taskScale: bestScale,
        saturatedJoints,
        criticalJoint: bestCriticalJoint,
        iterations: stages.length,
        stages,
        feasibleFullTask: false
    };
}

function repairGeneralHardRows({
    n,
    primaryBlocks,
    damping,
    A,
    b,
    dq,
    tolerance,
    qpOptions
} = {}) {
    const beforeViolation = maxHalfspaceViolation(A, b, dq);
    const zero = new Array(n).fill(0);
    const zeroViolation = maxHalfspaceViolation(A, b, zero);
    if (beforeViolation <= tolerance) {
        return {
            dq,
            accepted: false,
            skipped: true,
            beforeViolation,
            afterViolation: beforeViolation,
            iterations: 0
        };
    }
    const { H, f } = buildTaskQP(n, primaryBlocks, damping);
    const repair = solveQPNumericActiveSet(H, f, A, b, dq, qpOptions);
    const candidate = Array.isArray(repair?.x)
        ? repair.x.slice(0, n).map((value) => finiteNumber(value, 0))
        : dq;
    const afterViolation = maxHalfspaceViolation(A, b, candidate);
    const zeroResidual = taskResidualNorm(primaryBlocks, zero);
    const candidateResidual = taskResidualNorm(primaryBlocks, candidate);
    const zeroFeasible = zeroViolation <= tolerance;
    const accepted = afterViolation <= tolerance && (
        !zeroFeasible ||
        candidateResidual < zeroResidual - Math.max(tolerance, 1e-8) ||
        vectorNorm(candidate) <= Math.max(tolerance, 1e-8)
    );
    const fallbackToZero = !accepted && zeroFeasible;
    const fallbackCandidate = fallbackToZero ? zero : dq;
    return {
        dq: accepted ? candidate : fallbackCandidate,
        accepted,
        fallbackToZero,
        skipped: false,
        beforeViolation,
        afterViolation: accepted ? afterViolation : maxHalfspaceViolation(A, b, fallbackCandidate),
        residualNorm: accepted ? candidateResidual : taskResidualNorm(primaryBlocks, fallbackCandidate),
        iterations: finiteNumber(repair?.iterations, 0)
    };
}

export function solveHumanoidJogSNS(options = {}) {
    const n = inferDimension(options);
    if (n <= 0) {
        return {
            solver: 'sns',
            converged: true,
            reason: 'empty_problem',
            dq: [],
            qApplied: Array.isArray(options.qActive) ? options.qActive.slice() : null,
            activeResidualNorm: 0,
            maxViolation: 0,
            iterations: 0,
            stages: [],
            task_scale: 1,
            saturated_joints: [],
            critical_joint: null
        };
    }

    const taskBlocks = normalizeTaskBlocks(options.taskBlocks, n);
    const damping = Math.max(0, finiteNumber(options.damping, 1e-6));
    const qpOptions = options.qpOptions || buildQPNumericSolveOptions({});
    const { primaryBlocks, secondaryBlocks } = splitPrimaryAndSecondaryBlocks(taskBlocks);
    const { A, b, lower, upper } = extractComponentBounds(n, options.hardConstraints || {});
    const tolerance = Math.max(0, finiteNumber(options.tolerance, 1e-8));
    const { J: primaryJ, target: primaryTarget } = flattenTaskBlocks(primaryBlocks, n);
    const primary = solveSingleTaskSNS({
        J: primaryJ,
        target: primaryTarget,
        lower,
        upper,
        damping,
        tolerance
    });
    let dq = primary.dq.map((value) => finiteNumber(value, 0));
    let totalIterations = finiteNumber(primary.iterations, 0);
    const stages = [{
        label: 'sns-primary',
        residualNorm: taskResidualNorm(primaryBlocks, dq),
        taskScale: primary.taskScale,
        saturatedJoints: primary.saturatedJoints.slice(),
        feasibleFullTask: primary.feasibleFullTask
    }, ...primary.stages];

    const primaryRepair = repairGeneralHardRows({
        n,
        primaryBlocks,
        damping,
        A,
        b,
        dq,
        tolerance,
        qpOptions
    });
    totalIterations += finiteNumber(primaryRepair.iterations, 0);
    if (!primaryRepair.skipped) {
        stages.push({
            label: 'sns-general-row-repair',
            accepted: primaryRepair.accepted,
            fallbackToZero: primaryRepair.fallbackToZero,
            beforeViolation: primaryRepair.beforeViolation,
            afterViolation: primaryRepair.afterViolation,
            residualNorm: primaryRepair.residualNorm
        });
    }
    dq = primaryRepair.dq.map((value) => finiteNumber(value, 0));

    if (secondaryBlocks.length > 0) {
        const equalityTolerance = Math.max(0, finiteNumber(options.equalityTolerance, 1e-8));
        const secondaryA = A.map((row) => row.slice());
        const secondaryB = b.slice();
        appendProtectedTaskRows(secondaryA, secondaryB, primaryBlocks, dq, equalityTolerance);

        const { H, f } = buildTaskQP(n, secondaryBlocks, damping);
        const secondary = solveQPNumericActiveSet(H, f, secondaryA, secondaryB, dq, qpOptions);
        totalIterations += finiteNumber(secondary?.iterations, 0);
        const candidate = Array.isArray(secondary?.x)
            ? secondary.x.slice(0, n).map((value) => finiteNumber(value, 0))
            : dq;
        const candidateViolation = maxHalfspaceViolation(secondaryA, secondaryB, candidate);
        const beforeSecondaryResidual = taskResidualNorm(secondaryBlocks, dq);
        const afterSecondaryResidual = taskResidualNorm(secondaryBlocks, candidate);
        const accepted = candidateViolation <= tolerance && afterSecondaryResidual <= beforeSecondaryResidual + 1e-10;
        stages.push({
            label: 'sns-secondary',
            residualNorm: accepted ? afterSecondaryResidual : beforeSecondaryResidual,
            accepted,
            maxViolation: candidateViolation
        });
        if (accepted) dq = candidate;
    }

    const maxViolation = maxHalfspaceViolation(A, b, dq);
    const slackNorm = positiveViolationNorm(A, b, dq);
    const residualNorm = taskResidualNorm(taskBlocks, dq);
    const saturatedJoints = primary.saturatedJoints.slice();
    const constrained = maxViolation > tolerance;
    const taskScaleLimited = primary.taskScale < 1 - 1e-9;

    return {
        solver: 'sns',
        converged: !constrained,
        reason: constrained ? 'constraint_violation' : (taskScaleLimited ? 'scaled' : 'converged'),
        dq,
        qApplied: qAppliedFrom(options.qActive, dq),
        activeResidualNorm: residualNorm,
        maxViolation,
        iterations: totalIterations,
        stages,
        task_scale: primary.taskScale,
        taskScale: primary.taskScale,
        saturated_joints: saturatedJoints,
        saturatedJoints,
        critical_joint: primary.criticalJoint,
        criticalJoint: primary.criticalJoint,
        active_rows: A.length,
        slack_norm: slackNorm,
        qp_status: constrained ? 'constraint_violation' : 'solved',
        stepNorm: vectorNorm(dq)
    };
}
