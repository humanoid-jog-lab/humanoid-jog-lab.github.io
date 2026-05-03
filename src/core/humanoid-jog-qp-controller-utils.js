import {
    appendComponentBounds,
    appendEqualityConstraints,
    buildQPNumericSolveOptions,
    solveQPNumericActiveSet
} from './vfi-utils.js';

export function finiteNumber(value, fallback = 0) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
}

export function vectorNorm(v) {
    if (!Array.isArray(v) || v.length === 0) return 0;
    let sum = 0;
    for (let i = 0; i < v.length; i++) {
        const value = finiteNumber(v[i], 0);
        sum += value * value;
    }
    return Math.sqrt(sum);
}

export function dot(a, b) {
    let sum = 0;
    const n = Math.min(Array.isArray(a) ? a.length : 0, Array.isArray(b) ? b.length : 0);
    for (let i = 0; i < n; i++) sum += finiteNumber(a[i], 0) * finiteNumber(b[i], 0);
    return sum;
}

export function matVec(M, x) {
    const rows = Array.isArray(M) ? M.length : 0;
    const out = new Array(rows).fill(0);
    for (let i = 0; i < rows; i++) {
        const row = Array.isArray(M[i]) ? M[i] : [];
        out[i] = dot(row, x);
    }
    return out;
}

export function inferDimension({ taskBlocks = null, levels = null, hardConstraints = null, qActive = null } = {}) {
    if (Array.isArray(qActive) && qActive.length > 0) return qActive.length;
    const scanBlocks = (blocks) => {
        if (!Array.isArray(blocks)) return 0;
        for (const block of blocks) {
            const J = block?.J || block?.jacobian;
            if (Array.isArray(J) && Array.isArray(J[0]) && J[0].length > 0) return J[0].length;
        }
        return 0;
    };
    const fromTaskBlocks = scanBlocks(taskBlocks);
    if (fromTaskBlocks > 0) return fromTaskBlocks;
    if (Array.isArray(levels)) {
        for (const level of levels) {
            const fromLevel = scanBlocks(Array.isArray(level) ? level : level?.taskBlocks);
            if (fromLevel > 0) return fromLevel;
        }
    }
    const A = hardConstraints?.A || hardConstraints?.rows;
    if (Array.isArray(A) && Array.isArray(A[0]) && A[0].length > 0) return A[0].length;
    const lower = hardConstraints?.lowerStep || hardConstraints?.lower;
    if (Array.isArray(lower) && lower.length > 0) return lower.length;
    const upper = hardConstraints?.upperStep || hardConstraints?.upper;
    if (Array.isArray(upper) && upper.length > 0) return upper.length;
    return 0;
}

export function normalizeTaskBlocks(taskBlocks, n) {
    if (!Array.isArray(taskBlocks)) return [];
    const out = [];
    for (const block of taskBlocks) {
        const Jraw = block?.J || block?.jacobian;
        const targetRaw = block?.target || block?.rhs || block?.error;
        if (!Array.isArray(Jraw) || Jraw.length === 0) continue;
        const J = [];
        const target = [];
        for (let r = 0; r < Jraw.length; r++) {
            const rowRaw = Array.isArray(Jraw[r]) ? Jraw[r] : [];
            const row = new Array(n).fill(0);
            for (let c = 0; c < n; c++) row[c] = finiteNumber(rowRaw[c], 0);
            J.push(row);
            target.push(finiteNumber(Array.isArray(targetRaw) ? targetRaw[r] : targetRaw, 0));
        }
        out.push({
            label: typeof block?.label === 'string' ? block.label : 'task',
            J,
            target,
            weight: Math.max(0, finiteNumber(block?.weight, 1))
        });
    }
    return out;
}

export function normalizeLevels({ levels = null, taskBlocks = null, n = 0 } = {}) {
    if (Array.isArray(levels) && levels.length > 0) {
        return levels
            .map((level, idx) => {
                const blocks = normalizeTaskBlocks(Array.isArray(level) ? level : level?.taskBlocks, n);
                return {
                    label: typeof level?.label === 'string' ? level.label : `level_${idx + 1}`,
                    taskBlocks: blocks
                };
            })
            .filter((level) => level.taskBlocks.length > 0);
    }
    const blocks = normalizeTaskBlocks(taskBlocks, n);
    return blocks.length > 0 ? [{ label: 'primary', taskBlocks: blocks }] : [];
}

export function appendLeastSquaresTerm(H, f, J, target, weight = 1) {
    if (!Array.isArray(J) || J.length === 0) return;
    const n = H.length;
    const w = Math.max(0, finiteNumber(weight, 1));
    if (w <= 0) return;
    for (let r = 0; r < J.length; r++) {
        const row = Array.isArray(J[r]) ? J[r] : [];
        const yr = finiteNumber(Array.isArray(target) ? target[r] : target, 0);
        for (let i = 0; i < n; i++) {
            const Ji = finiteNumber(row[i], 0);
            if (!Ji) continue;
            f[i] -= w * Ji * yr;
            for (let j = 0; j < n; j++) {
                H[i][j] += w * Ji * finiteNumber(row[j], 0);
            }
        }
    }
}

export function buildHardConstraintRows(n, hardConstraints = {}) {
    const A = [];
    const b = [];
    const rows = hardConstraints?.A || hardConstraints?.rows;
    const rhs = hardConstraints?.b || hardConstraints?.rhs;
    if (Array.isArray(rows) && Array.isArray(rhs)) {
        for (let i = 0; i < rows.length; i++) {
            const src = Array.isArray(rows[i]) ? rows[i] : [];
            const row = new Array(n).fill(0);
            for (let j = 0; j < n; j++) row[j] = finiteNumber(src[j], 0);
            A.push(row);
            b.push(finiteNumber(rhs[i], 0));
        }
    }
    const lower = hardConstraints?.lowerStep || hardConstraints?.lower || hardConstraints?.stepLower;
    const upper = hardConstraints?.upperStep || hardConstraints?.upper || hardConstraints?.stepUpper;
    if (Array.isArray(lower) || Array.isArray(upper)) {
        const lb = new Array(n).fill(Number.NEGATIVE_INFINITY);
        const ub = new Array(n).fill(Number.POSITIVE_INFINITY);
        for (let i = 0; i < n; i++) {
            if (Array.isArray(lower)) lb[i] = finiteNumber(lower[i], Number.NEGATIVE_INFINITY);
            if (Array.isArray(upper)) ub[i] = finiteNumber(upper[i], Number.POSITIVE_INFINITY);
        }
        appendComponentBounds(A, b, n, lb, ub);
    } else if (Number.isFinite(hardConstraints?.maxStep)) {
        const maxStep = Math.max(0, finiteNumber(hardConstraints.maxStep, 0));
        appendComponentBounds(A, b, n, -maxStep, maxStep);
    }
    return { A, b };
}

export function maxHalfspaceViolation(A, b, x) {
    let maxViolation = 0;
    for (let i = 0; i < (Array.isArray(A) ? A.length : 0); i++) {
        const violation = dot(A[i], x) - finiteNumber(b?.[i], Number.POSITIVE_INFINITY);
        if (violation > maxViolation) maxViolation = violation;
    }
    return maxViolation;
}

export function positiveViolationNorm(A, b, x) {
    let sum = 0;
    for (let i = 0; i < (Array.isArray(A) ? A.length : 0); i++) {
        const violation = Math.max(0, dot(A[i], x) - finiteNumber(b?.[i], Number.POSITIVE_INFINITY));
        sum += violation * violation;
    }
    return Math.sqrt(sum);
}

export function taskResidualNorm(taskBlocks, dq) {
    const residuals = [];
    for (const block of Array.isArray(taskBlocks) ? taskBlocks : []) {
        const achieved = matVec(block.J, dq);
        for (let i = 0; i < achieved.length; i++) {
            residuals.push((achieved[i] || 0) - finiteNumber(block.target?.[i], 0));
        }
    }
    return vectorNorm(residuals);
}

export function buildTaskQP(n, taskBlocks, damping = 1e-6) {
    const H = new Array(n).fill(0).map(() => new Array(n).fill(0));
    const f = new Array(n).fill(0);
    for (const block of Array.isArray(taskBlocks) ? taskBlocks : []) {
        appendLeastSquaresTerm(H, f, block.J, block.target, block.weight);
    }
    const lambda = Math.max(0, finiteNumber(damping, 1e-6));
    for (let i = 0; i < n; i++) H[i][i] += lambda;
    return { H, f };
}

export function solveConstrainedTaskQP({
    n,
    taskBlocks,
    hardConstraints,
    damping = 1e-6,
    x0 = null,
    qpOptions = null
}) {
    const { A, b } = buildHardConstraintRows(n, hardConstraints);
    const { H, f } = buildTaskQP(n, taskBlocks, damping);
    const solveOptions = qpOptions || buildQPNumericSolveOptions({});
    const result = solveQPNumericActiveSet(H, f, A, b, Array.isArray(x0) ? x0 : new Array(n).fill(0), solveOptions);
    const dq = Array.isArray(result?.x) ? result.x.slice(0, n).map((v) => finiteNumber(v, 0)) : new Array(n).fill(0);
    return {
        dq,
        result,
        A,
        b,
        maxViolation: maxHalfspaceViolation(A, b, dq),
        slackNorm: positiveViolationNorm(A, b, dq),
        residualNorm: taskResidualNorm(taskBlocks, dq)
    };
}

export function addEqualityRows(A, b, J, target, tolerance = 1e-8) {
    appendEqualityConstraints(A, b, J, target, { tolerance });
}
