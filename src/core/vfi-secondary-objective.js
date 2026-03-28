import { gradientVFICCTSM } from './cctsm.js';

function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function dot(a, b) {
    const n = Math.min(Array.isArray(a) ? a.length : 0, Array.isArray(b) ? b.length : 0);
    let sum = 0;
    for (let i = 0; i < n; i++) sum += asFinite(a[i]) * asFinite(b[i]);
    return sum;
}

function vectorNorm(v) {
    return Math.sqrt(dot(v, v));
}

function matVec(A, x) {
    const rows = Array.isArray(A) ? A.length : 0;
    const out = new Array(rows).fill(0);
    for (let i = 0; i < rows; i++) {
        const row = Array.isArray(A[i]) ? A[i] : [];
        out[i] = dot(row, x);
    }
    return out;
}

function solveLinearSystem(A, b) {
    const n = Array.isArray(A) ? A.length : 0;
    if (n === 0) return [];

    const M = A.map((row, i) => {
        const dense = new Array(n + 1).fill(0);
        for (let j = 0; j < n; j++) dense[j] = asFinite(row?.[j]);
        dense[n] = asFinite(b?.[i]);
        return dense;
    });

    for (let col = 0; col < n; col++) {
        let pivotRow = col;
        let pivotMag = Math.abs(asFinite(M[col]?.[col]));
        for (let row = col + 1; row < n; row++) {
            const mag = Math.abs(asFinite(M[row]?.[col]));
            if (mag > pivotMag) {
                pivotMag = mag;
                pivotRow = row;
            }
        }
        if (pivotMag < 1e-12) return null;
        if (pivotRow !== col) [M[col], M[pivotRow]] = [M[pivotRow], M[col]];

        const pivot = asFinite(M[col][col]);
        for (let j = col; j <= n; j++) M[col][j] /= pivot;

        for (let row = 0; row < n; row++) {
            if (row === col) continue;
            const factor = asFinite(M[row][col]);
            if (Math.abs(factor) < 1e-18) continue;
            for (let j = col; j <= n; j++) {
                M[row][j] -= factor * M[col][j];
            }
        }
    }

    return M.map((row) => asFinite(row[n]));
}

function normalizeOrZero(v, tol = 1e-10) {
    const norm = vectorNorm(v);
    if (!(norm > tol)) return new Array(Array.isArray(v) ? v.length : 0).fill(0);
    return v.map((value) => asFinite(value) / norm);
}

export function buildManipulabilityVFIConstraints({
    qActive,
    activeLimits,
    activationThreshold = 0.05,
    extraConstraints = null
} = {}) {
    const n = Array.isArray(qActive) ? qActive.length : 0;
    const constraints = [];
    const threshold = Math.max(0, asFinite(activationThreshold, 0.05));

    if (threshold > 0 && Array.isArray(activeLimits) && n > 0) {
        for (let i = 0; i < n; i++) {
            const lim = activeLimits[i];
            if (!lim) continue;
            const qi = asFinite(qActive[i]);
            const dUp = Number.isFinite(lim.upper) ? Math.max(0, lim.upper - qi) : Infinity;
            const dLo = Number.isFinite(lim.lower) ? Math.max(0, qi - lim.lower) : Infinity;
            const dMin = Math.min(dUp, dLo);
            if (!Number.isFinite(dMin) || dMin >= threshold) continue;
            const row = new Array(n).fill(0);
            row[i] = 1;
            constraints.push({ row, distance: dMin, threshold, source: 'joint_limit' });
        }
    }

    if (Array.isArray(extraConstraints)) {
        for (const item of extraConstraints) {
            if (!Array.isArray(item?.row) || item.row.length !== n) continue;
            const distance = asFinite(item.distance, Number.NaN);
            const itemThreshold = asFinite(item.threshold, Number.NaN);
            if (!Number.isFinite(distance) || !Number.isFinite(itemThreshold)) continue;
            constraints.push({
                row: item.row.map((value) => asFinite(value)),
                distance,
                threshold: itemThreshold,
                source: typeof item?.source === 'string' && item.source.trim()
                    ? item.source.trim()
                    : 'extra'
            });
        }
    }

    return constraints;
}

export function summarizeManipulabilityVFIConstraints(constraints = []) {
    let jointLimitCount = 0;
    let extraCount = 0;
    let minDistance = Number.POSITIVE_INFINITY;
    for (const item of Array.isArray(constraints) ? constraints : []) {
        if (item?.source === 'joint_limit') {
            jointLimitCount += 1;
        } else {
            extraCount += 1;
        }
        const distance = asFinite(item?.distance, Number.NaN);
        if (Number.isFinite(distance)) {
            minDistance = Math.min(minDistance, distance);
        }
    }
    return {
        jointLimitCount,
        extraCount,
        totalCount: jointLimitCount + extraCount,
        minDistance: Number.isFinite(minDistance) ? minDistance : Number.NaN
    };
}

export function projectSecondaryGradient(gradient, protectRows = [], options = {}) {
    const grad = Array.isArray(gradient) ? gradient.map((value) => asFinite(value)) : [];
    if (grad.length === 0) return [];

    const normalize = options.normalize !== false;
    const damping = Math.max(1e-12, asFinite(options.damping, 1e-6));
    const absoluteZeroTolerance = Math.max(0, asFinite(options.absoluteZeroTolerance, 1e-10));
    const relativeZeroTolerance = Math.max(0, asFinite(options.relativeZeroTolerance, 1e-6));
    const rows = (Array.isArray(protectRows) ? protectRows : [])
        .filter((row) => Array.isArray(row) && row.length === grad.length);

    if (rows.length === 0) {
        return normalize ? normalizeOrZero(grad) : grad;
    }

    const rhs = matVec(rows, grad);
    const gramBase = rows.map((rowI) => rows.map((rowJ) => dot(rowI, rowJ)));
    let alpha = solveLinearSystem(gramBase, rhs);
    if (!Array.isArray(alpha) || alpha.length !== rows.length) {
        const gramDamped = gramBase.map((row, i) => row.map((value, j) => (
            i === j ? value + damping : value
        )));
        alpha = solveLinearSystem(gramDamped, rhs);
    }
    if (!Array.isArray(alpha) || alpha.length !== rows.length) {
        return new Array(grad.length).fill(0);
    }

    const rowSpaceComponent = new Array(grad.length).fill(0);
    for (let i = 0; i < rows.length; i++) {
        const scale = asFinite(alpha[i]);
        const row = rows[i];
        for (let j = 0; j < grad.length; j++) {
            rowSpaceComponent[j] += scale * asFinite(row[j]);
        }
    }

    const projected = grad.map((value, index) => value - rowSpaceComponent[index]);
    const projectedNorm = vectorNorm(projected);
    const rawNorm = vectorNorm(grad);
    if (projectedNorm <= Math.max(absoluteZeroTolerance, rawNorm * relativeZeroTolerance)) {
        return new Array(grad.length).fill(0);
    }
    return normalize ? normalizeOrZero(projected) : projected;
}

export function computeProtectedSecondaryGradient({
    evalFn,
    qActive,
    protectRows = [],
    eps = 1e-5,
    damping = 1e-6,
    normalize = true
} = {}) {
    const q = Array.isArray(qActive) ? qActive.slice() : [];
    if (q.length === 0 || typeof evalFn !== 'function') return [];
    const rawGradient = gradientVFICCTSM(evalFn, q, eps);
    return projectSecondaryGradient(rawGradient, protectRows, { damping, normalize });
}
