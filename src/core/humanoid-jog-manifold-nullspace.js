function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function dot(a, b) {
    const n = Math.min(Array.isArray(a) ? a.length : 0, Array.isArray(b) ? b.length : 0);
    let sum = 0;
    for (let i = 0; i < n; i++) sum += asFinite(a[i]) * asFinite(b[i]);
    return sum;
}

function vectorNorm(values) {
    return Math.sqrt(dot(values, values));
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
            for (let j = col; j <= n; j++) M[row][j] -= factor * M[col][j];
        }
    }
    return M.map((row) => asFinite(row[n]));
}

function matVec(A, x) {
    return (Array.isArray(A) ? A : []).map((row) => dot(row, x));
}

function asStepVector(values, width) {
    const source = Array.isArray(values) ? values : [];
    const out = new Array(width).fill(0);
    const limit = Math.min(width, source.length);
    for (let i = 0; i < limit; i++) {
        out[i] = asFinite(source[i]);
    }
    return out;
}

function buildProjector(equalityJacobian, width, damping) {
    const rows = Array.isArray(equalityJacobian) ? equalityJacobian : [];
    if (rows.length === 0) {
        return Array.from({ length: width }, (_, row) =>
            Array.from({ length: width }, (_, col) => (row === col ? 1 : 0))
        );
    }
    const gram = rows.map((rowI, i) => rows.map((rowJ, j) => {
        const value = dot(rowI, rowJ);
        return i === j ? value + damping : value;
    }));
    const projector = Array.from({ length: width }, (_, row) =>
        Array.from({ length: width }, (_, col) => (row === col ? 1 : 0))
    );
    for (let col = 0; col < width; col++) {
        const rhs = rows.map((row) => asFinite(row?.[col]));
        const alpha = solveLinearSystem(gram, rhs);
        if (!Array.isArray(alpha) || alpha.length !== rows.length) continue;
        for (let row = 0; row < width; row++) {
            let correction = 0;
            for (let i = 0; i < rows.length; i++) correction += asFinite(rows[i]?.[row]) * asFinite(alpha[i]);
            projector[row][col] -= correction;
        }
    }
    return projector;
}

function clampToBounds(qCurrent, step, lowerBounds, upperBounds, stepBounds) {
    const n = Math.min(
        Array.isArray(qCurrent) ? qCurrent.length : 0,
        Array.isArray(step) ? step.length : 0
    );
    const qNext = new Array(n).fill(0);
    const dqClamped = new Array(n).fill(0);
    let clipped = false;
    for (let i = 0; i < n; i++) {
        const q0 = asFinite(qCurrent[i]);
        const lower = Number.isFinite(lowerBounds?.[i]) ? lowerBounds[i] : Number.NEGATIVE_INFINITY;
        const upper = Number.isFinite(upperBounds?.[i]) ? upperBounds[i] : Number.POSITIVE_INFINITY;
        const stepLimit = Number.isFinite(stepBounds?.[i]) ? Math.max(0, stepBounds[i]) : Number.POSITIVE_INFINITY;
        const qMin = Math.max(lower, q0 - stepLimit);
        const qMax = Math.min(upper, q0 + stepLimit);
        const qCandidate = q0 + asFinite(step[i]);
        const qClamped = Math.min(qMax, Math.max(qMin, qCandidate));
        qNext[i] = qClamped;
        dqClamped[i] = qClamped - q0;
        if (Math.abs(qClamped - qCandidate) > 1e-12) clipped = true;
    }
    return { qNext, step: dqClamped, clipped };
}

export function computeHumanoidJogManifoldNullspaceStep({
    qCurrent = [],
    equalityJacobian = [],
    objectiveGradient = [],
    wvfiGradient = [],
    stepSize = 1,
    lowerBounds = [],
    upperBounds = [],
    stepBounds = [],
    evaluateWVFI = null,
    wvfiMin = 0,
    preferWVFIFallbackOnClip = false,
    damping = 1e-12
} = {}) {
    const width = Array.isArray(qCurrent) ? qCurrent.length : 0;
    const zeroStep = new Array(width).fill(0);
    if (width === 0) {
        return { accepted: false, reason: 'invalid_input', step: zeroStep, qNext: Array.isArray(qCurrent) ? qCurrent.slice() : [] };
    }

    const projector = buildProjector(equalityJacobian, width, Math.max(1e-12, asFinite(damping, 1e-12)));
    const objectiveStep = asStepVector(objectiveGradient, width);
    const wvfiStep = asStepVector(wvfiGradient, width);
    const objectiveProjected = matVec(projector, objectiveStep);
    const objectiveProjectedNorm = vectorNorm(objectiveProjected);
    const wvfiProjected = matVec(projector, wvfiStep);
    const wvfiProjectedNorm = vectorNorm(wvfiProjected);

    const minimumProjectedNorm = 1e-10;
    let selected = null;
    let selectedSource = 'objective';

    if (objectiveProjectedNorm > minimumProjectedNorm) {
        selected = objectiveProjected;
    } else if (wvfiProjectedNorm > minimumProjectedNorm) {
        selected = wvfiProjected;
        selectedSource = 'wvfi';
    }

    if (!Array.isArray(selected) || selected.length === 0) {
        return {
            accepted: false,
            reason: 'no_nullspace',
            step: zeroStep,
            qNext: qCurrent.slice(),
            projector,
            projectedGradient: zeroStep,
            projectedGradientNorm: 0,
            gradientSource: 'none'
        };
    }

    const signedStepSize = asFinite(stepSize, 1);
    const buildCandidate = (projectedGradient, projectedNorm, source) => {
        if (!Array.isArray(projectedGradient) || projectedGradient.length === 0 || projectedNorm <= minimumProjectedNorm) {
            return null;
        }
        const scale = signedStepSize / projectedNorm;
        const rawStep = projectedGradient.map((value) => value * scale);
        const clamped = clampToBounds(qCurrent, rawStep, lowerBounds, upperBounds, stepBounds);
        if (typeof evaluateWVFI === 'function') {
            const wvfi = evaluateWVFI(clamped.qNext, clamped.step);
            if (Number.isFinite(wvfiMin) && wvfiMin > 0 && (!Number.isFinite(wvfi) || wvfi + 1e-12 < wvfiMin)) {
                return {
                    accepted: false,
                    reason: 'wvfi_lower_bound',
                    projector,
                    projectedGradient,
                    projectedGradientNorm: projectedNorm,
                    gradientSource: source
                };
            }
        }
        return {
            accepted: true,
            reason: clamped.clipped ? 'accepted_clipped' : 'accepted',
            gradientSource: source,
            step: clamped.step,
            qNext: clamped.qNext,
            projector,
            projectedGradient,
            projectedGradientNorm: projectedNorm,
            clipped: clamped.clipped
        };
    };

    const objectiveCandidate = selectedSource === 'objective'
        ? buildCandidate(objectiveProjected, objectiveProjectedNorm, 'objective')
        : null;
    const wvfiCandidate = buildCandidate(wvfiProjected, wvfiProjectedNorm, 'wvfi');

    let selectedCandidate = null;
    if (objectiveCandidate?.accepted === true) {
        selectedCandidate = objectiveCandidate;
        if (preferWVFIFallbackOnClip === true &&
            objectiveCandidate.clipped === true &&
            wvfiCandidate?.accepted === true) {
            selectedCandidate = wvfiCandidate;
        }
    } else if (wvfiCandidate?.accepted === true) {
        selectedCandidate = wvfiCandidate;
    }

    if (!selectedCandidate) {
        const rejected = objectiveCandidate || wvfiCandidate;
        return {
            accepted: false,
            reason: rejected?.reason ?? 'no_nullspace',
            step: zeroStep,
            qNext: qCurrent.slice(),
            projector,
            projectedGradient: selected,
            projectedGradientNorm: selectedSource === 'objective'
                ? objectiveProjectedNorm
                : wvfiProjectedNorm,
            gradientSource: rejected?.gradientSource ?? selectedSource
        };
    }

    return {
        ...selectedCandidate
    };
}
