import numeric from 'numeric';

function asNumber(value, fallback = 0) {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
}

function getIndexed(value, index, fallback = 0) {
    if (Array.isArray(value)) {
        return asNumber(value[index], fallback);
    }
    return asNumber(value, fallback);
}

function dot(a, b) {
    let sum = 0;
    const n = Math.min(a.length, b.length);
    for (let i = 0; i < n; i++) {
        sum += (a[i] || 0) * (b[i] || 0);
    }
    return sum;
}

function withUnitAt(index, n, scale = 1) {
    const row = new Array(n).fill(0);
    row[index] = scale;
    return row;
}

export function appendBandConstraints(A, b, J, target, band) {
    if (!Array.isArray(J) || J.length === 0) return;
    for (let i = 0; i < J.length; i++) {
        const row = Array.isArray(J[i]) ? [...J[i]] : [];
        if (row.length === 0) continue;

        const rowTarget = getIndexed(target, i, 0);
        const rowBand = Math.max(0, getIndexed(band, i, 0));
        const upper = rowTarget + rowBand;
        const lower = rowTarget - rowBand;

        A.push(row);
        b.push(upper);
        A.push(row.map((v) => -v));
        b.push(-lower);
    }
}

// Build VFI-like component-wise tube constraints for an error vector e.
// Keep each scalar error inside [-margin_i, margin_i] with exponential dynamics:
//   e_dot <= eta * (margin - e)
//  -e_dot <= eta * (margin + e)
// where e_dot = J_i * dq + zeta_i.
export function appendVFIErrorTubeConstraints(A, b, J, error, options = {}) {
    if (!Array.isArray(J) || J.length === 0) return;
    const eta = Math.max(0, asNumber(options.eta, 0.8));
    const marginBase = Math.max(0, asNumber(options.marginBase, 1e-4));
    const marginGain = Math.max(0, asNumber(options.marginGain, 0.0));
    const zeta = options.zeta;

    for (let i = 0; i < J.length; i++) {
        const row = Array.isArray(J[i]) ? [...J[i]] : [];
        if (row.length === 0) continue;
        const e = getIndexed(error, i, 0);
        const zi = getIndexed(zeta, i, 0);
        const margin = marginBase + marginGain * Math.abs(e);

        // J_i dq <= eta * (margin - e) - zeta_i
        A.push(row);
        b.push(eta * (margin - e) - zi);

        // -J_i dq <= eta * (margin + e) + zeta_i
        A.push(row.map((v) => -v));
        b.push(eta * (margin + e) + zi);
    }
}

// Append linear equalities W2 x = w2 using two-sided inequalities:
//   W2 x <= w2 + tol
//  -W2 x <= -w2 + tol
export function appendEqualityConstraints(A, b, W2, w2, options = {}) {
    if (!Array.isArray(W2) || W2.length === 0) return;
    const tol = Math.max(0, asNumber(options.tolerance, 0));
    for (let i = 0; i < W2.length; i++) {
        const row = Array.isArray(W2[i]) ? [...W2[i]] : [];
        if (row.length === 0) continue;
        const rhs = getIndexed(w2, i, 0);
        A.push(row);
        b.push(rhs + tol);
        A.push(row.map((v) => -v));
        b.push(-rhs + tol);
    }
}

export function appendComponentBounds(A, b, n, lowerBound, upperBound) {
    if (!Number.isFinite(n) || n <= 0) return;
    for (let i = 0; i < n; i++) {
        const ub = getIndexed(upperBound, i, Number.POSITIVE_INFINITY);
        const lb = getIndexed(lowerBound, i, Number.NEGATIVE_INFINITY);

        if (Number.isFinite(ub)) {
            A.push(withUnitAt(i, n, 1));
            b.push(ub);
        }
        if (Number.isFinite(lb)) {
            A.push(withUnitAt(i, n, -1));
            b.push(-lb);
        }
    }
}

export function appendJointLimitStepConstraints(A, b, q, limits, options = {}) {
    if (!Array.isArray(q) || !Array.isArray(limits) || q.length === 0) return;
    const n = q.length;
    const eta = Math.max(0, asNumber(options.eta, 1));
    const maxStep = Math.max(0, asNumber(options.maxStep, Number.POSITIVE_INFINITY));

    for (let i = 0; i < n; i++) {
        const lim = limits[i];
        if (!lim) continue;
        const lower = asNumber(lim.lower, Number.NEGATIVE_INFINITY);
        const upper = asNumber(lim.upper, Number.POSITIVE_INFINITY);
        const qi = asNumber(q[i], 0);

        const upperSlack = Math.max(0, upper - qi);
        const lowerSlack = Math.max(0, qi - lower);

        const upperStep = Math.min(maxStep, eta * upperSlack);
        const lowerStep = Math.min(maxStep, eta * lowerSlack);

        A.push(withUnitAt(i, n, 1));
        b.push(upperStep);
        A.push(withUnitAt(i, n, -1));
        b.push(lowerStep);
    }
}

export function projectOntoHalfspaces(initialX, A, b, options = {}) {
    if (!Array.isArray(initialX)) return [];
    if (!Array.isArray(A) || !Array.isArray(b) || A.length === 0) return [...initialX];

    const x = [...initialX];
    const passes = Math.max(1, asNumber(options.passes, 6));
    const tolerance = Math.max(0, asNumber(options.tolerance, 1e-10));
    const eps = 1e-12;

    for (let p = 0; p < passes; p++) {
        let anyUpdate = false;
        for (let i = 0; i < A.length; i++) {
            const row = A[i];
            if (!Array.isArray(row) || row.length === 0) continue;
            const rhs = asNumber(b[i], Number.POSITIVE_INFINITY);
            const violation = dot(row, x) - rhs;
            if (violation <= tolerance) continue;

            const norm2 = dot(row, row);
            if (norm2 <= eps) continue;
            const scale = violation / norm2;
            for (let j = 0; j < x.length && j < row.length; j++) {
                x[j] -= scale * row[j];
            }
            anyUpdate = true;
        }
        if (!anyUpdate) break;
    }
    return x;
}

function matVec(M, x) {
    const out = new Array(Array.isArray(M) ? M.length : 0).fill(0);
    for (let i = 0; i < out.length; i++) {
        const row = Array.isArray(M[i]) ? M[i] : [];
        let sum = 0;
        const n = Math.min(row.length, x.length);
        for (let j = 0; j < n; j++) sum += (row[j] || 0) * (x[j] || 0);
        out[i] = sum;
    }
    return out;
}

function vectorNorm2(v) {
    if (!Array.isArray(v) || v.length === 0) return 0;
    let s = 0;
    for (let i = 0; i < v.length; i++) s += (v[i] || 0) * (v[i] || 0);
    return s;
}

function estimateLipschitz(H, n, options = {}) {
    if (!Array.isArray(H) || H.length === 0 || n <= 0) return 1;
    const powerIters = Math.max(2, asNumber(options.powerIters, 12));
    const eps = 1e-12;
    let v = new Array(n).fill(1 / Math.sqrt(n));
    let lambda = 1;
    for (let k = 0; k < powerIters; k++) {
        const Hv = matVec(H, v);
        const hvNorm = Math.sqrt(vectorNorm2(Hv));
        if (!Number.isFinite(hvNorm) || hvNorm <= eps) break;
        for (let i = 0; i < n; i++) v[i] = (Hv[i] || 0) / hvNorm;
        lambda = Math.max(lambda, dot(v, matVec(H, v)));
    }
    if (!Number.isFinite(lambda) || lambda <= eps) return 1;
    return lambda;
}

function evaluateQuadraticObjective(H, f, x) {
    const Hx = matVec(H, x);
    return 0.5 * dot(x, Hx) + dot(f, x);
}

function maxHalfspaceViolation(A, b, x) {
    let maxViolation = 0;
    const rows = Array.isArray(A) ? A.length : 0;
    for (let i = 0; i < rows; i++) {
        const row = Array.isArray(A?.[i]) ? A[i] : [];
        const rhs = asNumber(b?.[i], Number.POSITIVE_INFINITY);
        const violation = dot(row, x) - rhs;
        if (violation > maxViolation) maxViolation = violation;
    }
    return maxViolation;
}

function toDenseSymmetricMatrix(H, n) {
    const out = new Array(n).fill(0).map(() => new Array(n).fill(0));
    for (let i = 0; i < n; i++) {
        for (let j = 0; j < n; j++) {
            const hij = asNumber(H?.[i]?.[j], 0);
            const hji = asNumber(H?.[j]?.[i], hij);
            out[i][j] = 0.5 * (hij + hji);
        }
    }
    return out;
}

function buildNumericHalfspaceColumns(A, b, n) {
    const m = Array.isArray(A) ? A.length : 0;
    const Amat = new Array(n).fill(0).map(() => new Array(m).fill(0));
    const bvec = new Array(m).fill(0);
    for (let i = 0; i < m; i++) {
        const row = Array.isArray(A?.[i]) ? A[i] : [];
        // numeric.solveQP expects A^T x >= b. Convert from row*x <= rhs:
        //   (-row)^T x >= -rhs
        bvec[i] = -asNumber(b?.[i], 0);
        for (let j = 0; j < n; j++) {
            Amat[j][i] = -(row[j] || 0);
        }
    }
    return { Amat, bvec };
}

export function solveQPNumericActiveSet(
    H,
    f,
    A,
    b,
    x0,
    options = {}
) {
    const n = Array.isArray(H) ? H.length : 0;
    if (n <= 0) {
        return {
            x: [],
            iterations: 0,
            converged: true,
            objective: 0,
            solverMessage: null,
            rawIterations: null
        };
    }

    const xSeed = Array.isArray(x0) ? x0.slice(0, n).map((v) => asNumber(v, 0)) : new Array(n).fill(0);
    if (xSeed.length < n) {
        for (let i = xSeed.length; i < n; i++) xSeed.push(0);
    }

    const hasIneq = Array.isArray(A) && A.length > 0;
    if (!hasIneq) {
        const fallback = solveQPProjectedGradient(H, f, [], [], xSeed, options);
        return {
            ...fallback,
            solverMessage: null,
            rawIterations: [fallback.iterations, 0]
        };
    }

    const D = toDenseSymmetricMatrix(H, n);
    const d = new Array(n).fill(0);
    for (let i = 0; i < n; i++) d[i] = -asNumber(f?.[i], 0);
    const { Amat, bvec } = buildNumericHalfspaceColumns(A, b, n);
    const solveQPBackend = typeof options.solveQPBackend === 'function'
        ? options.solveQPBackend
        : numeric.solveQP;
    const solutionTolerance = Math.max(0, asNumber(options.solutionTolerance, 1e-8));

    try {
        const result = solveQPBackend(D, d, Amat, bvec, 0);
        const candidate = Array.isArray(result?.solution) ? result.solution : xSeed;
        const x = new Array(n).fill(0);
        let finite = true;
        for (let i = 0; i < n; i++) {
            x[i] = asNumber(candidate[i], 0);
            if (!Number.isFinite(x[i])) finite = false;
        }
        const rawIterations = Array.isArray(result?.iterations) ? result.iterations.slice(0, 2) : null;
        const iterations = Number.isFinite(rawIterations?.[0]) ? rawIterations[0] : 0;
        const solverMessageRaw = typeof result?.message === 'string' ? result.message.trim() : '';
        const feasible = finite && maxHalfspaceViolation(A, b, x) <= solutionTolerance;
        const converged = feasible || (finite && solverMessageRaw.length === 0);
        if (!feasible) {
            const fallback = solveQPProjectedGradient(H, f, A, b, xSeed, options);
            const fallbackFeasible = Array.isArray(fallback?.x)
                && maxHalfspaceViolation(A, b, fallback.x) <= solutionTolerance;
            if (fallbackFeasible) {
                const fallbackMessage = solverMessageRaw
                    ? `${solverMessageRaw}; projected_gradient_fallback`
                    : 'projected_gradient_fallback';
                return {
                    x: fallback.x,
                    iterations: fallback.iterations,
                    converged: Boolean(fallback.converged),
                    objective: fallback.objective,
                    solverMessage: fallbackMessage,
                    rawIterations: [iterations, fallback.iterations]
                };
            }
        }
        return {
            x,
            iterations,
            converged,
            objective: evaluateQuadraticObjective(H, f, x),
            solverMessage: solverMessageRaw || null,
            rawIterations
        };
    } catch (error) {
        return {
            x: xSeed,
            iterations: 0,
            converged: false,
            objective: evaluateQuadraticObjective(H, f, xSeed),
            solverMessage: error instanceof Error ? error.message : String(error || 'numeric.solveQP failed'),
            rawIterations: null
        };
    }
}

export function buildQPNumericSolveOptions(config = {}) {
    const maxIter = Math.max(1, asNumber(config.qpMaxIter, 80));
    const stepTol = Math.max(0, asNumber(config.qpStepTol, 1e-7));
    const gradTol = Math.max(0, asNumber(config.qpGradTol, 1e-7));
    const solutionTolerance = Math.max(0, asNumber(config.qpSolutionTolerance, gradTol));

    return {
        maxIter,
        stepTol,
        gradTol,
        solutionTolerance
    };
}

export function solveQPProjectedGradient(
    H,
    f,
    A,
    b,
    x0,
    options = {}
) {
    const n = Array.isArray(H) ? H.length : 0;
    if (n <= 0) {
        return {
            x: [],
            iterations: 0,
            converged: true,
            objective: 0,
            stepSize: 1
        };
    }

    const maxIter = Math.max(1, asNumber(options.maxIter, 80));
    const stepTol = Math.max(0, asNumber(options.stepTol, 1e-7));
    const gradTol = Math.max(0, asNumber(options.gradTol, 1e-7));
    const projPasses = Math.max(1, asNumber(options.projectionPasses, 10));
    const projTol = Math.max(0, asNumber(options.projectionTolerance, 1e-10));
    const objTol = Math.max(0, asNumber(options.objectiveTolerance, 1e-11));

    let x = Array.isArray(x0) ? x0.slice(0, n).map((v) => asNumber(v, 0)) : new Array(n).fill(0);
    if (x.length < n) {
        for (let i = x.length; i < n; i++) x.push(0);
    }

    if (Array.isArray(A) && A.length > 0) {
        x = projectOntoHalfspaces(x, A, b, { passes: projPasses, tolerance: projTol });
    }

    const L = estimateLipschitz(H, n, options);
    const alpha = 1 / Math.max(1e-9, L);
    let prevObjective = evaluateQuadraticObjective(H, f, x);

    let converged = false;
    let iterations = 0;
    for (let iter = 0; iter < maxIter; iter++) {
        const grad = matVec(H, x);
        for (let i = 0; i < n; i++) grad[i] = (grad[i] || 0) + asNumber(f?.[i], 0);
        const gradNorm = Math.sqrt(vectorNorm2(grad));
        if (gradNorm <= gradTol) {
            converged = true;
            iterations = iter;
            break;
        }

        const trial = new Array(n).fill(0);
        for (let i = 0; i < n; i++) trial[i] = (x[i] || 0) - alpha * (grad[i] || 0);

        const projected = (Array.isArray(A) && A.length > 0)
            ? projectOntoHalfspaces(trial, A, b, { passes: projPasses, tolerance: projTol })
            : trial;

        const dx = new Array(n).fill(0);
        for (let i = 0; i < n; i++) dx[i] = (projected[i] || 0) - (x[i] || 0);
        const stepNorm = Math.sqrt(vectorNorm2(dx));
        x = projected;
        iterations = iter + 1;
        const objective = evaluateQuadraticObjective(H, f, x);
        const objectiveDelta = Math.abs(objective - prevObjective);
        prevObjective = objective;

        if (stepNorm <= stepTol * (1 + Math.sqrt(vectorNorm2(x)))) {
            converged = true;
            break;
        }
        if (objectiveDelta <= objTol * (1 + Math.abs(objective))) {
            converged = true;
            break;
        }
    }

    return {
        x,
        iterations,
        converged,
        objective: evaluateQuadraticObjective(H, f, x),
        stepSize: alpha
    };
}
