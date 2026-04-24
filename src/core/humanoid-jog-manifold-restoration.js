function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function defaultNowMs() {
    if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
        return performance.now();
    }
    return Date.now();
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

function multiplyRowsVector(rows, vector) {
    const src = Array.isArray(rows) ? rows : [];
    return src.map((row) => dot(row, vector));
}

function clipReferenceDampingTerm(residual, dampingTerm) {
    const residualValue = asFinite(residual, 0);
    const dampingValue = asFinite(dampingTerm, 0);
    if (Math.abs(residualValue) <= 1e-18) return 0;
    const sameDirection = Math.sign(residualValue) === Math.sign(dampingValue);
    if (!sameDirection) return dampingValue;
    const maxMagnitude = Math.max(0, Math.abs(residualValue) * 0.8);
    return Math.sign(dampingValue) * Math.min(Math.abs(dampingValue), maxMagnitude);
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

function projectStateToBounds(q, bounds = {}) {
    const n = Array.isArray(q) ? q.length : 0;
    const projected = new Array(n).fill(0);
    let clipped = false;
    for (let i = 0; i < n; i++) {
        const lower = Number.isFinite(bounds?.effectiveLower?.[i]) ? bounds.effectiveLower[i] : Number.NEGATIVE_INFINITY;
        const upper = Number.isFinite(bounds?.effectiveUpper?.[i]) ? bounds.effectiveUpper[i] : Number.POSITIVE_INFINITY;
        const clamped = Math.min(upper, Math.max(lower, asFinite(q[i])));
        projected[i] = clamped;
        if (Math.abs(clamped - asFinite(q[i])) > 1e-12) clipped = true;
    }
    return {
        qProjected: projected,
        clipped
    };
}

function addScaledStep(q, dq, scale) {
    const n = Array.isArray(q) ? q.length : 0;
    const out = new Array(n).fill(0);
    for (let i = 0; i < n; i++) out[i] = asFinite(q[i]) + scale * asFinite(dq?.[i]);
    return out;
}

function totalInequalityViolation(problem) {
    const wvfiViolation = asFinite(problem?.inequalities?.wvfiFloor?.violation, 0);
    const corridorViolation = asFinite(problem?.inequalities?.waistXCorridor?.violation, 0);
    return Math.max(wvfiViolation, corridorViolation);
}

function stepInequalityViolation(problem, dqStep) {
    const rows = Array.isArray(problem?.inequalities?.waistXCorridor?.vfiRows)
        ? problem.inequalities.waistXCorridor.vfiRows
        : [];
    const rhs = Array.isArray(problem?.inequalities?.waistXCorridor?.rhs)
        ? problem.inequalities.waistXCorridor.rhs
        : [];
    let maxViolation = 0;
    for (let i = 0; i < rows.length; i++) {
        const row = Array.isArray(rows[i]) ? rows[i] : null;
        if (!row) continue;
        const lhs = dot(row, dqStep);
        const violation = lhs - asFinite(rhs[i], 0);
        if (violation > maxViolation) maxViolation = violation;
    }
    return maxViolation;
}

export function buildHumanoidJogManifoldRestorationStep(problem = {}, options = {}) {
    const residual = Array.isArray(problem?.equalities?.weightedResidual)
        ? problem.equalities.weightedResidual
        : (Array.isArray(problem?.equalities?.residual) ? problem.equalities.residual : []);
    const jacobian = Array.isArray(problem?.equalities?.weightedJacobian)
        ? problem.equalities.weightedJacobian
        : (Array.isArray(problem?.equalities?.jacobian) ? problem.equalities.jacobian : []);
    const rowCount = residual.length;
    const width = Array.isArray(problem?.q) ? problem.q.length : 0;
    if (rowCount === 0 || width === 0) return new Array(width).fill(0);
    const damping = Math.max(1e-12, asFinite(options.damping, 1e-6));
    const referenceStiffness = Math.max(0, asFinite(options.referenceStiffness, 1));
    const referenceDamping = Math.max(0, asFinite(options.referenceDamping, 0));
    const previousStep = Array.isArray(options.previousStep) ? options.previousStep : new Array(width).fill(0);
    const projectedPreviousStep = multiplyRowsVector(jacobian, previousStep);
    const referenceResidual = residual.map((value, index) =>
        referenceStiffness * asFinite(value) - clipReferenceDampingTerm(
            referenceStiffness * asFinite(value),
            referenceDamping * asFinite(projectedPreviousStep[index])
        )
    );
    const gram = Array.from({ length: rowCount }, (_, i) =>
        Array.from({ length: rowCount }, (_, j) => {
            const value = dot(jacobian[i], jacobian[j]);
            return i === j ? value + damping : value;
        })
    );
    const alpha = solveLinearSystem(gram, referenceResidual);
    if (!Array.isArray(alpha) || alpha.length !== rowCount) return null;
    const dq = new Array(width).fill(0);
    for (let col = 0; col < width; col++) {
        let sum = 0;
        for (let row = 0; row < rowCount; row++) sum += asFinite(jacobian[row]?.[col]) * asFinite(alpha[row]);
        dq[col] = sum;
    }
    return dq;
}

function getActiveResidualNorm(problem = {}) {
    return asFinite(
        problem?.equalities?.activeResidualNorm,
        asFinite(problem?.equalities?.residualNorm, vectorNorm(problem?.equalities?.residual))
    );
}

export function restoreHumanoidJogManifold({
    qInitial,
    evaluateProblem,
    maxIterations = 16,
    damping = 1e-6,
    referenceStiffness = 1,
    referenceDamping = 0,
    initialScale = 1,
    minScale = 1e-6,
    reductionFactor = 0.5,
    equalityTolerance = 1e-12,
    inequalityTolerance = 1e-12,
    maxSolveMs = Number.POSITIVE_INFINITY,
    nowMs = defaultNowMs
} = {}) {
    const qStart = Array.isArray(qInitial) ? qInitial.slice() : [];
    const result = {
        converged: false,
        reason: 'invalid_input',
        qApplied: qStart.slice(),
        iterations: 0,
        acceptedIterations: 0,
        lastAcceptedScale: 0,
        lastStep: null,
        currentProblem: null,
        currentResidualNorm: Number.NaN,
        timeBudgetExceeded: false
    };
    if (typeof evaluateProblem !== 'function') return result;

    let qCurrent = qStart.slice();
    let currentProblem = evaluateProblem(qCurrent.slice());
    if (!currentProblem || typeof currentProblem !== 'object') {
        result.reason = 'problem_eval_failed';
        return result;
    }
    let currentResidualNorm = getActiveResidualNorm(currentProblem);
    result.currentProblem = currentProblem;
    result.currentResidualNorm = currentResidualNorm;

    if (currentResidualNorm <= equalityTolerance && totalInequalityViolation(currentProblem) <= inequalityTolerance) {
        result.converged = true;
        result.reason = 'converged';
        return result;
    }

    let qAccepted = qStart.slice();
    let acceptedProblem = currentProblem;
    let acceptedResidualNorm = currentResidualNorm;
    let hasAccepted = false;
    let previousAcceptedStep = new Array(qStart.length).fill(0);
    const maxIters = Math.max(1, Math.floor(asFinite(maxIterations, 16)));
    const scale0 = Math.max(minScale, asFinite(initialScale, 1));
    const minStepScale = Math.max(1e-8, asFinite(minScale, 1e-6));
    const shrink = Math.min(0.99, Math.max(0.01, asFinite(reductionFactor, 0.5)));
    const improvementTolerance = Math.max(1e-15, Math.min(1e-12, Math.max(0, equalityTolerance) * 1e-3));
    const solveClock = typeof nowMs === 'function' ? nowMs : defaultNowMs;
    const solveStartMs = solveClock();
    const rawSolveBudgetMs = Number(maxSolveMs);
    const solveBudgetMs = Number.isFinite(rawSolveBudgetMs)
        ? Math.max(0, rawSolveBudgetMs)
        : Number.POSITIVE_INFINITY;
    const hasSolveBudget = Number.isFinite(solveBudgetMs);
    const isSolveBudgetExceeded = () => (
        hasSolveBudget &&
        solveClock() - solveStartMs >= solveBudgetMs
    );
    const finishTimeBudget = () => {
        result.reason = hasAccepted ? 'best_effort_time_budget' : 'time_budget_exceeded';
        result.qApplied = hasAccepted ? qAccepted.slice() : qStart.slice();
        result.currentProblem = hasAccepted ? acceptedProblem : currentProblem;
        result.currentResidualNorm = hasAccepted ? acceptedResidualNorm : currentResidualNorm;
        result.timeBudgetExceeded = true;
        return result;
    };

    for (let iter = 0; iter < maxIters; iter++) {
        result.iterations = iter + 1;
        if (isSolveBudgetExceeded()) {
            return finishTimeBudget();
        }
        const dq = buildHumanoidJogManifoldRestorationStep(currentProblem, {
            damping,
            referenceStiffness,
            referenceDamping,
            previousStep: previousAcceptedStep
        });
        result.lastStep = Array.isArray(dq) ? dq.slice() : null;
        if (!Array.isArray(dq) || dq.length !== qCurrent.length || vectorNorm(dq) <= 1e-18) {
            result.reason = hasAccepted ? 'best_effort' : 'line_search_failed';
            result.qApplied = hasAccepted ? qAccepted.slice() : qStart.slice();
            result.currentProblem = hasAccepted ? acceptedProblem : currentProblem;
            result.currentResidualNorm = hasAccepted ? acceptedResidualNorm : currentResidualNorm;
            return result;
        }

        let acceptedTrial = null;
        for (let scale = scale0; scale >= minStepScale; scale *= shrink) {
            if (isSolveBudgetExceeded()) {
                return finishTimeBudget();
            }
            const dqTrial = dq.map((value) => scale * asFinite(value));
            if (stepInequalityViolation(currentProblem, dqTrial) > inequalityTolerance) {
                if (scale <= minStepScale) break;
                continue;
            }
            const qTrialRaw = addScaledStep(qCurrent, dq, scale);
            const { qProjected } = projectStateToBounds(qTrialRaw, currentProblem?.bounds);
            const trialProblem = evaluateProblem(qProjected);
            if (!trialProblem || typeof trialProblem !== 'object') {
                if (scale <= minStepScale) break;
                continue;
            }
            const trialResidualNorm = getActiveResidualNorm(trialProblem);
            const trialViolation = totalInequalityViolation(trialProblem);
            if (trialResidualNorm + improvementTolerance < currentResidualNorm && trialViolation <= inequalityTolerance) {
                acceptedTrial = {
                    scale,
                    qTrial: qProjected,
                    trialProblem,
                    trialResidualNorm
                };
                break;
            }
        }

        if (!acceptedTrial) {
            result.reason = hasAccepted ? 'best_effort' : 'line_search_failed';
            result.qApplied = hasAccepted ? qAccepted.slice() : qStart.slice();
            result.currentProblem = hasAccepted ? acceptedProblem : currentProblem;
            result.currentResidualNorm = hasAccepted ? acceptedResidualNorm : currentResidualNorm;
            return result;
        }

        qCurrent = acceptedTrial.qTrial.slice();
        currentProblem = acceptedTrial.trialProblem;
        currentResidualNorm = acceptedTrial.trialResidualNorm;
        previousAcceptedStep = dq.map((value) => acceptedTrial.scale * asFinite(value));
        qAccepted = qCurrent.slice();
        acceptedProblem = currentProblem;
        acceptedResidualNorm = currentResidualNorm;
        hasAccepted = true;
        result.acceptedIterations += 1;
        result.lastAcceptedScale = acceptedTrial.scale;
        result.currentProblem = currentProblem;
        result.currentResidualNorm = currentResidualNorm;

        if (currentResidualNorm <= equalityTolerance && totalInequalityViolation(currentProblem) <= inequalityTolerance) {
            result.converged = true;
            result.reason = 'converged';
            result.qApplied = qAccepted.slice();
            return result;
        }
    }

    result.reason = 'max_iter';
    result.qApplied = hasAccepted ? qAccepted.slice() : qStart.slice();
    result.currentProblem = hasAccepted ? acceptedProblem : currentProblem;
    result.currentResidualNorm = hasAccepted ? acceptedResidualNorm : currentResidualNorm;
    return result;
}
