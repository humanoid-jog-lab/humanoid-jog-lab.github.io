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

function projectStateToBounds(q, bounds = {}) {
    const n = Array.isArray(q) ? q.length : 0;
    const projected = new Array(n).fill(0);
    for (let i = 0; i < n; i++) {
        const lower = Number.isFinite(bounds?.effectiveLower?.[i]) ? bounds.effectiveLower[i] : Number.NEGATIVE_INFINITY;
        const upper = Number.isFinite(bounds?.effectiveUpper?.[i]) ? bounds.effectiveUpper[i] : Number.POSITIVE_INFINITY;
        projected[i] = Math.min(upper, Math.max(lower, asFinite(q[i])));
    }
    return projected;
}

function addScaledStep(q, dq, scale) {
    return q.map((value, index) => asFinite(value) + scale * asFinite(dq?.[index]));
}

function multiplyRowsVector(rows, vector) {
    return (Array.isArray(rows) ? rows : []).map((row) => dot(row, vector));
}

function totalInequalityViolation(problem) {
    return Math.max(
        asFinite(problem?.inequalities?.wvfiFloor?.violation, 0),
        asFinite(problem?.inequalities?.waistXCorridor?.violation, 0)
    );
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
        const violation = dot(rows[i], dqStep) - asFinite(rhs[i], 0);
        if (violation > maxViolation) maxViolation = violation;
    }
    return maxViolation;
}

function clipDamping(residual, dampingTerm) {
    const r = asFinite(residual, 0);
    const d = asFinite(dampingTerm, 0);
    if (Math.abs(r) <= 1e-18) return 0;
    if (Math.sign(r) !== Math.sign(d)) return d;
    const maxMagnitude = Math.abs(r) * 0.8;
    return Math.sign(d) * Math.min(Math.abs(d), maxMagnitude);
}

function getActiveResidualNorm(problem = {}) {
    return asFinite(
        problem?.equalities?.activeResidualNorm,
        asFinite(problem?.equalities?.residualNorm, vectorNorm(problem?.equalities?.residual))
    );
}

function normalizeRegularizationWeights(regularizationWeights, width = 0) {
    if (!Array.isArray(regularizationWeights) || width <= 0) return null;
    const weights = new Array(width).fill(1);
    let hasCustomWeight = false;
    for (let i = 0; i < width; i++) {
        const weight = Math.max(1e-12, asFinite(regularizationWeights[i], 1));
        weights[i] = weight;
        if (Math.abs(weight - 1) > 1e-12) hasCustomWeight = true;
    }
    return hasCustomWeight ? weights : null;
}

function buildReferenceResidual(problem = {}, previousStep = []) {
    const residual = Array.isArray(problem?.equalities?.weightedResidual)
        ? problem.equalities.weightedResidual
        : [];
    const jacobian = Array.isArray(problem?.equalities?.weightedJacobian)
        ? problem.equalities.weightedJacobian
        : [];
    const stiffnessRows = Array.isArray(problem?.referenceModel?.stiffnessRows)
        ? problem.referenceModel.stiffnessRows
        : new Array(residual.length).fill(1);
    const dampingRows = Array.isArray(problem?.referenceModel?.dampingRows)
        ? problem.referenceModel.dampingRows
        : new Array(residual.length).fill(0);
    const constraintMotion = multiplyRowsVector(jacobian, previousStep);
    const referenceResidual = residual.map((value, index) => {
        const stiffValue = Math.max(0, asFinite(stiffnessRows[index], 1));
        const dampValue = Math.max(0, asFinite(dampingRows[index], 0));
        const base = stiffValue * asFinite(value);
        const dampingTerm = dampValue * asFinite(constraintMotion[index]);
        return base - clipDamping(base, dampingTerm);
    });
    return { referenceResidual, constraintMotion };
}

function buildActiveCorridorRows(problem = {}, width = 0) {
    const corridor = problem?.inequalities?.waistXCorridor;
    const rows = Array.isArray(corridor?.vfiRows) ? corridor.vfiRows : [];
    const rhs = Array.isArray(corridor?.rhs) ? corridor.rhs : [];
    if (rows.length === 0 || rows.length !== rhs.length) {
        return { rows: [], rhs: [] };
    }
    const activeRows = [];
    const activeRhs = [];
    for (let i = 0; i < rows.length; i++) {
        const row = Array.isArray(rows[i]) ? rows[i] : null;
        if (!row || row.length !== width) continue;
        activeRows.push(row.slice());
        activeRhs.push(asFinite(rhs[i], 0));
    }
    return { rows: activeRows, rhs: activeRhs };
}

function buildNormalEquations(
    jacobian = [],
    referenceResidual = [],
    width = 0,
    damping = 1e-6,
    regularizationWeights = null
) {
    const H = Array.from({ length: width }, (_, row) =>
        Array.from({ length: width }, (_, col) => (
            row === col
                ? Math.max(1e-12, damping * Math.max(1e-12, asFinite(regularizationWeights?.[row], 1)))
                : 0
        ))
    );
    const g = new Array(width).fill(0);
    for (let r = 0; r < jacobian.length; r++) {
        const row = Array.isArray(jacobian[r]) ? jacobian[r] : [];
        const residual = asFinite(referenceResidual[r], 0);
        for (let i = 0; i < width; i++) {
            const Ji = asFinite(row[i], 0);
            if (!Ji) continue;
            g[i] += Ji * residual;
            for (let j = 0; j < width; j++) {
                H[i][j] += Ji * asFinite(row[j], 0);
            }
        }
    }
    return { H, g };
}

function solveUnconstrainedLeastSquares(
    jacobian = [],
    referenceResidual = [],
    width = 0,
    damping = 1e-6,
    regularizationWeights = null
) {
    const { H, g } = buildNormalEquations(jacobian, referenceResidual, width, damping, regularizationWeights);
    return solveLinearSystem(H, g);
}

function selectViolatedCorridorRows(step = [], rows = [], rhs = []) {
    const activeRows = [];
    const activeRhs = [];
    for (let i = 0; i < rows.length; i++) {
        const row = Array.isArray(rows[i]) ? rows[i] : null;
        if (!row || row.length !== step.length) continue;
        if (dot(row, step) > asFinite(rhs[i], 0) + 1e-12) {
            activeRows.push(row.slice());
            activeRhs.push(asFinite(rhs[i], 0));
        }
    }
    return { rows: activeRows, rhs: activeRhs };
}

function solveEqualityConstrainedLeastSquares({
    jacobian = [],
    referenceResidual = [],
    width = 0,
    damping = 1e-6,
    regularizationWeights = null,
    activeRows = [],
    activeRhs = []
} = {}) {
    const constraintCount = Array.isArray(activeRows) ? activeRows.length : 0;
    if (constraintCount <= 0) {
        return solveUnconstrainedLeastSquares(jacobian, referenceResidual, width, damping, regularizationWeights);
    }
    const { H, g } = buildNormalEquations(jacobian, referenceResidual, width, damping, regularizationWeights);
    const dim = width + constraintCount;
    const kkt = Array.from({ length: dim }, () => new Array(dim).fill(0));
    const rhs = new Array(dim).fill(0);
    for (let i = 0; i < width; i++) {
        rhs[i] = g[i];
        for (let j = 0; j < width; j++) kkt[i][j] = H[i][j];
    }
    for (let c = 0; c < constraintCount; c++) {
        const row = activeRows[c];
        rhs[width + c] = asFinite(activeRhs[c], 0);
        for (let j = 0; j < width; j++) {
            const value = asFinite(row[j], 0);
            kkt[j][width + c] = value;
            kkt[width + c][j] = value;
        }
    }
    const solution = solveLinearSystem(kkt, rhs);
    return Array.isArray(solution) ? solution.slice(0, width) : null;
}

function buildRestorationStep(problem = {}, previousStep = [], damping = 1e-6, regularizationWeights = null) {
    const jacobian = Array.isArray(problem?.equalities?.weightedJacobian)
        ? problem.equalities.weightedJacobian
        : [];
    const width = Array.isArray(problem?.q) ? problem.q.length : 0;
    if (jacobian.length === 0 || width === 0) {
        return { dq: new Array(width).fill(0), referenceResidual: [], constraintMotion: [] };
    }
    const { referenceResidual, constraintMotion } = buildReferenceResidual(problem, previousStep);
    const corridorRows = buildActiveCorridorRows(problem, width);
    const unconstrainedDQ = solveUnconstrainedLeastSquares(
        jacobian,
        referenceResidual,
        width,
        damping,
        regularizationWeights
    );
    if (!Array.isArray(unconstrainedDQ) || unconstrainedDQ.length !== width) {
        return { dq: null, referenceResidual, constraintMotion };
    }
    const activeCorridor = selectViolatedCorridorRows(unconstrainedDQ, corridorRows.rows, corridorRows.rhs);
    if (activeCorridor.rows.length === 0) {
        return { dq: unconstrainedDQ, referenceResidual, constraintMotion };
    }
    const constrainedDQ = solveEqualityConstrainedLeastSquares({
        jacobian,
        referenceResidual,
        width,
        damping,
        regularizationWeights,
        activeRows: activeCorridor.rows,
        activeRhs: activeCorridor.rhs
    });
    return {
        dq: Array.isArray(constrainedDQ) && constrainedDQ.length === width ? constrainedDQ : unconstrainedDQ,
        referenceResidual,
        constraintMotion
    };
}

export function restoreHumanoidJogMJ({
    qInitial,
    evaluateProblem,
    maxIterations = 16,
    damping = 1e-6,
    regularizationWeights = null,
    maxDampingRetries = 8,
    dampingGrowthFactor = 10,
    initialScale = 1,
    minScale = 1e-6,
    reductionFactor = 0.5,
    equalityTolerance = 1e-12,
    inequalityTolerance = 1e-12
} = {}) {
    const qStart = Array.isArray(qInitial) ? qInitial.slice() : [];
    const normalizedRegularizationWeights = normalizeRegularizationWeights(regularizationWeights, qStart.length);
    const result = {
        converged: false,
        reason: 'invalid_input',
        qApplied: qStart.slice(),
        iterations: 0,
        acceptedIterations: 0,
        lastAcceptedScale: 0,
        lastAcceptedDamping: Number.NaN,
        currentProblem: null,
        currentResidualNorm: Number.NaN,
        lastReferenceResidual: [],
        lastConstraintMotion: []
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
    const retryCount = Math.max(0, Math.floor(asFinite(maxDampingRetries, 8)));
    const dampingGrowth = Math.max(1.01, asFinite(dampingGrowthFactor, 10));

    for (let iter = 0; iter < maxIters; iter++) {
        result.iterations = iter + 1;
        let acceptedTrial = null;
        let lastReferenceResidual = [];
        let lastConstraintMotion = [];
        let acceptedDamping = Number.NaN;
        let acceptedReferenceResidual = [];
        let acceptedConstraintMotion = [];

        for (let dampingAttempt = 0; dampingAttempt <= retryCount; dampingAttempt++) {
            const attemptDamping = damping * (dampingGrowth ** dampingAttempt);
            const { dq, referenceResidual, constraintMotion } = buildRestorationStep(
                currentProblem,
                previousAcceptedStep,
                attemptDamping,
                normalizedRegularizationWeights
            );
            lastReferenceResidual = Array.isArray(referenceResidual) ? referenceResidual.slice() : [];
            lastConstraintMotion = Array.isArray(constraintMotion) ? constraintMotion.slice() : [];
            if (!Array.isArray(dq) || dq.length !== qCurrent.length || vectorNorm(dq) <= 1e-18) {
                continue;
            }

            for (let scale = scale0; scale >= minStepScale; scale *= shrink) {
                const dqTrial = dq.map((value) => scale * asFinite(value));
                if (stepInequalityViolation(currentProblem, dqTrial) > inequalityTolerance) {
                    if (scale <= minStepScale) break;
                    continue;
                }
                const qTrial = projectStateToBounds(addScaledStep(qCurrent, dq, scale), currentProblem?.bounds);
                const trialProblem = evaluateProblem(qTrial);
                if (!trialProblem || typeof trialProblem !== 'object') continue;
                const trialResidualNorm = getActiveResidualNorm(trialProblem);
                if (trialResidualNorm + improvementTolerance < currentResidualNorm
                    && totalInequalityViolation(trialProblem) <= inequalityTolerance) {
                    const candidate = { scale, qTrial, trialProblem, trialResidualNorm, dqTrial };
                    const shouldReplace = !acceptedTrial
                        || trialResidualNorm + improvementTolerance < acceptedTrial.trialResidualNorm;
                    if (shouldReplace) {
                        acceptedTrial = candidate;
                        acceptedDamping = attemptDamping;
                        acceptedReferenceResidual = referenceResidual.slice();
                        acceptedConstraintMotion = constraintMotion.slice();
                    }
                    break;
                }
            }
        }

        result.lastReferenceResidual = acceptedTrial ? acceptedReferenceResidual : lastReferenceResidual;
        result.lastConstraintMotion = acceptedTrial ? acceptedConstraintMotion : lastConstraintMotion;

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
        previousAcceptedStep = acceptedTrial.dqTrial.slice();
        qAccepted = qCurrent.slice();
        acceptedProblem = currentProblem;
        acceptedResidualNorm = currentResidualNorm;
        hasAccepted = true;
        result.acceptedIterations += 1;
        result.lastAcceptedScale = acceptedTrial.scale;
        result.lastAcceptedDamping = acceptedDamping;
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
