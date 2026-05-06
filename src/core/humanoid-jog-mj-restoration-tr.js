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

function subtractRowspaceComponent(row = [], protectRows = []) {
    const validRows = (Array.isArray(protectRows) ? protectRows : [])
        .filter((item) => Array.isArray(item) && item.length === row.length);
    if (validRows.length === 0 || row.length === 0) return row.slice();
    const gram = validRows.map((rowI) => validRows.map((rowJ) => dot(rowI, rowJ)));
    const rhs = validRows.map((item) => dot(item, row));
    let alpha = solveLinearSystem(gram, rhs);
    if (!Array.isArray(alpha) || alpha.length !== validRows.length) {
        const dampedGram = gram.map((gramRow, i) => gramRow.map((value, j) => (
            i === j ? value + 1e-8 : value
        )));
        alpha = solveLinearSystem(dampedGram, rhs);
    }
    if (!Array.isArray(alpha) || alpha.length !== validRows.length) {
        return new Array(row.length).fill(0);
    }
    const projected = row.slice();
    for (let i = 0; i < validRows.length; i++) {
        const scale = asFinite(alpha[i], 0);
        for (let j = 0; j < row.length; j++) {
            projected[j] -= scale * asFinite(validRows[i][j], 0);
        }
    }
    return vectorNorm(projected) > 1e-10 ? projected : new Array(row.length).fill(0);
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

function getSafetyResidualNorm(problem = {}) {
    return asFinite(problem?.safetyObjective?.residualNorm, 0);
}

function getSecondaryResidualNorm(problem = {}) {
    return asFinite(problem?.secondaryObjective?.residualNorm, 0);
}

function buildSafetyLeastSquaresRows(problem = {}, weight = 0) {
    const objective = problem?.safetyObjective;
    return buildObjectiveLeastSquaresRows(problem, objective, weight);
}

function buildSecondaryLeastSquaresRows(problem = {}, weight = 0) {
    const objective = problem?.secondaryObjective;
    return buildObjectiveLeastSquaresRows(problem, objective, weight, {
        projectRows: problem?.equalities?.weightedJacobian,
        normalizeRows: true
    });
}

function buildObjectiveLeastSquaresRows(problem = {}, objective = null, weight = 0, options = {}) {
    const rows = Array.isArray(objective?.rows) ? objective.rows : [];
    const targets = Array.isArray(objective?.targets) ? objective.targets : [];
    const scale = Math.sqrt(Math.max(0, asFinite(weight, 0)));
    const width = Array.isArray(problem?.q) ? problem.q.length : 0;
    if (scale <= 0 || rows.length === 0 || rows.length !== targets.length || width <= 0) {
        return { rows: [], targets: [] };
    }
    const modelRows = [];
    const modelTargets = [];
    for (let i = 0; i < rows.length; i++) {
        const row = Array.isArray(rows[i]) ? rows[i] : null;
        if (!row || row.length !== width) continue;
        const projectedRow = Array.isArray(options?.projectRows)
            ? subtractRowspaceComponent(row.map((value) => asFinite(value, 0)), options.projectRows)
            : row.map((value) => asFinite(value, 0));
        const projectedNorm = vectorNorm(projectedRow);
        if (projectedNorm <= 1e-10) continue;
        const normalizedRow = options?.normalizeRows === true
            ? projectedRow.map((value) => value / projectedNorm)
            : projectedRow;
        modelRows.push(normalizedRow.map((value) => scale * asFinite(value, 0)));
        modelTargets.push(scale * Math.max(0, asFinite(targets[i], 0)));
    }
    return { rows: modelRows, targets: modelTargets };
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

function collectLinearStepInequalities(problem = {}, width = 0) {
    const sources = [
        {
            rows: problem?.inequalities?.jointLimitVFI?.rows,
            rhs: problem?.inequalities?.jointLimitVFI?.rhs
        },
        {
            rows: problem?.inequalities?.waistXCorridor?.vfiRows,
            rhs: problem?.inequalities?.waistXCorridor?.rhs
        }
    ];
    const activeRows = [];
    const activeRhs = [];
    for (const source of sources) {
        const rows = Array.isArray(source?.rows) ? source.rows : [];
        const rhs = Array.isArray(source?.rhs) ? source.rhs : [];
        if (rows.length === 0 || rows.length !== rhs.length) continue;
        for (let i = 0; i < rows.length; i++) {
            const row = Array.isArray(rows[i]) ? rows[i] : null;
            if (!row || row.length !== width) continue;
            activeRows.push(row.slice());
            activeRhs.push(asFinite(rhs[i], 0));
        }
    }
    if (activeRows.length === 0) {
        return { rows: [], rhs: [] };
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

function selectViolatedStepRows(step = [], rows = [], rhs = []) {
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

function buildRestorationStep(
    problem = {},
    previousStep = [],
    damping = 1e-6,
    regularizationWeights = null,
    safetyOptions = {},
    secondaryOptions = {}
) {
    const jacobian = Array.isArray(problem?.equalities?.weightedJacobian)
        ? problem.equalities.weightedJacobian
        : [];
    const width = Array.isArray(problem?.q) ? problem.q.length : 0;
    if (jacobian.length === 0 || width === 0) {
        return {
            dq: new Array(width).fill(0),
            referenceResidual: [],
            constraintMotion: [],
            modelJacobian: [],
            modelReferenceResidual: []
        };
    }
    const { referenceResidual, constraintMotion } = buildReferenceResidual(problem, previousStep);
    const safetyRows = safetyOptions?.enabled === true
        ? buildSafetyLeastSquaresRows(problem, safetyOptions.weight)
        : { rows: [], targets: [] };
    const secondaryRows = secondaryOptions?.enabled === true
        ? buildSecondaryLeastSquaresRows(problem, secondaryOptions.weight)
        : { rows: [], targets: [] };
    const modelJacobian = [
        ...jacobian,
        ...safetyRows.rows,
        ...secondaryRows.rows
    ];
    const modelReferenceResidual = [
        ...referenceResidual,
        ...safetyRows.targets,
        ...secondaryRows.targets
    ];
    const linearStepRows = collectLinearStepInequalities(problem, width);
    const unconstrainedDQ = solveUnconstrainedLeastSquares(
        modelJacobian,
        modelReferenceResidual,
        width,
        damping,
        regularizationWeights
    );
    if (!Array.isArray(unconstrainedDQ) || unconstrainedDQ.length !== width) {
        return { dq: null, referenceResidual, constraintMotion, modelJacobian, modelReferenceResidual };
    }
    const activeInequalities = selectViolatedStepRows(unconstrainedDQ, linearStepRows.rows, linearStepRows.rhs);
    if (activeInequalities.rows.length === 0) {
        return { dq: unconstrainedDQ, referenceResidual, constraintMotion, modelJacobian, modelReferenceResidual };
    }
    const constrainedDQ = solveEqualityConstrainedLeastSquares({
        jacobian: modelJacobian,
        referenceResidual: modelReferenceResidual,
        width,
        damping,
        regularizationWeights,
        activeRows: activeInequalities.rows,
        activeRhs: activeInequalities.rhs
    });
    return {
        dq: Array.isArray(constrainedDQ) && constrainedDQ.length === width ? constrainedDQ : unconstrainedDQ,
        referenceResidual,
        constraintMotion,
        modelJacobian,
        modelReferenceResidual
    };
}

function computeConstraintViolation(problem) {
    return totalInequalityViolation(problem);
}

function computeRestorationMerit(phi, theta, penalty) {
    return asFinite(phi) + penalty * asFinite(theta);
}

function computeModelObjective(
    problem = {},
    dqStep = [],
    damping = 1e-6,
    regularizationWeights = null,
    referenceResidual = null,
    modelJacobian = null
) {
    const jacobian = Array.isArray(modelJacobian)
        ? modelJacobian
        : (Array.isArray(problem?.equalities?.weightedJacobian)
        ? problem.equalities.weightedJacobian
        : []);
    const target = Array.isArray(referenceResidual)
        ? referenceResidual
        : (Array.isArray(problem?.equalities?.weightedResidual) ? problem.equalities.weightedResidual : []);
    let residualCost = 0;
    for (let rowIndex = 0; rowIndex < jacobian.length; rowIndex++) {
        const row = jacobian[rowIndex];
        const value = dot(row, dqStep) - asFinite(target[rowIndex], 0);
        residualCost += value * value;
    }
    let regularizationCost = 0;
    for (let i = 0; i < dqStep.length; i++) {
        const weight = Math.max(1e-12, asFinite(regularizationWeights?.[i], 1));
        regularizationCost += weight * asFinite(dqStep[i]) * asFinite(dqStep[i]);
    }
    return 0.5 * residualCost + 0.5 * Math.max(1e-12, asFinite(damping, 1e-6)) * regularizationCost;
}

function clipStepToTrustRegion(step = [], trustRadius = Number.POSITIVE_INFINITY) {
    const radius = Math.max(1e-12, asFinite(trustRadius, Number.POSITIVE_INFINITY));
    const stepNorm = vectorNorm(step);
    if (!Number.isFinite(radius) || stepNorm <= radius) {
        return {
            dq: step.slice(),
            stepNorm,
            clipped: false
        };
    }
    const scale = radius / Math.max(stepNorm, 1e-18);
    return {
        dq: step.map((value) => scale * asFinite(value)),
        stepNorm: radius,
        clipped: true
    };
}

function dominatesFilterEntry(lhs, rhs, phiTolerance, thetaTolerance) {
    const phiBetter = lhs.phi <= rhs.phi + phiTolerance;
    const thetaBetter = lhs.theta <= rhs.theta + thetaTolerance;
    const strictlyBetter = lhs.phi < rhs.phi - phiTolerance || lhs.theta < rhs.theta - thetaTolerance;
    return phiBetter && thetaBetter && strictlyBetter;
}

function isDominatedByFilter(filter, candidate, phiTolerance, thetaTolerance) {
    return filter.some((entry) => dominatesFilterEntry(entry, candidate, phiTolerance, thetaTolerance));
}

function insertFilterEntry(filter, candidate, phiTolerance, thetaTolerance) {
    const next = filter.filter((entry) => !dominatesFilterEntry(candidate, entry, phiTolerance, thetaTolerance));
    next.push({
        phi: candidate.phi,
        theta: candidate.theta
    });
    return next;
}

function acceptRestorationCandidateByFilter({
    filter,
    currentPhi,
    currentTheta,
    candidatePhi,
    candidateTheta,
    predictedReduction,
    actualReduction,
    rho,
    rhoAcceptanceThreshold,
    phiTolerance,
    thetaTolerance
}) {
    const candidate = {
        phi: candidatePhi,
        theta: candidateTheta
    };
    if (isDominatedByFilter(filter, candidate, phiTolerance, thetaTolerance)) {
        return { accepted: false, reason: 'dominated', nextFilter: filter };
    }
    const phiImproved = candidatePhi + phiTolerance < currentPhi;
    const thetaImproved = candidateTheta + thetaTolerance < currentTheta;
    const rhoAccepted = predictedReduction <= 1e-12 || rho >= rhoAcceptanceThreshold;
    if (!phiImproved && !thetaImproved) {
        return { accepted: false, reason: 'not_improved', nextFilter: filter };
    }
    if (!rhoAccepted && !thetaImproved) {
        return { accepted: false, reason: 'poor_ratio', nextFilter: filter };
    }
    return {
        accepted: true,
        reason: thetaImproved && !rhoAccepted
            ? 'filter'
            : (thetaImproved && !phiImproved ? 'filter' : 'rho'),
        nextFilter: insertFilterEntry(filter, candidate, phiTolerance, thetaTolerance),
        phiImproved,
        thetaImproved,
        predictedReduction: Math.max(0, predictedReduction),
        actualReduction
    };
}

function updateTrustRegionRadius(radius, rho, stepNorm, clipped, {
    minTrustRadius,
    maxTrustRadius,
    shrinkFactor,
    expandFactor,
    rhoShrinkThreshold,
    rhoExpandThreshold
}) {
    let nextRadius = radius;
    if (!Number.isFinite(rho) || rho < rhoShrinkThreshold) {
        nextRadius = Math.max(minTrustRadius, radius * shrinkFactor);
    } else if (clipped && rho > rhoExpandThreshold && stepNorm >= radius * 0.99) {
        nextRadius = Math.min(maxTrustRadius, radius * expandFactor);
    }
    return nextRadius;
}

export function restoreHumanoidJogMJTrustRegion({
    qInitial,
    evaluateProblem,
    maxIterations = 16,
    damping = 1e-6,
    regularizationWeights = null,
    maxDampingRetries = 8,
    dampingGrowthFactor = 10,
    equalityTolerance = 1e-12,
    inequalityTolerance = 1e-12,
    initialTrustRadius = 0.2,
    minTrustRadius = 1e-6,
    maxTrustRadius = 1.0,
    maxTrustRegionRetries = 6,
    trustRegionShrinkFactor = 0.5,
    trustRegionExpandFactor = 1.75,
    rhoAcceptanceThreshold = 0.05,
    rhoShrinkThreshold = 0.05,
    rhoExpandThreshold = 0.75,
    maxSolveMs = Number.POSITIVE_INFINITY,
    nowMs = defaultNowMs,
    safetyPushEnabled = true,
    safetyPushWeight = 0.05,
    safetyPushActivationResidual = 1e-5,
    safetyPushPoseHoldTolerance = 2e-6,
    safetyBiasReleaseResidual = null,
    safetyBiasDuringRestore = false,
    safetyPushPostConvergenceEnabled = true,
    secondaryObjectiveEnabled = false,
    secondaryObjectiveWeight = 0.002,
    secondaryObjectiveActivationResidual = 1e-5,
    secondaryObjectivePoseHoldTolerance = 2e-6,
    secondaryObjectiveDuringRestore = false,
    secondaryObjectivePostConvergenceEnabled = true
} = {}) {
    const qStart = Array.isArray(qInitial) ? qInitial.slice() : [];
    const normalizedRegularizationWeights = normalizeRegularizationWeights(regularizationWeights, qStart.length);
    const result = {
        converged: false,
        reason: 'invalid_input',
        qApplied: qStart.slice(),
        iterations: 0,
        acceptedIterations: 0,
        lastAcceptedScale: 1,
        lastAcceptedDamping: Number.NaN,
        currentProblem: null,
        currentResidualNorm: Number.NaN,
        lastReferenceResidual: [],
        lastConstraintMotion: [],
        trustRadius: Math.max(minTrustRadius, asFinite(initialTrustRadius, 0.2)),
        lastAcceptedRho: Number.NaN,
        lastPredictedReduction: Number.NaN,
        lastActualReduction: Number.NaN,
        acceptedBy: '',
        lastRejectedRho: Number.NaN,
        lastRejectedPredictedReduction: Number.NaN,
        lastRejectedActualReduction: Number.NaN,
        lastRejectedReason: '',
        bestConstraintViolation: Number.POSITIVE_INFINITY,
        filterSize: 1,
        timeBudgetExceeded: false,
        safetyStatus: 'skipped',
        safetyResidualBefore: Number.NaN,
        safetyResidualAfter: Number.NaN,
        safetyAcceptedIterations: 0,
        secondaryStatus: 'skipped',
        secondaryResidualBefore: Number.NaN,
        secondaryResidualAfter: Number.NaN,
        secondaryAcceptedIterations: 0
    };
    if (typeof evaluateProblem !== 'function') return result;

    const phiTolerance = Math.max(1e-12, Math.max(0, equalityTolerance) * 1e-3);
    const thetaTolerance = Math.max(1e-12, Math.max(0, inequalityTolerance) * 1e-3);
    const retryCount = Math.max(0, Math.floor(asFinite(maxDampingRetries, 8)));
    const dampingGrowth = Math.max(1.01, asFinite(dampingGrowthFactor, 10));
    const maxIters = Math.max(1, Math.floor(asFinite(maxIterations, 16)));
    const minRadius = Math.max(1e-8, asFinite(minTrustRadius, 1e-6));
    const maxRadius = Math.max(minRadius, asFinite(maxTrustRadius, 1.0));
    const maxRadiusRetries = Math.max(1, Math.floor(asFinite(maxTrustRegionRetries, 6)));
    const shrinkFactor = Math.min(0.99, Math.max(0.1, asFinite(trustRegionShrinkFactor, 0.5)));
    const expandFactor = Math.max(1.01, asFinite(trustRegionExpandFactor, 1.75));
    const solveClock = typeof nowMs === 'function' ? nowMs : defaultNowMs;
    const solveStartMs = solveClock();
    const rawSolveBudgetMs = Number(maxSolveMs);
    const solveBudgetMs = Number.isFinite(rawSolveBudgetMs)
        ? Math.max(0, rawSolveBudgetMs)
        : Number.POSITIVE_INFINITY;
    const hasSolveBudget = Number.isFinite(solveBudgetMs);
    const safetyWeight = Math.max(0, asFinite(safetyPushWeight, 0.05));
    const safetyActivationResidual = Math.max(0, asFinite(safetyPushActivationResidual, 1e-5));
    const safetyHoldTolerance = Math.max(
        Math.max(0, equalityTolerance),
        Math.max(0, asFinite(safetyPushPoseHoldTolerance, 2e-6))
    );
    const safetyBiasRelease = Math.max(
        safetyActivationResidual * 5,
        Math.max(0, equalityTolerance) * 50,
        safetyHoldTolerance * 2,
        Math.max(0, asFinite(safetyBiasReleaseResidual, 0))
    );
    const safetyResidualTolerance = 1e-12;
    const secondaryWeight = Math.max(0, asFinite(secondaryObjectiveWeight, 0.002));
    const secondaryActivationResidual = Math.max(0, asFinite(secondaryObjectiveActivationResidual, 1e-5));
    const secondaryHoldTolerance = Math.max(
        Math.max(0, equalityTolerance),
        Math.max(0, asFinite(secondaryObjectivePoseHoldTolerance, 2e-6))
    );
    const secondaryResidualTolerance = 1e-12;
    const getSafetyBiasWeight = (problem, phi) => {
        if (
            safetyPushEnabled === false ||
            safetyBiasDuringRestore !== true ||
            safetyWeight <= 0 ||
            getSafetyResidualNorm(problem) <= safetyResidualTolerance
        ) {
            return 0;
        }
        // Bias is allowed to guide the coarse TR step, but the final hard
        // tolerance must be owned by the primary pose/body/relative task.
        if (asFinite(phi, Number.POSITIVE_INFINITY) <= safetyBiasRelease) {
            return 0;
        }
        return safetyWeight;
    };
    const getSecondaryBiasWeight = (problem, phi, theta) => {
        if (
            secondaryObjectiveEnabled !== true ||
            secondaryObjectiveDuringRestore !== true ||
            secondaryWeight <= 0 ||
            getSecondaryResidualNorm(problem) <= secondaryResidualTolerance
        ) {
            return 0;
        }
        if (
            asFinite(phi, Number.POSITIVE_INFINITY) > secondaryActivationResidual ||
            asFinite(theta, Number.POSITIVE_INFINITY) > Math.max(0, inequalityTolerance) + thetaTolerance
        ) {
            return 0;
        }
        return secondaryWeight;
    };
    const isPrimaryHeldForSafety = (phi, theta) => (
        asFinite(phi, Number.POSITIVE_INFINITY) <= safetyHoldTolerance &&
        asFinite(theta, Number.POSITIVE_INFINITY) <= Math.max(0, inequalityTolerance) + thetaTolerance
    );
    const isPrimaryHeldForSecondary = (phi, theta) => (
        asFinite(phi, Number.POSITIVE_INFINITY) <= secondaryHoldTolerance &&
        asFinite(theta, Number.POSITIVE_INFINITY) <= Math.max(0, inequalityTolerance) + thetaTolerance
    );
    const isSafetyActive = (problem, phi, theta) => (
        safetyPushEnabled !== false &&
        safetyPushPostConvergenceEnabled !== false &&
        safetyWeight > 0 &&
        asFinite(phi, Number.POSITIVE_INFINITY) <= safetyActivationResidual &&
        asFinite(theta, Number.POSITIVE_INFINITY) <= Math.max(0, inequalityTolerance) + thetaTolerance &&
        getSafetyResidualNorm(problem) > safetyResidualTolerance
    );
    const isSecondaryActive = (problem, phi, theta) => (
        secondaryObjectiveEnabled === true &&
        secondaryObjectivePostConvergenceEnabled !== false &&
        secondaryWeight > 0 &&
        asFinite(phi, Number.POSITIVE_INFINITY) <= secondaryActivationResidual &&
        asFinite(theta, Number.POSITIVE_INFINITY) <= Math.max(0, inequalityTolerance) + thetaTolerance &&
        getSecondaryResidualNorm(problem) > secondaryResidualTolerance
    );
    const isSolveBudgetExceeded = () => (
        hasSolveBudget &&
        solveClock() - solveStartMs >= solveBudgetMs
    );
    const evaluateCurrentProblem = (qValue) => evaluateProblem(qValue.slice(), {
        evaluationRole: 'current',
        computeWVFISecondary: true
    });
    const evaluateTrialProblem = (qValue) => evaluateProblem(qValue.slice(), {
        evaluationRole: 'trial',
        computeWVFISecondary: false,
        computeManipData: false
    });
    const hasTrialSecondaryStatus = (problem = {}) => {
        const objective = problem?.secondaryObjective;
        return objective?.enabled === true ||
            (Array.isArray(objective?.residual) && objective.residual.length > 0);
    };

    let qCurrent = qStart.slice();
    let currentProblem = evaluateCurrentProblem(qCurrent);
    if (!currentProblem || typeof currentProblem !== 'object') {
        result.reason = 'problem_eval_failed';
        return result;
    }
    let currentPhi = getActiveResidualNorm(currentProblem);
    let currentTheta = computeConstraintViolation(currentProblem);
    let currentSafety = getSafetyResidualNorm(currentProblem);
    let currentSecondary = getSecondaryResidualNorm(currentProblem);
    let currentPenalty = Math.max(1, currentPhi + 1);
    let currentMerit = computeRestorationMerit(currentPhi, currentTheta, currentPenalty);
    result.currentProblem = currentProblem;
    result.currentResidualNorm = currentPhi;
    result.bestConstraintViolation = currentTheta;
    result.safetyResidualBefore = currentSafety;
    result.safetyResidualAfter = currentSafety;
    result.secondaryResidualBefore = currentSecondary;
    result.secondaryResidualAfter = currentSecondary;
    if (isSafetyActive(currentProblem, currentPhi, currentTheta)) {
        result.safetyStatus = 'active';
    }
    if (isSecondaryActive(currentProblem, currentPhi, currentTheta)) {
        result.secondaryStatus = 'active';
    }
    if (
        currentPhi <= equalityTolerance &&
        currentTheta <= inequalityTolerance &&
        !isSafetyActive(currentProblem, currentPhi, currentTheta) &&
        !isSecondaryActive(currentProblem, currentPhi, currentTheta)
    ) {
        result.converged = true;
        result.reason = 'converged';
        return result;
    }

    let trustRadius = result.trustRadius;
    let filter = [{ phi: currentPhi, theta: currentTheta }];
    let qAccepted = qStart.slice();
    let acceptedProblem = currentProblem;
    let acceptedPhi = currentPhi;
    let acceptedTheta = currentTheta;
    let hasAccepted = false;
    let previousAcceptedStep = new Array(qStart.length).fill(0);
    const startPhi = currentPhi;
    const startTheta = currentTheta;
    const finishTimeBudget = () => {
        result.reason = hasAccepted ? 'best_effort_time_budget' : 'time_budget_exceeded';
        result.qApplied = hasAccepted ? qAccepted.slice() : qStart.slice();
        result.currentProblem = hasAccepted ? acceptedProblem : currentProblem;
        result.currentResidualNorm = hasAccepted ? acceptedPhi : currentPhi;
        result.bestConstraintViolation = Math.min(result.bestConstraintViolation, hasAccepted ? acceptedTheta : currentTheta);
        result.trustRadius = trustRadius;
        result.filterSize = filter.length;
        result.timeBudgetExceeded = true;
        return result;
    };
    const finishSafetyLimited = () => {
        result.converged = true;
        result.reason = 'safety_limited';
        result.qApplied = qCurrent.slice();
        result.currentProblem = currentProblem;
        result.currentResidualNorm = currentPhi;
        result.bestConstraintViolation = Math.min(result.bestConstraintViolation, currentTheta);
        result.trustRadius = trustRadius;
        result.filterSize = filter.length;
        result.safetyStatus = 'limited';
        result.safetyResidualAfter = currentSafety;
        return result;
    };
    const finishSecondaryLimited = () => {
        result.converged = true;
        result.reason = 'secondary_limited';
        result.qApplied = qCurrent.slice();
        result.currentProblem = currentProblem;
        result.currentResidualNorm = currentPhi;
        result.bestConstraintViolation = Math.min(result.bestConstraintViolation, currentTheta);
        result.trustRadius = trustRadius;
        result.filterSize = filter.length;
        result.secondaryStatus = 'limited';
        result.secondaryResidualAfter = currentSecondary;
        return result;
    };

    for (let iter = 0; iter < maxIters; iter++) {
        result.iterations = iter + 1;
        if (isSolveBudgetExceeded()) {
            return finishTimeBudget();
        }
        let acceptedTrial = null;
        let lastReferenceResidual = [];
        let lastConstraintMotion = [];
        let candidateRadius = trustRadius;
        const safetyActiveThisIteration = isSafetyActive(currentProblem, currentPhi, currentTheta);
        const safetyBiasWeightThisIteration = getSafetyBiasWeight(currentProblem, currentPhi);
        const safetyBiasThisIteration = safetyBiasWeightThisIteration > 0;
        const secondaryActiveThisIteration = isSecondaryActive(currentProblem, currentPhi, currentTheta);
        const secondaryBiasWeightThisIteration = getSecondaryBiasWeight(currentProblem, currentPhi, currentTheta);
        const secondaryBiasThisIteration = secondaryBiasWeightThisIteration > 0;
        if (safetyActiveThisIteration) {
            result.safetyStatus = 'active';
        } else if (safetyBiasThisIteration) {
            result.safetyStatus = 'bias';
        }
        if (secondaryActiveThisIteration) {
            result.secondaryStatus = 'active';
        } else if (secondaryBiasThisIteration) {
            result.secondaryStatus = 'bias';
        }

        for (let dampingAttempt = 0; dampingAttempt <= retryCount && !acceptedTrial; dampingAttempt++) {
            const attemptDamping = damping * (dampingGrowth ** dampingAttempt);
            const { dq, referenceResidual, constraintMotion, modelJacobian, modelReferenceResidual } = buildRestorationStep(
                currentProblem,
                previousAcceptedStep,
                attemptDamping,
                normalizedRegularizationWeights,
                {
                    enabled: safetyActiveThisIteration || safetyBiasThisIteration,
                    weight: safetyActiveThisIteration ? safetyWeight : safetyBiasWeightThisIteration
                },
                {
                    enabled: secondaryActiveThisIteration || secondaryBiasThisIteration,
                    weight: secondaryActiveThisIteration ? secondaryWeight : secondaryBiasWeightThisIteration
                }
            );
            lastReferenceResidual = Array.isArray(referenceResidual) ? referenceResidual.slice() : [];
            lastConstraintMotion = Array.isArray(constraintMotion) ? constraintMotion.slice() : [];
            if (!Array.isArray(dq) || dq.length !== qCurrent.length || vectorNorm(dq) <= 1e-18) continue;

            for (let radiusAttempt = 0; radiusAttempt < maxRadiusRetries; radiusAttempt++) {
                if (isSolveBudgetExceeded()) {
                    return finishTimeBudget();
                }
                const clippedStep = clipStepToTrustRegion(dq, candidateRadius);
                if (clippedStep.stepNorm <= 1e-18) break;
                const qTrial = projectStateToBounds(addScaledStep(qCurrent, clippedStep.dq, 1), currentProblem?.bounds);
                const trialProblem = evaluateTrialProblem(qTrial);
                if (!trialProblem || typeof trialProblem !== 'object') {
                    candidateRadius = Math.max(minRadius, candidateRadius * shrinkFactor);
                    continue;
                }
                const trialPhi = getActiveResidualNorm(trialProblem);
                const trialTheta = computeConstraintViolation(trialProblem);
                const trialSafety = getSafetyResidualNorm(trialProblem);
                const trialSecondary = getSecondaryResidualNorm(trialProblem);
                const trialPenalty = Math.max(currentPenalty, trialPhi + 1);
                const trialMerit = computeRestorationMerit(trialPhi, trialTheta, trialPenalty);
                const predictedReduction = computeModelObjective(
                    currentProblem,
                    new Array(qCurrent.length).fill(0),
                    attemptDamping,
                    normalizedRegularizationWeights,
                    modelReferenceResidual,
                    modelJacobian
                ) - computeModelObjective(
                    currentProblem,
                    clippedStep.dq,
                    attemptDamping,
                    normalizedRegularizationWeights,
                    modelReferenceResidual,
                    modelJacobian
                );
                const actualReduction = currentMerit - trialMerit;
                const rho = predictedReduction > 1e-12 ? actualReduction / predictedReduction : Number.POSITIVE_INFINITY;
                let acceptance = acceptRestorationCandidateByFilter({
                    filter,
                    currentPhi,
                    currentTheta,
                    candidatePhi: trialPhi,
                    candidateTheta: trialTheta,
                    predictedReduction,
                    actualReduction,
                    rho,
                    rhoAcceptanceThreshold,
                    phiTolerance,
                    thetaTolerance
                });
                const safetyImproved = trialSafety + safetyResidualTolerance < currentSafety;
                const secondaryImproved = hasTrialSecondaryStatus(trialProblem) &&
                    trialSecondary + secondaryResidualTolerance < currentSecondary;
                const safetyAccepted = (
                    safetyActiveThisIteration &&
                    isPrimaryHeldForSafety(currentPhi, currentTheta) &&
                    isPrimaryHeldForSafety(trialPhi, trialTheta) &&
                    safetyImproved
                );
                const secondaryAccepted = (
                    secondaryActiveThisIteration &&
                    isPrimaryHeldForSecondary(currentPhi, currentTheta) &&
                    isPrimaryHeldForSecondary(trialPhi, trialTheta) &&
                    secondaryImproved
                );
                if (!acceptance.accepted && safetyAccepted) {
                    acceptance = {
                        accepted: true,
                        reason: 'safety',
                        nextFilter: filter,
                        phiImproved: false,
                        thetaImproved: false,
                        predictedReduction: Math.max(0, predictedReduction),
                        actualReduction
                    };
                }
                if (!acceptance.accepted && secondaryAccepted) {
                    acceptance = {
                        accepted: true,
                        reason: 'secondary',
                        nextFilter: filter,
                        phiImproved: false,
                        thetaImproved: false,
                        predictedReduction: Math.max(0, predictedReduction),
                        actualReduction
                    };
                }
                const nextRadius = updateTrustRegionRadius(candidateRadius, rho, clippedStep.stepNorm, clippedStep.clipped, {
                    minTrustRadius: minRadius,
                    maxTrustRadius: maxRadius,
                    shrinkFactor,
                    expandFactor,
                    rhoShrinkThreshold,
                    rhoExpandThreshold
                });
                if (acceptance.accepted) {
                    acceptedTrial = {
                        qTrial,
                        trialProblem,
                        trialPhi,
                        trialTheta,
                        trialMerit,
                        dqTrial: clippedStep.dq.slice(),
                        damping: attemptDamping,
                        rho,
                        predictedReduction,
                        actualReduction,
                        acceptedBy: acceptance.reason,
                        trialSafety,
                        trialSecondary,
                        nextFilter: acceptance.nextFilter,
                        nextRadius: (acceptance.reason === 'safety' || acceptance.reason === 'secondary') ? candidateRadius : nextRadius
                    };
                    break;
                }
                result.lastRejectedRho = rho;
                result.lastRejectedPredictedReduction = predictedReduction;
                result.lastRejectedActualReduction = actualReduction;
                result.lastRejectedReason = acceptance.reason;
                candidateRadius = nextRadius;
                if (candidateRadius <= minRadius + 1e-12) break;
            }
        }

        result.lastReferenceResidual = lastReferenceResidual;
        result.lastConstraintMotion = lastConstraintMotion;
        if (!acceptedTrial) {
            if (safetyActiveThisIteration && isPrimaryHeldForSafety(currentPhi, currentTheta)) {
                return finishSafetyLimited();
            }
            if (secondaryActiveThisIteration && isPrimaryHeldForSecondary(currentPhi, currentTheta)) {
                return finishSecondaryLimited();
            }
            result.reason = hasAccepted ? 'best_effort' : 'line_search_failed';
            result.qApplied = hasAccepted ? qAccepted.slice() : qStart.slice();
            result.currentProblem = hasAccepted ? acceptedProblem : currentProblem;
            result.currentResidualNorm = hasAccepted ? acceptedPhi : currentPhi;
            result.bestConstraintViolation = Math.min(result.bestConstraintViolation, hasAccepted ? acceptedTheta : currentTheta);
            result.trustRadius = trustRadius;
            result.filterSize = filter.length;
            return result;
        }

        qCurrent = acceptedTrial.qTrial.slice();
        currentProblem = evaluateCurrentProblem(qCurrent);
        if (!currentProblem || typeof currentProblem !== 'object') {
            currentProblem = acceptedTrial.trialProblem;
        }
        currentPhi = getActiveResidualNorm(currentProblem);
        currentTheta = computeConstraintViolation(currentProblem);
        currentSafety = getSafetyResidualNorm(currentProblem);
        currentSecondary = getSecondaryResidualNorm(currentProblem);
        currentPenalty = Math.max(currentPenalty, currentPhi + 1);
        currentMerit = computeRestorationMerit(currentPhi, currentTheta, currentPenalty);
        qAccepted = qCurrent.slice();
        acceptedProblem = currentProblem;
        acceptedPhi = currentPhi;
        acceptedTheta = currentTheta;
        hasAccepted = true;
        previousAcceptedStep = acceptedTrial.dqTrial.slice();
        trustRadius = acceptedTrial.nextRadius;
        filter = acceptedTrial.nextFilter;

        result.acceptedIterations += 1;
        result.lastAcceptedScale = 1;
        result.lastAcceptedDamping = acceptedTrial.damping;
        result.currentProblem = currentProblem;
        result.currentResidualNorm = currentPhi;
        result.trustRadius = trustRadius;
        result.lastAcceptedRho = acceptedTrial.rho;
        result.lastPredictedReduction = acceptedTrial.predictedReduction;
        result.lastActualReduction = acceptedTrial.actualReduction;
        result.acceptedBy = acceptedTrial.acceptedBy;
        if (acceptedTrial.acceptedBy === 'safety') {
            result.safetyAcceptedIterations += 1;
        }
        if (acceptedTrial.acceptedBy === 'secondary') {
            result.secondaryAcceptedIterations += 1;
        }
        result.safetyResidualAfter = currentSafety;
        result.secondaryResidualAfter = currentSecondary;
        result.bestConstraintViolation = Math.min(result.bestConstraintViolation, currentTheta);
        result.filterSize = filter.length;

        if (
            currentPhi <= equalityTolerance &&
            currentTheta <= inequalityTolerance &&
            !isSafetyActive(currentProblem, currentPhi, currentTheta) &&
            !isSecondaryActive(currentProblem, currentPhi, currentTheta)
        ) {
            result.converged = true;
            result.reason = 'converged';
            result.qApplied = qAccepted.slice();
            return result;
        }
    }

    if (isPrimaryHeldForSafety(currentPhi, currentTheta) && isSafetyActive(currentProblem, currentPhi, currentTheta)) {
        return finishSafetyLimited();
    }
    if (isPrimaryHeldForSecondary(currentPhi, currentTheta) && isSecondaryActive(currentProblem, currentPhi, currentTheta)) {
        return finishSecondaryLimited();
    }
    result.reason = (
        hasAccepted &&
        (acceptedPhi + phiTolerance < startPhi || acceptedTheta + thetaTolerance < startTheta)
    ) ? 'best_effort' : 'max_iter';
    result.qApplied = hasAccepted ? qAccepted.slice() : qStart.slice();
    result.currentProblem = hasAccepted ? acceptedProblem : currentProblem;
    result.currentResidualNorm = hasAccepted ? acceptedPhi : currentPhi;
    result.bestConstraintViolation = Math.min(result.bestConstraintViolation, hasAccepted ? acceptedTheta : currentTheta);
    result.trustRadius = trustRadius;
    result.filterSize = filter.length;
    return result;
}
