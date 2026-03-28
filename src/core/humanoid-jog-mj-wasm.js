export const HUMANOID_MJ_RESTORE_WASM_DEFAULTS = Object.freeze({
    keepBodyZVertical: false,
    bodyEqualityTolerance: 0,
    absoluteWeight: 1,
    relativeWeight: 1,
    bodyWeight: 1,
    wvfiLowerBound: 0,
    jointLimitActivationThreshold: 0.05,
    referenceModel: {
        absolute: { stiffness: 1.0, damping: 0.25 },
        relative: { stiffness: 1.0, damping: 0.35 },
        bodyZ: { stiffness: 0.6, damping: 0.45 }
    },
    maxIterations: 16,
    damping: 1e-6,
    initialScale: 1,
    minScale: 1e-6,
    reductionFactor: 0.5,
    equalityTolerance: 1e-12,
    inequalityTolerance: 1e-12
});

function nowMs() {
    if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
        return performance.now();
    }
    return Date.now();
}

function cloneRows(rows) {
    return Array.isArray(rows)
        ? rows.map((row) => Array.isArray(row) ? row.slice() : [])
        : [];
}

function cloneCorridorRows(waistXCorridor) {
    return cloneRows(waistXCorridor?.rows ?? waistXCorridor?.vfiRows);
}

function cloneArray(values) {
    return Array.isArray(values) ? values.slice() : [];
}

function finiteOr(value, fallback) {
    return Number.isFinite(value) ? value : fallback;
}

function buildReferenceAxisModel(axis = {}, fallback = {}) {
    return {
        stiffness: Math.max(0, finiteOr(axis?.stiffness, fallback?.stiffness ?? 0)),
        damping: Math.max(0, finiteOr(axis?.damping, fallback?.damping ?? 0))
    };
}

export function canUseHumanoidJogMJRestoreWasm(humanoidModel) {
    return typeof humanoidModel?.solveAbsoluteTargetMJRestore === 'function';
}

export function buildHumanoidJogMJRestoreWasmRequest({
    qCurrent,
    activeStart,
    activeCount,
    absoluteTarget,
    relativeTarget,
    keepBodyZVertical,
    bodyEqualityTolerance,
    absoluteWeight,
    relativeWeight,
    bodyWeight,
    lowerBounds,
    upperBounds,
    stepBounds,
    wvfiLowerBound,
    jointLimitActivationThreshold,
    waistXCorridor,
    referenceModel,
    maxIterations,
    damping,
    initialScale,
    minScale,
    reductionFactor,
    equalityTolerance,
    inequalityTolerance
} = {}) {
    const fallback = HUMANOID_MJ_RESTORE_WASM_DEFAULTS;
    return {
        qCurrent: cloneArray(qCurrent),
        activeStart: Number.isFinite(activeStart) ? Math.max(0, Math.floor(activeStart)) : 0,
        activeCount: Number.isFinite(activeCount) ? Math.max(0, Math.floor(activeCount)) : 0,
        absoluteTarget: cloneArray(absoluteTarget),
        relativeTarget: cloneArray(relativeTarget),
        keepBodyZVertical: keepBodyZVertical === true,
        bodyEqualityTolerance: Math.max(0, finiteOr(bodyEqualityTolerance, fallback.bodyEqualityTolerance)),
        absoluteWeight: Math.max(0, finiteOr(absoluteWeight, fallback.absoluteWeight)),
        relativeWeight: Math.max(0, finiteOr(relativeWeight, fallback.relativeWeight)),
        bodyWeight: Math.max(0, finiteOr(bodyWeight, fallback.bodyWeight)),
        lowerBounds: cloneArray(lowerBounds),
        upperBounds: cloneArray(upperBounds),
        stepBounds: cloneArray(stepBounds),
        wvfiLowerBound: Math.max(0, finiteOr(wvfiLowerBound, fallback.wvfiLowerBound)),
        jointLimitActivationThreshold: Math.max(
            0,
            finiteOr(jointLimitActivationThreshold, fallback.jointLimitActivationThreshold)
        ),
        waistXCorridor: {
            enabled: waistXCorridor?.enabled === true,
            min: finiteOr(waistXCorridor?.min, Number.NEGATIVE_INFINITY),
            max: finiteOr(waistXCorridor?.max, Number.POSITIVE_INFINITY),
            tolerance: Math.max(0, finiteOr(waistXCorridor?.tolerance, 0)),
            rows: cloneCorridorRows(waistXCorridor),
            rhs: cloneArray(waistXCorridor?.rhs)
        },
        referenceModel: {
            absolute: buildReferenceAxisModel(
                referenceModel?.absolute,
                fallback.referenceModel.absolute
            ),
            relative: buildReferenceAxisModel(
                referenceModel?.relative,
                fallback.referenceModel.relative
            ),
            bodyZ: buildReferenceAxisModel(
                referenceModel?.bodyZ,
                fallback.referenceModel.bodyZ
            )
        },
        maxIterations: Math.max(1, Math.floor(finiteOr(maxIterations, fallback.maxIterations))),
        damping: Math.max(1e-12, finiteOr(damping, fallback.damping)),
        initialScale: Math.max(1e-8, finiteOr(initialScale, fallback.initialScale)),
        minScale: Math.max(1e-8, finiteOr(minScale, fallback.minScale)),
        reductionFactor: Math.min(0.99, Math.max(0.01, finiteOr(reductionFactor, fallback.reductionFactor))),
        equalityTolerance: Math.max(0, finiteOr(equalityTolerance, fallback.equalityTolerance)),
        inequalityTolerance: Math.max(0, finiteOr(inequalityTolerance, fallback.inequalityTolerance))
    };
}

export function normalizeHumanoidJogMJRestoreWasmResult(resultRaw, fallbackQApplied = []) {
    return {
        converged: resultRaw?.converged === true,
        reason: typeof resultRaw?.reason === 'string' && resultRaw.reason.trim()
            ? resultRaw.reason.trim()
            : 'unknown',
        iterations: Number.isFinite(resultRaw?.iterations) ? resultRaw.iterations : 0,
        acceptedIterations: Number.isFinite(resultRaw?.acceptedIterations)
            ? resultRaw.acceptedIterations
            : 0,
        lastAcceptedScale: Number.isFinite(resultRaw?.lastAcceptedScale)
            ? resultRaw.lastAcceptedScale
            : 0,
        commandResidualNorm: Number.isFinite(resultRaw?.commandResidualNorm)
            ? resultRaw.commandResidualNorm
            : Number.POSITIVE_INFINITY,
        relativeResidualNorm: Number.isFinite(resultRaw?.relativeResidualNorm)
            ? resultRaw.relativeResidualNorm
            : Number.POSITIVE_INFINITY,
        bodyResidualNorm: Number.isFinite(resultRaw?.bodyResidualNorm)
            ? resultRaw.bodyResidualNorm
            : 0,
        inequalityViolation: Number.isFinite(resultRaw?.inequalityViolation)
            ? resultRaw.inequalityViolation
            : Number.POSITIVE_INFINITY,
        workerInitMs: Number.isFinite(resultRaw?.workerInitMs)
            ? resultRaw.workerInitMs
            : Number.NaN,
        solveRoundTripMs: Number.isFinite(resultRaw?.solveRoundTripMs)
            ? resultRaw.solveRoundTripMs
            : Number.NaN,
        solveMs: Number.isFinite(resultRaw?.solveMs)
            ? resultRaw.solveMs
            : Number.NaN,
        secondaryMetric: typeof resultRaw?.secondaryMetric === 'string' && resultRaw.secondaryMetric.trim()
            ? resultRaw.secondaryMetric.trim()
            : '',
        secondaryState: typeof resultRaw?.secondaryState === 'string' && resultRaw.secondaryState.trim()
            ? resultRaw.secondaryState.trim()
            : '',
        secondaryBefore: Number.isFinite(resultRaw?.secondaryBefore)
            ? resultRaw.secondaryBefore
            : Number.NaN,
        secondaryAfter: Number.isFinite(resultRaw?.secondaryAfter)
            ? resultRaw.secondaryAfter
            : Number.NaN,
        secondaryApplied: resultRaw?.secondaryApplied === true,
        referenceResidual: cloneArray(resultRaw?.referenceResidual),
        constraintMotion: cloneArray(resultRaw?.constraintMotion),
        qApplied: Array.isArray(resultRaw?.qApplied)
            ? resultRaw.qApplied.map((value, index) => Number.isFinite(value) ? value : (fallbackQApplied[index] ?? 0))
            : cloneArray(fallbackQApplied)
    };
}

export function attemptHumanoidJogMJRestoreWasm({
    humanoidModel,
    qInitial,
    evaluateProblem,
    activeStart,
    activeCount,
    absoluteTarget,
    relativeTarget,
    keepBodyZVertical,
    bodyEqualityTolerance,
    absoluteWeight,
    relativeWeight,
    bodyWeight,
    lowerBounds,
    upperBounds,
    stepBounds,
    wvfiLowerBound,
    jointLimitActivationThreshold,
    referenceModel,
    maxIterations,
    damping,
    initialScale,
    minScale,
    reductionFactor,
    equalityTolerance,
    inequalityTolerance
} = {}) {
    if (!canUseHumanoidJogMJRestoreWasm(humanoidModel) || typeof evaluateProblem !== 'function') {
        return null;
    }

    const preStartMs = nowMs();
    const qInitialActive = cloneArray(qInitial);
    const initialProblem = evaluateProblem(qInitialActive);
    if (!initialProblem || typeof initialProblem !== 'object') {
        return null;
    }

    const request = buildHumanoidJogMJRestoreWasmRequest({
        qCurrent: cloneArray(initialProblem?.metadata?.qFull ?? initialProblem?.qCurrent ?? []),
        activeStart,
        activeCount,
        absoluteTarget,
        relativeTarget,
        keepBodyZVertical,
        bodyEqualityTolerance,
        absoluteWeight,
        relativeWeight,
        bodyWeight,
        lowerBounds,
        upperBounds,
        stepBounds,
        wvfiLowerBound,
        jointLimitActivationThreshold,
        waistXCorridor: initialProblem?.inequalities?.waistXCorridor ?? null,
        referenceModel,
        maxIterations,
        damping,
        initialScale,
        minScale,
        reductionFactor,
        equalityTolerance,
        inequalityTolerance
    });
    const preMs = nowMs() - preStartMs;

    const solveStartMs = nowMs();
    const rawResult = humanoidModel.solveAbsoluteTargetMJRestore(request);
    const measuredSolveMs = nowMs() - solveStartMs;
    const normalized = normalizeHumanoidJogMJRestoreWasmResult(rawResult, qInitialActive);
    const postStartMs = nowMs();
    const currentProblem = evaluateProblem(normalized.qApplied);
    const postMs = nowMs() - postStartMs;
    return {
        ...normalized,
        currentProblem: currentProblem && typeof currentProblem === 'object' ? currentProblem : initialProblem,
        currentResidualNorm: Number.isFinite(currentProblem?.equalities?.activeResidualNorm)
            ? currentProblem.equalities.activeResidualNorm
            : Number.POSITIVE_INFINITY,
        lastReferenceResidual: cloneArray(normalized.referenceResidual),
        lastConstraintMotion: cloneArray(normalized.constraintMotion),
        timings: {
            preMs,
            workerInitMs: Number.isFinite(normalized.workerInitMs) ? normalized.workerInitMs : Number.NaN,
            solveWaitMs: Number.isFinite(normalized.solveRoundTripMs) && Number.isFinite(normalized.solveMs)
                ? Math.max(0, normalized.solveRoundTripMs - normalized.solveMs)
                : Number.NaN,
            solveMs: Number.isFinite(normalized.solveMs) ? normalized.solveMs : measuredSolveMs,
            postMs,
            totalMs: preMs + (Number.isFinite(normalized.solveMs) ? normalized.solveMs : measuredSolveMs) + postMs
        }
    };
}

export function buildHumanoidJogMJRestoreWasmEnvelope({
    attempt = null,
    qInitial = [],
    evaluateProblem = null
} = {}) {
    if (!attempt || typeof attempt !== 'object') return null;
    const qApplied = Array.isArray(attempt.qApplied) ? attempt.qApplied.slice() : cloneArray(qInitial);
    const currentProblem = attempt.currentProblem || (typeof evaluateProblem === 'function' ? evaluateProblem(qApplied) : null);
    return {
        converged: attempt.converged === true,
        reason: typeof attempt.reason === 'string' && attempt.reason.trim()
            ? attempt.reason.trim()
            : 'unknown',
        qApplied,
        iterations: Number.isFinite(attempt.iterations) ? attempt.iterations : 0,
        acceptedIterations: Number.isFinite(attempt.acceptedIterations)
            ? attempt.acceptedIterations
            : 0,
        lastAcceptedScale: Number.isFinite(attempt.lastAcceptedScale)
            ? attempt.lastAcceptedScale
            : 0,
        currentProblem,
        currentResidualNorm: Number.isFinite(attempt.currentResidualNorm)
            ? attempt.currentResidualNorm
            : (Number.isFinite(currentProblem?.equalities?.activeResidualNorm)
                ? currentProblem.equalities.activeResidualNorm
                : Number.POSITIVE_INFINITY),
        lastReferenceResidual: Array.isArray(attempt.lastReferenceResidual)
            ? attempt.lastReferenceResidual.slice()
            : [],
        lastConstraintMotion: Array.isArray(attempt.lastConstraintMotion)
            ? attempt.lastConstraintMotion.slice()
            : [],
        secondaryMetricAttempt: attempt.secondaryMetricAttempt && typeof attempt.secondaryMetricAttempt === 'object'
            ? { ...attempt.secondaryMetricAttempt }
            : null,
        timings: attempt.timings && typeof attempt.timings === 'object'
            ? { ...attempt.timings }
            : null
    };
}

export async function attemptHumanoidJogMJRestoreWasmWorker({
    workerClient,
    modelSpec,
    qInitial,
    evaluateProblem,
    activeStart,
    activeCount,
    absoluteTarget,
    relativeTarget,
    keepBodyZVertical,
    bodyEqualityTolerance,
    absoluteWeight,
    relativeWeight,
    bodyWeight,
    lowerBounds,
    upperBounds,
    stepBounds,
    wvfiLowerBound,
    jointLimitActivationThreshold,
    referenceModel,
    maxIterations,
    damping,
    initialScale,
    minScale,
    reductionFactor,
    equalityTolerance,
    inequalityTolerance,
    secondaryMetricRequest = null
} = {}) {
    if (!workerClient || !modelSpec || typeof workerClient.solve !== 'function' || typeof evaluateProblem !== 'function') {
        return null;
    }

    const preStartMs = nowMs();
    const qInitialActive = cloneArray(qInitial);
    const initialProblem = evaluateProblem(qInitialActive);
    if (!initialProblem || typeof initialProblem !== 'object') {
        return null;
    }

    const request = buildHumanoidJogMJRestoreWasmRequest({
        qCurrent: cloneArray(initialProblem?.metadata?.qFull ?? initialProblem?.qCurrent ?? []),
        activeStart,
        activeCount,
        absoluteTarget,
        relativeTarget,
        keepBodyZVertical,
        bodyEqualityTolerance,
        absoluteWeight,
        relativeWeight,
        bodyWeight,
        lowerBounds,
        upperBounds,
        stepBounds,
        wvfiLowerBound,
        jointLimitActivationThreshold,
        waistXCorridor: initialProblem?.inequalities?.waistXCorridor ?? null,
        referenceModel,
        maxIterations,
        damping,
        initialScale,
        minScale,
        reductionFactor,
        equalityTolerance,
        inequalityTolerance
    });
    const preMs = nowMs() - preStartMs;

    const solveStartMs = nowMs();
    const rawResult = await workerClient.solve({
        modelSpec,
        request,
        secondaryMetricRequest
    });
    const measuredSolveMs = nowMs() - solveStartMs;
    if (!rawResult || typeof rawResult !== 'object') {
        return null;
    }

    const normalized = normalizeHumanoidJogMJRestoreWasmResult(rawResult, qInitialActive);
    const postStartMs = nowMs();
    const currentProblem = evaluateProblem(normalized.qApplied);
    const postMs = nowMs() - postStartMs;
    return {
        ...normalized,
        currentProblem: currentProblem && typeof currentProblem === 'object' ? currentProblem : initialProblem,
        currentResidualNorm: Number.isFinite(currentProblem?.equalities?.activeResidualNorm)
            ? currentProblem.equalities.activeResidualNorm
            : Number.POSITIVE_INFINITY,
        lastReferenceResidual: cloneArray(normalized.referenceResidual),
        lastConstraintMotion: cloneArray(normalized.constraintMotion),
        secondaryMetricAttempt: normalized.secondaryMetric
            ? {
                metric: normalized.secondaryMetric,
                state: normalized.secondaryState || 'unknown',
                before: normalized.secondaryBefore,
                after: normalized.secondaryAfter,
                applied: normalized.secondaryApplied === true
            }
            : null,
        timings: {
            preMs,
            workerInitMs: Number.isFinite(normalized.workerInitMs) ? normalized.workerInitMs : Number.NaN,
            solveWaitMs: Number.isFinite(normalized.solveRoundTripMs) && Number.isFinite(normalized.solveMs)
                ? Math.max(0, normalized.solveRoundTripMs - normalized.solveMs)
                : Number.NaN,
            solveMs: Number.isFinite(normalized.solveMs) ? normalized.solveMs : measuredSolveMs,
            postMs,
            totalMs: preMs + (Number.isFinite(normalized.solveMs) ? normalized.solveMs : measuredSolveMs) + postMs
        }
    };
}
