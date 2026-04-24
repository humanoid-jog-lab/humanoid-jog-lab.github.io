export const HUMANOID_MJ_RESTORE_WASM_DEFAULTS = Object.freeze({
    solveVariant: 'absolute_target',
    solverMode: 'mj_restore',
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
    maxDampingRetries: 8,
    dampingGrowthFactor: 10,
    initialScale: 1,
    minScale: 1e-6,
    reductionFactor: 0.5,
    initialTrustRadius: 0.2,
    minTrustRadius: 1e-6,
    maxTrustRadius: 1.0,
    maxTrustRegionRetries: 6,
    trustRegionShrinkFactor: 0.5,
    trustRegionExpandFactor: 1.75,
    rhoAcceptanceThreshold: 0.05,
    rhoShrinkThreshold: 0.05,
    rhoExpandThreshold: 0.75,
    equalityTolerance: 1e-12,
    inequalityTolerance: 1e-12,
    safetyPush: {
        enabled: true,
        weight: 0.05,
        activationResidual: 1e-5,
        poseHoldTolerance: 2e-6,
        targetFraction: 0.25,
        maxDistanceStep: 0.01
    }
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

function cloneJointLimitVFIRows(jointLimitVFI) {
    return cloneRows(jointLimitVFI?.rows);
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

function normalizeSolverMode(solverMode) {
    return solverMode === 'mj_restore_tr' ? 'mj_restore_tr' : 'mj_restore';
}

function normalizeSolveVariant(solveVariant) {
    if (solveVariant === 'cooperative_dq_setpoint') return 'cooperative_dq_setpoint';
    if (solveVariant === 'absolute_dq_setpoint') return 'absolute_dq_setpoint';
    return 'absolute_target';
}

function getHumanoidJogMJRestoreWasmSolveFunctionName(solverMode, solveVariant = 'absolute_target') {
    const normalizedSolverMode = normalizeSolverMode(solverMode);
    const normalizedSolveVariant = normalizeSolveVariant(solveVariant);
    if (normalizedSolveVariant === 'cooperative_dq_setpoint') {
        return normalizedSolverMode === 'mj_restore_tr'
            ? 'solveCooperativeDQSetpointMJRestoreTrustRegion'
            : 'solveCooperativeDQSetpointMJRestore';
    }
    if (normalizedSolveVariant === 'absolute_dq_setpoint') {
        return normalizedSolverMode === 'mj_restore_tr'
            ? 'solveAbsoluteDQSetpointMJRestoreTrustRegion'
            : 'solveAbsoluteDQSetpointMJRestore';
    }
    return normalizedSolverMode === 'mj_restore_tr'
        ? 'solveAbsoluteTargetMJRestoreTrustRegion'
        : 'solveAbsoluteTargetMJRestore';
}

export function canUseHumanoidJogMJRestoreWasm(
    humanoidModel,
    solverMode = 'mj_restore',
    solveVariant = 'absolute_target'
) {
    return typeof humanoidModel?.[getHumanoidJogMJRestoreWasmSolveFunctionName(solverMode, solveVariant)] === 'function';
}

export function buildHumanoidJogMJRestoreWasmRequest({
    solveVariant,
    solverMode,
    qCurrent,
    activeStart,
    activeCount,
    absoluteTarget,
    relativeTarget,
    leftTarget,
    rightTarget,
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
    jointLimitVFI,
    waistXCorridor,
    referenceModel,
    maxIterations,
    damping,
    maxDampingRetries,
    dampingGrowthFactor,
    initialScale,
    minScale,
    reductionFactor,
    initialTrustRadius,
    minTrustRadius,
    maxTrustRadius,
    maxTrustRegionRetries,
    trustRegionShrinkFactor,
    trustRegionExpandFactor,
    rhoAcceptanceThreshold,
    rhoShrinkThreshold,
    rhoExpandThreshold,
    regularizationWeights,
    equalityTolerance,
    inequalityTolerance,
    safetyPushEnabled,
    safetyPushWeight,
    safetyPushActivationResidual,
    safetyPushPoseHoldTolerance,
    safetyPushTargetFraction,
    safetyPushMaxDistanceStep
} = {}) {
    const fallback = HUMANOID_MJ_RESTORE_WASM_DEFAULTS;
    return {
        solveVariant: normalizeSolveVariant(solveVariant || fallback.solveVariant),
        solverMode: normalizeSolverMode(solverMode || fallback.solverMode),
        qCurrent: cloneArray(qCurrent),
        activeStart: Number.isFinite(activeStart) ? Math.max(0, Math.floor(activeStart)) : 0,
        activeCount: Number.isFinite(activeCount) ? Math.max(0, Math.floor(activeCount)) : 0,
        absoluteTarget: cloneArray(absoluteTarget),
        relativeTarget: cloneArray(relativeTarget),
        leftTarget: cloneArray(leftTarget),
        rightTarget: cloneArray(rightTarget),
        keepBodyZVertical: keepBodyZVertical === true,
        bodyEqualityTolerance: Math.max(0, finiteOr(bodyEqualityTolerance, fallback.bodyEqualityTolerance)),
        absoluteWeight: Math.max(0, finiteOr(absoluteWeight, fallback.absoluteWeight)),
        relativeWeight: Math.max(0, finiteOr(relativeWeight, fallback.relativeWeight)),
        bodyWeight: Math.max(0, finiteOr(bodyWeight, fallback.bodyWeight)),
        lowerBounds: cloneArray(lowerBounds),
        upperBounds: cloneArray(upperBounds),
        stepBounds: cloneArray(stepBounds),
        regularizationWeights: cloneArray(regularizationWeights),
        wvfiLowerBound: Math.max(0, finiteOr(wvfiLowerBound, fallback.wvfiLowerBound)),
        jointLimitActivationThreshold: Math.max(
            0,
            finiteOr(jointLimitActivationThreshold, fallback.jointLimitActivationThreshold)
        ),
        jointLimitVFI: {
            enabled: jointLimitVFI?.enabled === true,
            rows: cloneJointLimitVFIRows(jointLimitVFI),
            rhs: cloneArray(jointLimitVFI?.rhs),
            count: Math.max(
                0,
                Math.floor(
                    finiteOr(
                        jointLimitVFI?.count,
                        Array.isArray(jointLimitVFI?.rows) ? jointLimitVFI.rows.length : 0
                    )
                )
            )
        },
        waistXCorridor: {
            enabled: waistXCorridor?.enabled === true,
            min: finiteOr(waistXCorridor?.min, Number.NEGATIVE_INFINITY),
            max: finiteOr(waistXCorridor?.max, Number.POSITIVE_INFINITY),
            tolerance: Math.max(0, finiteOr(waistXCorridor?.tolerance, 0)),
            activationThreshold: Math.max(
                0,
                finiteOr(
                    waistXCorridor?.activationThreshold,
                    finiteOr(waistXCorridor?.vfiConstraints?.[0]?.threshold, 0.12)
                )
            ),
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
        maxDampingRetries: Math.max(0, Math.floor(finiteOr(maxDampingRetries, fallback.maxDampingRetries))),
        dampingGrowthFactor: Math.max(1.01, finiteOr(dampingGrowthFactor, fallback.dampingGrowthFactor)),
        initialScale: Math.max(1e-8, finiteOr(initialScale, fallback.initialScale)),
        minScale: Math.max(1e-8, finiteOr(minScale, fallback.minScale)),
        reductionFactor: Math.min(0.99, Math.max(0.01, finiteOr(reductionFactor, fallback.reductionFactor))),
        initialTrustRadius: Math.max(1e-8, finiteOr(initialTrustRadius, fallback.initialTrustRadius)),
        minTrustRadius: Math.max(1e-8, finiteOr(minTrustRadius, fallback.minTrustRadius)),
        maxTrustRadius: Math.max(1e-8, finiteOr(maxTrustRadius, fallback.maxTrustRadius)),
        maxTrustRegionRetries: Math.max(1, Math.floor(finiteOr(maxTrustRegionRetries, fallback.maxTrustRegionRetries))),
        trustRegionShrinkFactor: Math.min(
            0.99,
            Math.max(0.01, finiteOr(trustRegionShrinkFactor, fallback.trustRegionShrinkFactor))
        ),
        trustRegionExpandFactor: Math.max(1.01, finiteOr(trustRegionExpandFactor, fallback.trustRegionExpandFactor)),
        rhoAcceptanceThreshold: finiteOr(rhoAcceptanceThreshold, fallback.rhoAcceptanceThreshold),
        rhoShrinkThreshold: finiteOr(rhoShrinkThreshold, fallback.rhoShrinkThreshold),
        rhoExpandThreshold: finiteOr(rhoExpandThreshold, fallback.rhoExpandThreshold),
        equalityTolerance: Math.max(0, finiteOr(equalityTolerance, fallback.equalityTolerance)),
        inequalityTolerance: Math.max(0, finiteOr(inequalityTolerance, fallback.inequalityTolerance)),
        safetyPush: {
            enabled: safetyPushEnabled == null
                ? fallback.safetyPush.enabled
                : safetyPushEnabled === true,
            weight: Math.max(0, finiteOr(safetyPushWeight, fallback.safetyPush.weight)),
            activationResidual: Math.max(
                0,
                finiteOr(safetyPushActivationResidual, fallback.safetyPush.activationResidual)
            ),
            poseHoldTolerance: Math.max(
                0,
                finiteOr(safetyPushPoseHoldTolerance, fallback.safetyPush.poseHoldTolerance)
            ),
            targetFraction: Math.max(0, finiteOr(safetyPushTargetFraction, fallback.safetyPush.targetFraction)),
            maxDistanceStep: Math.max(0, finiteOr(safetyPushMaxDistanceStep, fallback.safetyPush.maxDistanceStep))
        }
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
        lastAcceptedDamping: Number.isFinite(resultRaw?.lastAcceptedDamping)
            ? resultRaw.lastAcceptedDamping
            : Number.NaN,
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
        trustRadius: Number.isFinite(resultRaw?.trustRadius)
            ? resultRaw.trustRadius
            : Number.NaN,
        lastAcceptedRho: Number.isFinite(resultRaw?.lastAcceptedRho)
            ? resultRaw.lastAcceptedRho
            : Number.NaN,
        lastPredictedReduction: Number.isFinite(resultRaw?.lastPredictedReduction)
            ? resultRaw.lastPredictedReduction
            : Number.NaN,
        lastActualReduction: Number.isFinite(resultRaw?.lastActualReduction)
            ? resultRaw.lastActualReduction
            : Number.NaN,
        acceptedBy: typeof resultRaw?.acceptedBy === 'string' && resultRaw.acceptedBy.trim()
            ? resultRaw.acceptedBy.trim()
            : '',
        lastRejectedRho: Number.isFinite(resultRaw?.lastRejectedRho)
            ? resultRaw.lastRejectedRho
            : Number.NaN,
        lastRejectedPredictedReduction: Number.isFinite(resultRaw?.lastRejectedPredictedReduction)
            ? resultRaw.lastRejectedPredictedReduction
            : Number.NaN,
        lastRejectedActualReduction: Number.isFinite(resultRaw?.lastRejectedActualReduction)
            ? resultRaw.lastRejectedActualReduction
            : Number.NaN,
        lastRejectedReason: typeof resultRaw?.lastRejectedReason === 'string' && resultRaw.lastRejectedReason.trim()
            ? resultRaw.lastRejectedReason.trim()
            : '',
        bestConstraintViolation: Number.isFinite(resultRaw?.bestConstraintViolation)
            ? resultRaw.bestConstraintViolation
            : Number.POSITIVE_INFINITY,
        filterSize: Number.isFinite(resultRaw?.filterSize)
            ? resultRaw.filterSize
            : 0,
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
        safetyStatus: typeof resultRaw?.safetyStatus === 'string' && resultRaw.safetyStatus.trim()
            ? resultRaw.safetyStatus.trim()
            : 'skipped',
        safetyResidualBefore: Number.isFinite(resultRaw?.safetyResidualBefore)
            ? resultRaw.safetyResidualBefore
            : Number.NaN,
        safetyResidualAfter: Number.isFinite(resultRaw?.safetyResidualAfter)
            ? resultRaw.safetyResidualAfter
            : Number.NaN,
        safetyAcceptedIterations: Number.isFinite(resultRaw?.safetyAcceptedIterations)
            ? resultRaw.safetyAcceptedIterations
            : 0,
        referenceResidual: cloneArray(resultRaw?.referenceResidual),
        constraintMotion: cloneArray(resultRaw?.constraintMotion),
        qApplied: Array.isArray(resultRaw?.qApplied)
            ? resultRaw.qApplied.map((value, index) => Number.isFinite(value) ? value : (fallbackQApplied[index] ?? 0))
            : cloneArray(fallbackQApplied)
    };
}

export function attemptHumanoidJogMJRestoreWasm({
    humanoidModel,
    solveVariant = 'absolute_target',
    solverMode = 'mj_restore',
    qInitial,
    evaluateProblem,
    activeStart,
    activeCount,
    absoluteTarget,
    relativeTarget,
    leftTarget,
    rightTarget,
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
    jointLimitVFI,
    referenceModel,
    maxIterations,
    damping,
    initialScale,
    minScale,
    reductionFactor,
    initialTrustRadius,
    minTrustRadius,
    maxTrustRadius,
    maxTrustRegionRetries,
    trustRegionShrinkFactor,
    trustRegionExpandFactor,
    rhoAcceptanceThreshold,
    rhoShrinkThreshold,
    rhoExpandThreshold,
    maxDampingRetries,
    dampingGrowthFactor,
    regularizationWeights,
    equalityTolerance,
    inequalityTolerance,
    safetyPushEnabled,
    safetyPushWeight,
    safetyPushActivationResidual,
    safetyPushPoseHoldTolerance,
    safetyPushTargetFraction,
    safetyPushMaxDistanceStep
} = {}) {
    const normalizedSolverMode = normalizeSolverMode(solverMode);
    const normalizedSolveVariant = normalizeSolveVariant(solveVariant);
    const solveFunctionName = getHumanoidJogMJRestoreWasmSolveFunctionName(
        normalizedSolverMode,
        normalizedSolveVariant
    );
    if (!canUseHumanoidJogMJRestoreWasm(humanoidModel, normalizedSolverMode, normalizedSolveVariant) || typeof evaluateProblem !== 'function') {
        return null;
    }

    const preStartMs = nowMs();
    const qInitialActive = cloneArray(qInitial);
    const initialProblem = evaluateProblem(qInitialActive);
    if (!initialProblem || typeof initialProblem !== 'object') {
        return null;
    }

    const request = buildHumanoidJogMJRestoreWasmRequest({
        solveVariant: normalizedSolveVariant,
        solverMode: normalizedSolverMode,
        qCurrent: cloneArray(initialProblem?.metadata?.qFull ?? initialProblem?.qCurrent ?? []),
        activeStart,
        activeCount,
        absoluteTarget,
        relativeTarget,
        leftTarget,
        rightTarget,
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
        jointLimitVFI: jointLimitVFI ?? initialProblem?.inequalities?.jointLimitVFI ?? null,
        waistXCorridor: initialProblem?.inequalities?.waistXCorridor ?? null,
        referenceModel,
        maxIterations,
        damping,
        maxDampingRetries,
        dampingGrowthFactor,
        initialScale,
        minScale,
        reductionFactor,
        initialTrustRadius,
        minTrustRadius,
        maxTrustRadius,
        maxTrustRegionRetries,
        trustRegionShrinkFactor,
        trustRegionExpandFactor,
        rhoAcceptanceThreshold,
        rhoShrinkThreshold,
        rhoExpandThreshold,
        regularizationWeights,
        equalityTolerance,
        inequalityTolerance,
        safetyPushEnabled,
        safetyPushWeight,
        safetyPushActivationResidual,
        safetyPushPoseHoldTolerance,
        safetyPushTargetFraction,
        safetyPushMaxDistanceStep
    });
    const preMs = nowMs() - preStartMs;

    const solveStartMs = nowMs();
    const rawResult = humanoidModel[solveFunctionName](request);
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
        lastAcceptedDamping: Number.isFinite(attempt.lastAcceptedDamping)
            ? attempt.lastAcceptedDamping
            : Number.NaN,
        currentProblem,
        currentResidualNorm: Number.isFinite(attempt.currentResidualNorm)
            ? attempt.currentResidualNorm
            : (Number.isFinite(currentProblem?.equalities?.activeResidualNorm)
                ? currentProblem.equalities.activeResidualNorm
                : Number.POSITIVE_INFINITY),
        trustRadius: Number.isFinite(attempt.trustRadius) ? attempt.trustRadius : Number.NaN,
        lastAcceptedRho: Number.isFinite(attempt.lastAcceptedRho) ? attempt.lastAcceptedRho : Number.NaN,
        lastPredictedReduction: Number.isFinite(attempt.lastPredictedReduction)
            ? attempt.lastPredictedReduction
            : Number.NaN,
        lastActualReduction: Number.isFinite(attempt.lastActualReduction)
            ? attempt.lastActualReduction
            : Number.NaN,
        acceptedBy: typeof attempt.acceptedBy === 'string' ? attempt.acceptedBy : '',
        lastRejectedRho: Number.isFinite(attempt.lastRejectedRho)
            ? attempt.lastRejectedRho
            : Number.NaN,
        lastRejectedPredictedReduction: Number.isFinite(attempt.lastRejectedPredictedReduction)
            ? attempt.lastRejectedPredictedReduction
            : Number.NaN,
        lastRejectedActualReduction: Number.isFinite(attempt.lastRejectedActualReduction)
            ? attempt.lastRejectedActualReduction
            : Number.NaN,
        lastRejectedReason: typeof attempt.lastRejectedReason === 'string' ? attempt.lastRejectedReason : '',
        bestConstraintViolation: Number.isFinite(attempt.bestConstraintViolation)
            ? attempt.bestConstraintViolation
            : Number.POSITIVE_INFINITY,
        filterSize: Number.isFinite(attempt.filterSize) ? attempt.filterSize : 0,
        safetyStatus: typeof attempt.safetyStatus === 'string' && attempt.safetyStatus.trim()
            ? attempt.safetyStatus.trim()
            : 'skipped',
        safetyResidualBefore: Number.isFinite(attempt.safetyResidualBefore)
            ? attempt.safetyResidualBefore
            : Number.NaN,
        safetyResidualAfter: Number.isFinite(attempt.safetyResidualAfter)
            ? attempt.safetyResidualAfter
            : Number.NaN,
        safetyAcceptedIterations: Number.isFinite(attempt.safetyAcceptedIterations)
            ? attempt.safetyAcceptedIterations
            : 0,
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
    solveVariant = 'absolute_target',
    solverMode = 'mj_restore',
    qInitial,
    evaluateProblem,
    activeStart,
    activeCount,
    absoluteTarget,
    relativeTarget,
    leftTarget,
    rightTarget,
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
    jointLimitVFI,
    referenceModel,
    maxIterations,
    damping,
    maxDampingRetries,
    dampingGrowthFactor,
    initialScale,
    minScale,
    reductionFactor,
    initialTrustRadius,
    minTrustRadius,
    maxTrustRadius,
    maxTrustRegionRetries,
    trustRegionShrinkFactor,
    trustRegionExpandFactor,
    rhoAcceptanceThreshold,
    rhoShrinkThreshold,
    rhoExpandThreshold,
    regularizationWeights,
    equalityTolerance,
    inequalityTolerance,
    safetyPushEnabled,
    safetyPushWeight,
    safetyPushActivationResidual,
    safetyPushPoseHoldTolerance,
    safetyPushTargetFraction,
    safetyPushMaxDistanceStep,
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

    const normalizedSolverMode = normalizeSolverMode(solverMode);
    const normalizedSolveVariant = normalizeSolveVariant(solveVariant);
    const request = buildHumanoidJogMJRestoreWasmRequest({
        solveVariant: normalizedSolveVariant,
        solverMode: normalizedSolverMode,
        qCurrent: cloneArray(initialProblem?.metadata?.qFull ?? initialProblem?.qCurrent ?? []),
        activeStart,
        activeCount,
        absoluteTarget,
        relativeTarget,
        leftTarget,
        rightTarget,
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
        jointLimitVFI: jointLimitVFI ?? initialProblem?.inequalities?.jointLimitVFI ?? null,
        waistXCorridor: initialProblem?.inequalities?.waistXCorridor ?? null,
        referenceModel,
        maxIterations,
        damping,
        maxDampingRetries,
        dampingGrowthFactor,
        initialScale,
        minScale,
        reductionFactor,
        initialTrustRadius,
        minTrustRadius,
        maxTrustRadius,
        maxTrustRegionRetries,
        trustRegionShrinkFactor,
        trustRegionExpandFactor,
        rhoAcceptanceThreshold,
        rhoShrinkThreshold,
        rhoExpandThreshold,
        regularizationWeights,
        equalityTolerance,
        inequalityTolerance,
        safetyPushEnabled,
        safetyPushWeight,
        safetyPushActivationResidual,
        safetyPushPoseHoldTolerance,
        safetyPushTargetFraction,
        safetyPushMaxDistanceStep
    });
    const preMs = nowMs() - preStartMs;

    const solveStartMs = nowMs();
    const rawResult = await workerClient.solve({
        modelSpec,
        solverMode: normalizedSolverMode,
        solveVariant: normalizedSolveVariant,
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
