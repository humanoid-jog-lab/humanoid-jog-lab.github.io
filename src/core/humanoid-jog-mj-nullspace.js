export function shouldEnableHumanoidJogMJNullspaceRefine({
    enabled = true,
    metric = 'none'
} = {}) {
    if (enabled === true) return true;
    return metric === 'w_abs_hier_vfi' || metric === 'w_abs_hier' || metric === 'w_abs_arm' || metric === 'w_abs_arm_vfi' || metric === 'w_abs_vfi';
}

export function shouldUseHumanoidJogMJProjectionOnlyMetricStep({
    isMetricOnlyJogRequest = false,
    requestedMetric = 'none',
    stageMetric = 'none'
} = {}) {
    if (isMetricOnlyJogRequest !== true) return false;
    const requested = typeof requestedMetric === 'string' ? requestedMetric.trim() : '';
    const stage = typeof stageMetric === 'string' ? stageMetric.trim() : '';
    if (requested !== 'w_abs_hier_vfi') return false;
    return stage === 'safety_margin' || stage === 'w_abs_arm_vfi' || stage === 'w_abs_hier_vfi';
}

export function resolveHumanoidJogMJProjectionPasses({
    isMetricOnlyJogRequest = false,
    requestedMetric = 'none',
    configuredPasses = 4
} = {}) {
    const normalizedPasses = Number.isFinite(configuredPasses)
        ? Math.max(1, Math.floor(configuredPasses))
        : 4;
    const requested = typeof requestedMetric === 'string' ? requestedMetric.trim() : '';
    if (isMetricOnlyJogRequest === true && requested === 'w_abs_hier_vfi') {
        return 1;
    }
    return normalizedPasses;
}

export function resolveHumanoidJogMJProjectionRetreatRestoreConfig({
    projectionMetric = 'none',
    configuredMaxIterations = 8
} = {}) {
    const normalizedIterations = Number.isFinite(configuredMaxIterations)
        ? Math.max(1, Math.floor(configuredMaxIterations))
        : 8;
    const metric = typeof projectionMetric === 'string' ? projectionMetric.trim() : '';
    if (metric === 'safety_margin' || metric === 'w_abs_arm_vfi' || metric === 'w_abs_hier_vfi') {
        return {
            useManifoldRestore: true,
            maxIterations: Math.min(normalizedIterations, 2)
        };
    }
    return {
        useManifoldRestore: false,
        maxIterations: normalizedIterations
    };
}

export function shouldBypassHumanoidJogMJMetricOnlyRestoreSolve({
    isMetricOnlyJogRequest = false,
    workerAvailable = false,
    activeResidualNorm = Number.POSITIVE_INFINITY,
    inequalityViolation = Number.POSITIVE_INFINITY,
    equalityTolerance = 1e-12,
    inequalityTolerance = 1e-12
} = {}) {
    if (isMetricOnlyJogRequest !== true) return false;
    if (workerAvailable === true) return false;
    return Number.isFinite(activeResidualNorm) &&
        activeResidualNorm <= Math.max(0, equalityTolerance) &&
        Number.isFinite(inequalityViolation) &&
        inequalityViolation <= Math.max(0, inequalityTolerance);
}

export function shouldUseHumanoidJogMJMetricOnlyVfiFallbackHeuristics({
    isMetricOnlyJogRequest = false,
    requestedMetric = 'none'
} = {}) {
    const requested = typeof requestedMetric === 'string' ? requestedMetric.trim() : '';
    if (isMetricOnlyJogRequest === true && requested === 'w_abs_hier_vfi') {
        return false;
    }
    return true;
}

export function shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt({
    metric = 'none',
    isMetricOnlyJogRequest = false,
    workerMetricAttempt = null
} = {}) {
    if (isMetricOnlyJogRequest !== true) return false;
    const normalizedMetric = typeof metric === 'string' ? metric.trim() : '';
    if (normalizedMetric !== 'w_abs_hier_vfi') return false;
    if (!workerMetricAttempt || typeof workerMetricAttempt !== 'object') return false;
    if (workerMetricAttempt.applied === true) return false;
    const state = typeof workerMetricAttempt.state === 'string' ? workerMetricAttempt.state.trim() : '';
    return state.startsWith('rejected(hard_constraints)');
}

export function shouldComputeHumanoidJogMJTrialManipData({
    metric = 'none',
    nullspaceRefineEnabled = false,
    wvfiLowerBound = 0
} = {}) {
    if (Number.isFinite(wvfiLowerBound) && wvfiLowerBound > 0) return true;
    if (nullspaceRefineEnabled !== true) return false;
    return metric !== 'none';
}

export function shouldComputeHumanoidJogMJInitialManipData({
    metric = 'none',
    nullspaceRefineEnabled = false,
    wvfiLowerBound = 0,
    workerHandlesMetric = false
} = {}) {
    if (workerHandlesMetric === true && (metric === 'w_abs_hier_vfi' || metric === 'w_abs_hier' || metric === 'w_abs_arm' || metric === 'w_abs_arm_vfi')) {
        return false;
    }
    return shouldComputeHumanoidJogMJTrialManipData({
        metric,
        nullspaceRefineEnabled,
        wvfiLowerBound
    });
}

export function buildHumanoidJogMJWorkerSecondaryMetricRequest({
    metric = 'none',
    metricMode = 'absolute',
    preferWorker = false,
    workerAvailable = false,
    nullspaceRefineEnabled = false,
    stepSize = 0.01,
    restoreMaxIterations = 24,
    waistActiveCount = 0,
    hierarchicalVfiBlend = 0.5
} = {}) {
    if (preferWorker !== true || workerAvailable !== true || nullspaceRefineEnabled !== true) {
        return null;
    }
    const normalizedMetric = typeof metric === 'string' ? metric.trim() : '';
    if (normalizedMetric !== 'w_abs_arm' &&
        normalizedMetric !== 'w_abs_arm_vfi' &&
        normalizedMetric !== 'w_abs_hier') {
        return null;
    }
    const normalizedMetricMode = typeof metricMode === 'string' && metricMode.trim().length > 0
        ? metricMode.trim()
        : 'absolute';
    return {
        metric: normalizedMetric,
        metricMode: normalizedMetricMode,
        stepSize: Number.isFinite(stepSize) ? Math.max(0, stepSize) : 0.01,
        restoreMaxIterations: Number.isFinite(restoreMaxIterations)
            ? Math.max(1, Math.floor(restoreMaxIterations))
            : 24,
        waistActiveCount: Number.isFinite(waistActiveCount)
            ? Math.max(0, Math.floor(waistActiveCount))
            : 0,
        hierarchicalVfiBlend: Number.isFinite(hierarchicalVfiBlend)
            ? Math.max(0, Math.min(1, hierarchicalVfiBlend))
            : 0.5
    };
}

export function computeHumanoidJogMJNullspaceRefine({
    enabled = true,
    restorationConverged = false,
    metricAttempt = null
} = {}) {
    if (enabled !== true) {
        return {
            applied: false,
            state: 'skipped(disabled_for_mj_restore)',
            before: Number.NaN,
            after: Number.NaN,
            axisLabel: '',
            nullspaceResult: null,
            refineResult: null,
            finalProblem: null
        };
    }
    if (restorationConverged !== true) {
        return {
            applied: false,
            state: 'skipped(waiting_for_restore)',
            before: Number.NaN,
            after: Number.NaN,
            axisLabel: '',
            nullspaceResult: null,
            refineResult: null,
            finalProblem: null
        };
    }
    if (!metricAttempt || typeof metricAttempt !== 'object') {
        return {
            applied: false,
            state: 'skipped(no_metric_attempt)',
            before: Number.NaN,
            after: Number.NaN,
            axisLabel: '',
            nullspaceResult: null,
            refineResult: null,
            finalProblem: null
        };
    }
    return {
        applied: metricAttempt.applied === true,
        state: typeof metricAttempt.state === 'string' ? metricAttempt.state : 'unknown',
        before: Number.isFinite(metricAttempt.before) ? metricAttempt.before : Number.NaN,
        after: Number.isFinite(metricAttempt.after) ? metricAttempt.after : Number.NaN,
        axisLabel: typeof metricAttempt.axisLabel === 'string' ? metricAttempt.axisLabel : '',
        nullspaceResult: metricAttempt.nullspaceResult ?? null,
        refineResult: metricAttempt.refineResult ?? null,
        finalProblem: metricAttempt.finalProblem ?? null
    };
}
