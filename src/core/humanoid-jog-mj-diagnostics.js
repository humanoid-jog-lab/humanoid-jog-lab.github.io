import { getHumanoidTaskMetricDisplayName } from './humanoid-jog-task-metric-labels.js';
import { t } from '../i18n.js';

function asFinite(value, fallback = Number.NaN) {
    return Number.isFinite(value) ? value : fallback;
}

function formatNumber(value, digits = 2) {
    if (!Number.isFinite(value)) return 'n/a';
    return value.toFixed(digits);
}

export function buildHumanoidJogMJDiagnostics({
    solverMode = 'mj_restore',
    modeTag = 'absolute-target',
    includeWaist = false,
    keepBodyZVertical = false,
    restorationResult = null,
    nullspaceResult = null,
    safetyRefineResult = null,
    referenceModel = null,
    jogMetric = '',
    jogMetricState = '',
    jogMetricBefore = Number.NaN,
    jogMetricAfter = Number.NaN,
    jogMetricApplied = false,
    jogMetricAxis = '',
    hierarchicalVfiBlend = Number.NaN,
    preWState = '',
    preWMetric = '',
    preWStart = '',
    relativeResidualNorm = Number.NaN,
    bodyResidualNorm = Number.NaN,
    polishState = '',
    polishReason = '',
    polishIterations = Number.NaN,
    polishResidualNorm = Number.NaN,
    timingPreMs = Number.NaN,
    timingWorkerInitMs = Number.NaN,
    timingSolveWaitMs = Number.NaN,
    timingSolveMs = Number.NaN,
    timingPostMs = Number.NaN,
    timingCCTSMMs = Number.NaN,
    timingDiagnosticMs = Number.NaN,
    timingTotalMs = Number.NaN,
    vfiJointLimitCount = Number.NaN,
    vfiExtraCount = Number.NaN,
    vfiTotalCount = Number.NaN,
    vfiMinDistance = Number.NaN,
    wvfiFloorEnabled = false,
    wvfiFloorValue = Number.NaN,
    wvfiFloorLowerBound = Number.NaN,
    wvfiFloorSlack = Number.NaN,
    wvfiFloorViolation = Number.NaN,
    waistXEnabled = false,
    waistXPx = Number.NaN,
    waistXDupper = Number.NaN,
    waistXDlower = Number.NaN,
    waistXViolation = Number.NaN
} = {}) {
    const normalizedSolverMode = solverMode === 'mj_restore_pre_w'
        ? 'mj_restore_pre_w'
        : (solverMode === 'mj_restore_tr_safe'
            ? 'mj_restore_tr_safe'
            : (solverMode === 'mj_restore_tr' ? 'mj_restore_tr' : 'mj_restore'));
    return {
        solverMode: normalizedSolverMode,
        modeTag,
        message: t('Restore (LS) diagnostics updated.'),
        includeWaist,
        keepBodyZVertical,
        restoreIterations: Number.isFinite(restorationResult?.iterations) ? restorationResult.iterations : 0,
        restoreAcceptedIterations: Number.isFinite(restorationResult?.acceptedIterations) ? restorationResult.acceptedIterations : 0,
        restoreResidualNorm: asFinite(restorationResult?.currentResidualNorm),
        restoreScale: asFinite(restorationResult?.lastAcceptedScale),
        stopReason: typeof restorationResult?.reason === 'string' ? restorationResult.reason : 'unknown',
        restoreAcceptedRho: asFinite(restorationResult?.lastAcceptedRho),
        restorePredictedReduction: asFinite(restorationResult?.lastPredictedReduction),
        restoreActualReduction: asFinite(restorationResult?.lastActualReduction),
        restoreAcceptedBy: typeof restorationResult?.acceptedBy === 'string'
            ? restorationResult.acceptedBy
            : '',
        restoreRejectedRho: asFinite(restorationResult?.lastRejectedRho),
        restoreRejectedPredictedReduction: asFinite(restorationResult?.lastRejectedPredictedReduction),
        restoreRejectedActualReduction: asFinite(restorationResult?.lastRejectedActualReduction),
        restoreRejectedReason: typeof restorationResult?.lastRejectedReason === 'string'
            ? restorationResult.lastRejectedReason
            : '',
        trSafetyStatus: typeof restorationResult?.safetyStatus === 'string'
            ? restorationResult.safetyStatus
            : 'skipped',
        safetyResidualBefore: asFinite(restorationResult?.safetyResidualBefore),
        safetyResidualAfter: asFinite(restorationResult?.safetyResidualAfter),
        safetyAcceptedIterations: Number.isFinite(restorationResult?.safetyAcceptedIterations)
            ? restorationResult.safetyAcceptedIterations
            : 0,
        safetyRefineState: typeof safetyRefineResult?.state === 'string'
            ? safetyRefineResult.state
            : '',
        safetyRefineReason: typeof safetyRefineResult?.reason === 'string'
            ? safetyRefineResult.reason
            : '',
        safetyRefineApplied: safetyRefineResult?.applied === true,
        safetyRefineRows: Number.isFinite(safetyRefineResult?.rowCount)
            ? safetyRefineResult.rowCount
            : 0,
        safetyRefineMarginBefore: asFinite(safetyRefineResult?.marginBefore),
        safetyRefineMarginAfter: asFinite(safetyRefineResult?.marginAfter),
        safetyRefineObjectiveBefore: asFinite(safetyRefineResult?.objectiveBefore),
        safetyRefineObjectiveAfter: asFinite(safetyRefineResult?.objectiveAfter),
        safetyRefineTaskCorrectionApplied: safetyRefineResult?.taskCorrectionResult?.accepted === true ||
            safetyRefineResult?.refineResult?.safetyRefineTaskCorrectionApplied === true,
        polishIterations: asFinite(polishIterations),
        nullspaceAccepted: nullspaceResult?.accepted === true,
        nullspaceReason: typeof nullspaceResult?.reason === 'string' ? nullspaceResult.reason : null,
        nullspaceStepNorm: asFinite(nullspaceResult?.stepNorm),
        relativeResidualNorm: asFinite(relativeResidualNorm),
        bodyResidualNorm: asFinite(bodyResidualNorm),
        referenceAbsoluteStiffness: asFinite(referenceModel?.absolute?.stiffness),
        referenceAbsoluteDamping: asFinite(referenceModel?.absolute?.damping),
        referenceRelativeStiffness: asFinite(referenceModel?.relative?.stiffness),
        referenceRelativeDamping: asFinite(referenceModel?.relative?.damping),
        referenceBodyStiffness: asFinite(referenceModel?.bodyZ?.stiffness),
        referenceBodyDamping: asFinite(referenceModel?.bodyZ?.damping),
        referenceResidual: Array.isArray(restorationResult?.lastReferenceResidual)
            ? restorationResult.lastReferenceResidual.slice()
            : [],
        constraintMotion: Array.isArray(restorationResult?.lastConstraintMotion)
            ? restorationResult.lastConstraintMotion.slice()
            : [],
        referenceSummary: `mj_ref: abs(k=${formatNumber(referenceModel?.absolute?.stiffness)},b=${formatNumber(referenceModel?.absolute?.damping)}) rel(k=${formatNumber(referenceModel?.relative?.stiffness)},b=${formatNumber(referenceModel?.relative?.damping)}) body(k=${formatNumber(referenceModel?.bodyZ?.stiffness)},b=${formatNumber(referenceModel?.bodyZ?.damping)})`,
        jogMetric,
        jogMetricState,
        jogMetricBefore: asFinite(jogMetricBefore),
        jogMetricAfter: asFinite(jogMetricAfter),
        jogMetricApplied: jogMetricApplied === true,
        jogMetricAxis,
        taskMetricLabel: getHumanoidTaskMetricDisplayName(
            typeof jogMetric === 'string' ? jogMetric : '',
            modeTag
        ),
        hierarchicalVfiBlend: asFinite(hierarchicalVfiBlend),
        preWState: typeof preWState === 'string' ? preWState : '',
        preWMetric: typeof preWMetric === 'string' ? preWMetric : '',
        taskPreWMetricLabel: getHumanoidTaskMetricDisplayName(
            typeof preWMetric === 'string' ? preWMetric : '',
            modeTag
        ),
        preWStart: typeof preWStart === 'string' ? preWStart : '',
        polishState: typeof polishState === 'string' ? polishState : '',
        polishReason: typeof polishReason === 'string' ? polishReason : '',
        polishResidualNorm: asFinite(polishResidualNorm),
        timingPreMs: asFinite(timingPreMs),
        timingWorkerInitMs: asFinite(timingWorkerInitMs),
        timingSolveWaitMs: asFinite(timingSolveWaitMs),
        timingSolveMs: asFinite(timingSolveMs),
        timingPostMs: asFinite(timingPostMs),
        timingCCTSMMs: asFinite(timingCCTSMMs),
        timingDiagnosticMs: asFinite(timingDiagnosticMs),
        timingTotalMs: asFinite(timingTotalMs),
        vfiJointLimitCount: asFinite(vfiJointLimitCount),
        vfiExtraCount: asFinite(vfiExtraCount),
        vfiTotalCount: asFinite(vfiTotalCount),
        vfiMinDistance: asFinite(vfiMinDistance),
        wvfiFloorEnabled: wvfiFloorEnabled === true,
        wvfiFloorValue: asFinite(wvfiFloorValue),
        wvfiFloorLowerBound: asFinite(wvfiFloorLowerBound),
        wvfiFloorSlack: asFinite(wvfiFloorSlack),
        wvfiFloorViolation: asFinite(wvfiFloorViolation),
        waistXEnabled: waistXEnabled === true,
        waistXPx: asFinite(waistXPx),
        waistXDupper: asFinite(waistXDupper),
        waistXDlower: asFinite(waistXDlower),
        waistXViolation: asFinite(waistXViolation)
    };
}
