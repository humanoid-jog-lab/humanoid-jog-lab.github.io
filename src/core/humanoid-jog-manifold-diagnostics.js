import { t } from '../i18n.js';

function asFinite(value, fallback = Number.NaN) {
    return Number.isFinite(value) ? value : fallback;
}

function formatDetailedNumber(value, digits = 6) {
    if (!Number.isFinite(value)) return 'n/a';
    const abs = Math.abs(value);
    if (abs === 0) return '0';
    if (abs >= 1000 || abs < 1e-6) return value.toExponential(2);
    return value.toFixed(digits);
}

export function buildHumanoidJogManifoldOptimizationLabel({
    metric = 'wvfi',
    state = 'skipped(waiting_for_restore)',
    axisLabel = '',
    before = Number.NaN,
    after = Number.NaN,
    applied = false
} = {}) {
    const prefix = metric === 'w' || metric === 'w_cmd'
        ? 'w_cmd_opt'
        : (metric === 'w_abs'
            ? 'w_abs_opt'
            : (metric === 'w_abs_arm'
                ? 'w_abs_arm_opt'
                : (metric === 'w_abs_hier'
                    ? 'w_abs_hier_opt'
                : (metric === 'w_abs_hier_vfi'
                    ? 'w_abs_hier_vfi_opt'
                : (metric === 'w_abs_vfi'
                ? 'w_abs_vfi_opt'
                : (metric === 'sigma' ? 'sigma_opt' : 'wvfi_opt'))))));
    const axisPart = typeof axisLabel === 'string' && axisLabel.trim().length > 0
        ? `  axis=${axisLabel.trim()}`
        : '';
    return `${prefix}: ${state}${axisPart}  before=${formatDetailedNumber(before)}  after=${formatDetailedNumber(after)}  applied=${applied ? 'yes' : 'no'}`;
}

export function buildHumanoidJogManifoldWvfiOptimizationLabel({
    state = 'skipped(waiting_for_restore)',
    before = Number.NaN,
    after = Number.NaN,
    applied = false
} = {}) {
    return buildHumanoidJogManifoldOptimizationLabel({
        metric: 'wvfi',
        state,
        before,
        after,
        applied
    });
}

export function buildHumanoidJogManifoldDiagnostics({
    modeTag,
    includeWaist = false,
    keepBodyZVertical = false,
    jointLimitAvoidance = true,
    restorationResult = null,
    nullspaceResult = null,
    optimizationMetric = 'wvfi',
    optimizationAxis = '',
    restoredWVFI = Number.NaN,
    refinedWVFI = Number.NaN,
    wvfiOptimizationApplied = false,
    jogMetric = '',
    jogMetricState = '',
    jogMetricBefore = Number.NaN,
    jogMetricAfter = Number.NaN,
    jogMetricApplied = false,
    jogMetricAxis = '',
    fallbackReason = null,
    cctsmResult = null,
    relativeResidualNorm = Number.NaN,
    bodyResidualNorm = Number.NaN,
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
    const resolvedOptimizationMetric = typeof optimizationMetric === 'string'
        ? optimizationMetric.trim()
        : '';
    const wvfiOptimizationState = restorationResult?.converged === true
        ? (nullspaceResult
            ? (nullspaceResult.accepted === true ? 'active' : `skipped(${nullspaceResult.reason ?? 'unknown'})`)
            : 'skipped(no_nullspace_phase)')
        : 'skipped(waiting_for_restore)';
    const optimizationLabel = resolvedOptimizationMetric
        ? buildHumanoidJogManifoldOptimizationLabel({
            metric: resolvedOptimizationMetric,
            state: wvfiOptimizationState,
            axisLabel: optimizationAxis,
            before: restoredWVFI,
            after: refinedWVFI,
            applied: wvfiOptimizationApplied === true
        })
        : '';
    return {
        solverMode: 'manifold',
        modeTag,
        active: true,
        message: t('Constraint manifold diagnostics updated.'),
        includeWaist,
        keepBodyZVertical,
        jointLimitAvoidance,
        restoreIterations: Number.isFinite(restorationResult?.iterations) ? restorationResult.iterations : 0,
        restoreAcceptedIterations: Number.isFinite(restorationResult?.acceptedIterations) ? restorationResult.acceptedIterations : 0,
        restoreResidualNorm: asFinite(restorationResult?.currentResidualNorm),
        restoreScale: asFinite(restorationResult?.lastAcceptedScale),
        stopReason: typeof restorationResult?.reason === 'string' ? restorationResult.reason : 'unknown',
        nullspaceAccepted: nullspaceResult?.accepted === true,
        nullspaceReason: typeof nullspaceResult?.reason === 'string' ? nullspaceResult.reason : null,
        nullspaceStepNorm: asFinite(nullspaceResult?.stepNorm),
        optimizationMetric: resolvedOptimizationMetric,
        optimizationAxis,
        optimizationState: resolvedOptimizationMetric ? wvfiOptimizationState : '',
        optimizationLabel,
        optimizationBefore: resolvedOptimizationMetric ? asFinite(restoredWVFI) : Number.NaN,
        optimizationAfter: resolvedOptimizationMetric ? asFinite(refinedWVFI) : Number.NaN,
        optimizationApplied: resolvedOptimizationMetric ? (wvfiOptimizationApplied === true) : false,
        jogMetric: typeof jogMetric === 'string' ? jogMetric : '',
        jogMetricState: typeof jogMetricState === 'string' ? jogMetricState : '',
        jogMetricBefore: asFinite(jogMetricBefore),
        jogMetricAfter: asFinite(jogMetricAfter),
        jogMetricApplied: jogMetricApplied === true,
        jogMetricAxis: typeof jogMetricAxis === 'string' ? jogMetricAxis : '',
        wvfiOptimizationState,
        wvfiOptimizationLabel: resolvedOptimizationMetric === 'wvfi' ? optimizationLabel : '',
        restoredWVFI: asFinite(restoredWVFI),
        refinedWVFI: asFinite(refinedWVFI),
        wvfiOptimizationApplied: wvfiOptimizationApplied === true,
        fallbackReason: typeof fallbackReason === 'string' ? fallbackReason : null,
        relativeResidualNorm: asFinite(relativeResidualNorm),
        bodyResidualNorm: asFinite(bodyResidualNorm),
        wvfiFloorEnabled: wvfiFloorEnabled === true,
        wvfiFloorValue: asFinite(wvfiFloorValue),
        wvfiFloorLowerBound: asFinite(wvfiFloorLowerBound),
        wvfiFloorSlack: asFinite(wvfiFloorSlack),
        wvfiFloorViolation: asFinite(wvfiFloorViolation),
        waistXEnabled: waistXEnabled === true,
        waistXPx: asFinite(waistXPx),
        waistXDupper: asFinite(waistXDupper),
        waistXDlower: asFinite(waistXDlower),
        waistXViolation: asFinite(waistXViolation),
        cctsmW: asFinite(cctsmResult?.w),
        cctsmWCmd: asFinite(cctsmResult?.w_cmd),
        cctsmWAbs: asFinite(cctsmResult?.w_abs),
        cctsmWAbsHier: asFinite(cctsmResult?.w_abs_hier),
        cctsmWAbsHierVfi: asFinite(cctsmResult?.w_abs_hier_vfi),
        cctsmWAbsArm: asFinite(cctsmResult?.w_abs_arm),
        cctsmWAbsArmVfi: asFinite(cctsmResult?.w_abs_arm_vfi),
        cctsmWAbsVfi: asFinite(cctsmResult?.w_abs_vfi),
        cctsmWvfi: asFinite(cctsmResult?.w_vfi)
    };
}
