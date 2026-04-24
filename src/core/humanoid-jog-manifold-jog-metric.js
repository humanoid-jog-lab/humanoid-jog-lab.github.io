const SUPPORTED_MANIFOLD_JOG_METRICS = new Set([
    'none',
    'w_vfi',
    'w_cmd',
    'w_abs',
    'w_abs_arm',
    'w_abs_arm_vfi',
    'w_abs_vfi',
    'w_abs_hier',
    'w_abs_hier_vfi',
    'safety_margin',
    'sigma'
]);

export function normalizeHumanoidJogManifoldJogMetric(metric) {
    const normalized = String(metric ?? '').trim();
    return SUPPORTED_MANIFOLD_JOG_METRICS.has(normalized) ? normalized : 'none';
}

export function resolveHumanoidJogManifoldJogMetric(metric, options = {}) {
    const normalized = normalizeHumanoidJogManifoldJogMetric(metric);
    const requestedMetric = normalized;
    const hierarchicalVfiBlend = Number.isFinite(options?.hierarchicalVfiBlend)
        ? Math.max(0, Math.min(1, options.hierarchicalVfiBlend))
        : Number.NaN;
    if (normalized === 'none') {
        return {
            requestedMetric,
            metric: 'none',
            family: 'none',
            shouldSkip: true,
            diagnosticMetric: 'none'
        };
    }
    if (normalized === 'sigma') {
        return {
            requestedMetric,
            metric: 'sigma',
            family: 'directional',
            shouldSkip: false,
            diagnosticMetric: 'sigma'
        };
    }
    if (normalized === 'w_abs_hier_vfi') {
        return {
            requestedMetric,
            metric: normalized,
            family: 'hierarchical',
            shouldSkip: false,
            diagnosticMetric: normalized
        };
    }
    if (normalized === 'w_abs_hier') {
        return {
            requestedMetric,
            metric: normalized,
            family: 'hierarchical',
            shouldSkip: false,
            diagnosticMetric: normalized
        };
    }
    return {
        requestedMetric,
        metric: normalized,
        family: 'direct',
        shouldSkip: false,
        diagnosticMetric: normalized
    };
}

export function resolveHumanoidJogManifoldJogMetricFallback({
    metric = 'none',
    restorationAccepted = false,
    effectiveAccepted = false,
    jogMetricState = ''
} = {}) {
    const resolved = resolveHumanoidJogManifoldJogMetric(metric);
    const normalizedState = typeof jogMetricState === 'string' ? jogMetricState.trim() : '';
    const preservesHierarchicalPrimary = (
        resolved.family === 'hierarchical' &&
        normalizedState.startsWith('active(primary_only;')
    );
    const preservesProjectionOnlyState = normalizedState.includes('projection_only');
    const shouldFallback = (
        resolved.shouldSkip !== true &&
        preservesHierarchicalPrimary !== true &&
        preservesProjectionOnlyState !== true &&
        restorationAccepted === true &&
        effectiveAccepted !== true
    );
    if (!shouldFallback) {
        return {
            shouldFallback: false,
            state: typeof jogMetricState === 'string' ? jogMetricState : ''
        };
    }
    const baseState = normalizedState.length > 0
        ? normalizedState
        : 'rejected(metric_refine)';
    return {
        shouldFallback: true,
        state: `${baseState}; fallback(restoration_only)`
    };
}
