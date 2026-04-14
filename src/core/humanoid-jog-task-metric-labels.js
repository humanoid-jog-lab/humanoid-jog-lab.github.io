function normalizeHumanoidTaskMetricContext(contextRaw = '') {
    const context = String(contextRaw || '').trim().toLowerCase();
    if (
        context === 'relative'
        || context === 'relative-fullbody'
        || context === 'relative-target'
    ) {
        return 'relative';
    }
    if (
        context === 'distance'
        || context === 'distance-fullbody'
        || context === 'distance-target'
    ) {
        return 'distance';
    }
    if (context === 'robot1' || context === 'robot1-target' || context === 'robot1-waist' || context === 'robot1-waist-target') {
        return 'robot1';
    }
    if (context === 'robot2' || context === 'robot2-target' || context === 'robot2-waist' || context === 'robot2-waist-target') {
        return 'robot2';
    }
    if (context === 'waist-body-link-xz' || context === 'waist-body-link-xz-target') {
        return 'waist';
    }
    if (
        context === 'surface-slide'
        || context === 'surface-slide-target'
        || context === 'dual-independent-surface-slide'
        || context === 'dual-independent-surface-slide-target'
        || context === 'cooperative-ruled-surface-polishing'
        || context === 'cooperative-ruled-surface-polishing-target'
    ) {
        return 'surface';
    }
    return 'absolute';
}

function getHumanoidTaskMetricPrefix(contextRaw = '') {
    switch (normalizeHumanoidTaskMetricContext(contextRaw)) {
        case 'relative':
            return 'rel';
        case 'distance':
            return 'rel';
        case 'robot1':
            return 'left';
        case 'robot2':
            return 'right';
        case 'waist':
            return 'waist';
        case 'surface':
            return 'surface';
        case 'absolute':
        default:
            return 'abs';
    }
}

export function getHumanoidTaskMetricDisplayName(metric = '', contextRaw = '') {
    const prefix = getHumanoidTaskMetricPrefix(contextRaw);
    if (metric === 'w_abs_hier_vfi') {
        return `w_${prefix}_vfi`;
    }
    if (metric === 'w_abs_hier') {
        return `w_${prefix}`;
    }
    return metric;
}

export function getHumanoidTaskMetricOptionLabels(contextRaw = '') {
    return {
        w_abs_hier: getHumanoidTaskMetricDisplayName('w_abs_hier', contextRaw),
        w_abs_hier_vfi: getHumanoidTaskMetricDisplayName('w_abs_hier_vfi', contextRaw)
    };
}
