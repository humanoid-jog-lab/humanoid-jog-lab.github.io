function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

export function buildHumanoidJogMJRegularizationWeights({
    activeCount = 0,
    waistActiveCount = 0,
    includeWaist = false,
    keepBodyZVertical = false,
    isMetricOnlyJogRequest = false,
    metricShouldSkip = true,
    config = {}
} = {}) {
    const width = Math.max(0, Math.floor(asFinite(activeCount, 0)));
    const waistCount = Math.max(0, Math.min(width, Math.floor(asFinite(waistActiveCount, 0))));
    if (
        width <= 0 ||
        waistCount <= 0 ||
        waistCount >= width ||
        includeWaist !== true ||
        keepBodyZVertical !== true ||
        isMetricOnlyJogRequest === true ||
        metricShouldSkip !== true
    ) {
        return null;
    }

    const waistWeight = Math.max(1, asFinite(config?.mjRestoreWaistMotionRegularization, 6));
    const armWeight = Math.max(1e-12, asFinite(config?.mjRestoreArmMotionRegularization, 1));
    const weights = new Array(width).fill(armWeight);
    for (let i = 0; i < waistCount; i++) weights[i] = waistWeight;
    return weights;
}
