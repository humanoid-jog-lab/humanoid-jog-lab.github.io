function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

export function buildHumanoidJogManifoldStepBounds({
    maxStep = 0.1,
    includeWaist = false,
    keepBodyZVertical = false,
    waistCount = 0,
    activeCount = 0,
    waistStepFraction = 1.0
} = {}) {
    const width = Math.max(0, Math.floor(asFinite(activeCount, 0)));
    const step = Math.max(0, asFinite(maxStep, 0.1));
    const bounds = new Array(width).fill(step);
    if (!includeWaist || keepBodyZVertical || width === 0) return bounds;

    const fraction = Math.max(0, Math.min(1, asFinite(waistStepFraction, 1.0)));
    const waistWidth = Math.max(0, Math.min(width, Math.floor(asFinite(waistCount, 0))));
    for (let i = 0; i < waistWidth; i++) bounds[i] = step * fraction;
    return bounds;
}
