function asFinite(value, fallback = Number.NaN) {
    return Number.isFinite(value) ? value : fallback;
}

export function evaluateHumanoidJogManifoldObjectiveDirection({
    before = Number.NaN,
    after = Number.NaN,
    direction = 'increase',
    tolerance = 1e-12
} = {}) {
    const beforeValue = asFinite(before);
    const afterValue = asFinite(after);
    const delta = Number.isFinite(beforeValue) && Number.isFinite(afterValue)
        ? (afterValue - beforeValue)
        : Number.NaN;
    const tol = Math.max(0, asFinite(tolerance, 1e-12));
    const wantsDecrease = direction === 'decrease';
    const applied = Number.isFinite(delta) && (wantsDecrease ? (delta < -tol) : (delta > tol));
    return {
        before: beforeValue,
        after: afterValue,
        delta,
        applied,
        reason: applied ? 'improved' : 'not_improved'
    };
}

export function evaluateHumanoidJogManifoldNonDegradation({
    before = Number.NaN,
    after = Number.NaN,
    direction = 'increase',
    tolerance = 1e-12
} = {}) {
    const beforeValue = asFinite(before);
    const afterValue = asFinite(after);
    const delta = Number.isFinite(beforeValue) && Number.isFinite(afterValue)
        ? (afterValue - beforeValue)
        : Number.NaN;
    const tol = Math.max(0, asFinite(tolerance, 1e-12));
    const wantsDecrease = direction === 'decrease';
    const applied = Number.isFinite(delta) && (wantsDecrease ? (delta <= tol) : (delta >= -tol));
    return {
        before: beforeValue,
        after: afterValue,
        delta,
        applied,
        reason: applied ? 'protected' : 'degraded'
    };
}
