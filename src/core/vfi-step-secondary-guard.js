function sanitizePositiveTolerance(value, fallback) {
    return Number.isFinite(value) && value > 0 ? value : fallback;
}

export function buildEqualityToleranceCandidates(eqTol) {
    const baseTol = sanitizePositiveTolerance(eqTol, 1e-7);
    return [baseTol];
}

export function shouldRetryWithoutSecondaryObjective({
    useInvariantEqualityInQP = false,
    hasSecondaryObjective = false,
    qpConverged = true,
    equalityToleranceRelaxed = false,
    usedSoftInvariantFallback = false
} = {}) {
    if (!useInvariantEqualityInQP || !hasSecondaryObjective) return false;
    return qpConverged === false || equalityToleranceRelaxed || usedSoftInvariantFallback;
}
