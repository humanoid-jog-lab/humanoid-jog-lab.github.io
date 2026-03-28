function clampNonNegative(value, fallback) {
    const numeric = Number.isFinite(value) ? value : fallback;
    return Math.max(0, numeric);
}

export function resolveHumanoidJogMJMetricTolerances({
    isMetricOnlyJogRequest = false,
    isHierarchicalVFIJogMetric = false,
    hardTolerance = 1e-12,
    metricButtonCommandTolerance = 1e-3,
    metricButtonInvariantTolerance = 1e-4,
    activeLimitRetreatCommandTolerance = 1e-2,
    activeLimitRetreatInvariantTolerance = 2e-3
} = {}) {
    const hard = clampNonNegative(hardTolerance, 1e-12);
    const metricCommand = clampNonNegative(metricButtonCommandTolerance, 1e-3);
    const metricInvariant = clampNonNegative(metricButtonInvariantTolerance, 1e-4);
    const retreatCommand = clampNonNegative(activeLimitRetreatCommandTolerance, 1e-2);
    const retreatInvariant = clampNonNegative(activeLimitRetreatInvariantTolerance, 2e-3);

    if (isMetricOnlyJogRequest !== true) {
        return {
            metricRefinementTolerance: hard,
            acceptanceCommandTolerance: hard,
            acceptanceRelativeTolerance: hard,
            activeLimitRetreatCommandTolerance: retreatCommand,
            activeLimitRetreatInvariantTolerance: retreatInvariant
        };
    }

    const metricRefinementTolerance = isHierarchicalVFIJogMetric === true
        ? Math.min(metricCommand, metricInvariant)
        : metricCommand;

    return {
        metricRefinementTolerance,
        acceptanceCommandTolerance: metricCommand,
        acceptanceRelativeTolerance: metricInvariant,
        activeLimitRetreatCommandTolerance: retreatCommand,
        activeLimitRetreatInvariantTolerance: retreatInvariant
    };
}
