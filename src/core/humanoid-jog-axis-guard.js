function norm3(vector) {
    if (!Array.isArray(vector) || vector.length < 3) return Number.NaN;
    const x = vector[0] || 0;
    const y = vector[1] || 0;
    const z = vector[2] || 0;
    return Math.sqrt(x * x + y * y + z * z);
}

export function evaluateTranslationAxisGuard(options = {}) {
    const commandDelta = Array.isArray(options.commandDelta) ? options.commandDelta : null;
    const actualDelta = Array.isArray(options.actualDelta) ? options.actualDelta : null;
    const minCommandNorm = Number.isFinite(options.minCommandNorm) ? Math.max(0, options.minCommandNorm) : 1e-9;
    const minProgressRatio = Number.isFinite(options.minProgressRatio) ? Math.max(0, options.minProgressRatio) : 0.05;
    const maxOrthogonalRatio = Number.isFinite(options.maxOrthogonalRatio) ? Math.max(0, options.maxOrthogonalRatio) : 0.35;
    const orthogonalRatioBasis = options.orthogonalRatioBasis === 'progress' ? 'progress' : 'command';

    const commandNorm = norm3(commandDelta);
    if (!Number.isFinite(commandNorm) || commandNorm <= minCommandNorm) {
        return {
            violated: false,
            reason: null,
            commandNorm: Number.NaN,
            projectedProgress: Number.NaN,
            orthogonalDrift: Number.NaN,
            progressRatio: Number.NaN,
            orthogonalRatio: Number.NaN
        };
    }

    const actual = Array.isArray(actualDelta) ? actualDelta : [0, 0, 0];
    const unit = [
        (commandDelta[0] || 0) / commandNorm,
        (commandDelta[1] || 0) / commandNorm,
        (commandDelta[2] || 0) / commandNorm
    ];
    const projectedProgress = (actual[0] || 0) * unit[0] + (actual[1] || 0) * unit[1] + (actual[2] || 0) * unit[2];
    const orthogonal = [
        (actual[0] || 0) - projectedProgress * unit[0],
        (actual[1] || 0) - projectedProgress * unit[1],
        (actual[2] || 0) - projectedProgress * unit[2]
    ];
    const orthogonalDrift = norm3(orthogonal);
    const progressRatio = projectedProgress / commandNorm;
    const orthogonalDenominator = orthogonalRatioBasis === 'progress'
        ? Math.max(minCommandNorm, Math.abs(projectedProgress))
        : commandNorm;
    const orthogonalRatio = orthogonalDrift / orthogonalDenominator;

    let reason = null;
    if (!Number.isFinite(projectedProgress) || progressRatio < minProgressRatio) {
        reason = 'insufficient_progress';
    } else if (Number.isFinite(orthogonalRatio) && orthogonalRatio > maxOrthogonalRatio) {
        reason = 'orthogonal_drift';
    }

    return {
        violated: reason !== null,
        reason,
        commandNorm,
        projectedProgress,
        orthogonalDrift,
        progressRatio,
        orthogonalRatio
    };
}
