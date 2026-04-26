function asFinite(value, fallback = Number.POSITIVE_INFINITY) {
    return Number.isFinite(value) ? value : fallback;
}

export function resolveHumanoidJogWaistXCorridorCommandHold({
    waistXCorridorEnabled = false,
    waistXPosition = Number.NaN,
    waistXMin = Number.NEGATIVE_INFINITY,
    waistXMax = Number.POSITIVE_INFINITY,
    edgeTolerance = 0,
    commandDelta = null
} = {}) {
    if (waistXCorridorEnabled !== true || !Number.isFinite(waistXPosition)) {
        return { hold: false, bound: '', reason: 'not_applicable' };
    }
    const commandX = Array.isArray(commandDelta) ? Number(commandDelta[0]) : Number.NaN;
    if (!Number.isFinite(commandX) || Math.abs(commandX) <= 1e-12) {
        return { hold: false, bound: '', reason: 'no_x_command' };
    }
    const tolerance = Math.max(Math.abs(commandX), Math.max(0, asFinite(edgeTolerance, 0)));
    const nearOrBelowLower = Number.isFinite(waistXMin) && waistXPosition <= waistXMin + tolerance;
    if (nearOrBelowLower && commandX < 0) {
        return { hold: true, bound: 'lower', reason: 'waist_corridor_lower_command_hold' };
    }
    const nearOrAboveUpper = Number.isFinite(waistXMax) && waistXPosition >= waistXMax - tolerance;
    if (nearOrAboveUpper && commandX > 0) {
        return { hold: true, bound: 'upper', reason: 'waist_corridor_upper_command_hold' };
    }
    return { hold: false, bound: '', reason: 'not_pushing_outward' };
}

export function resolveHumanoidJogManifoldArmOnlyFallback({
    includeWaist = false,
    converged = false,
    keepBodyZVertical = false,
    currentBodyResidualNorm = Number.POSITIVE_INFINITY,
    initialBodyResidualNorm = Number.POSITIVE_INFINITY,
    bodyTolerance = 5e-5,
    waistXCorridorEnabled = false,
    waistXPosition = Number.NaN,
    waistXMin = Number.NEGATIVE_INFINITY,
    waistXMax = Number.POSITIVE_INFINITY,
    waistXActivationThreshold = 0,
    commandDelta = null
} = {}) {
    if (!includeWaist || converged) {
        return {
            shouldFallback: false,
            keepBodyZVertical: false,
            reason: 'not_applicable'
        };
    }

    if (waistXCorridorEnabled === true && Number.isFinite(waistXPosition)) {
        const activation = Math.max(0, asFinite(waistXActivationThreshold, 0));
        const upperDistance = Number.isFinite(waistXMax) ? waistXMax - waistXPosition : Number.POSITIVE_INFINITY;
        const lowerDistance = Number.isFinite(waistXMin) ? waistXPosition - waistXMin : Number.POSITIVE_INFINITY;
        const commandX = Array.isArray(commandDelta) ? asFinite(commandDelta[0], Number.NaN) : Number.NaN;
        const pushesIntoUpperBound = upperDistance <= activation && Number.isFinite(commandX) && commandX > 0;
        const pushesIntoLowerBound = lowerDistance <= activation && Number.isFinite(commandX) && commandX < 0;
        if (pushesIntoUpperBound || pushesIntoLowerBound) {
            return {
                shouldFallback: true,
                keepBodyZVertical: false,
                reason: 'waist_corridor_saturated_arm_only'
            };
        }
        if (upperDistance <= activation || lowerDistance <= activation) {
            return {
                shouldFallback: false,
                keepBodyZVertical,
                reason: 'waist_corridor_requires_waist'
            };
        }
    }

    if (!keepBodyZVertical) {
        return {
            shouldFallback: true,
            keepBodyZVertical: false,
            reason: 'arm_only_retry'
        };
    }

    const bodyNorm = Number.isFinite(currentBodyResidualNorm)
        ? currentBodyResidualNorm
        : asFinite(initialBodyResidualNorm, Number.POSITIVE_INFINITY);
    const tol = Math.max(0, asFinite(bodyTolerance, 5e-5));
    if (bodyNorm <= tol) {
        return {
            shouldFallback: true,
            keepBodyZVertical: false,
            reason: 'body_already_satisfied'
        };
    }

    return {
        shouldFallback: false,
        keepBodyZVertical: true,
        reason: 'body_requires_waist'
    };
}
