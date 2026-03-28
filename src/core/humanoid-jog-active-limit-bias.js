function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function vectorNorm(values) {
    return Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => {
        const finiteValue = asFinite(value);
        return sum + finiteValue * finiteValue;
    }, 0));
}

function normalizeVector(values) {
    const source = Array.isArray(values) ? values : [];
    const norm = vectorNorm(source);
    if (!(norm > 1e-10)) return new Array(source.length).fill(0);
    return source.map((value) => asFinite(value) / norm);
}

export function evaluateHumanoidJogActiveLimitSeverity(constraints = []) {
    let minDistance = Number.NaN;
    let threshold = Number.NaN;
    let severity = 0;

    for (const constraint of Array.isArray(constraints) ? constraints : []) {
        if (constraint?.source !== 'joint_limit') continue;
        const distance = asFinite(constraint?.distance, Number.NaN);
        const limitThreshold = asFinite(constraint?.threshold, Number.NaN);
        if (!Number.isFinite(distance) || !Number.isFinite(limitThreshold) || !(limitThreshold > 0)) continue;
        const nextSeverity = Math.max(0, Math.min(1, 1 - (distance / limitThreshold)));
        if (!(nextSeverity > severity)) continue;
        severity = nextSeverity;
        minDistance = distance;
        threshold = limitThreshold;
    }

    return {
        active: severity > 0,
        severity,
        minDistance,
        threshold
    };
}

export function buildHumanoidJogActiveLimitRetreatGradient({
    objectiveGradient = [],
    jointLimitGradient = [],
    wvfiGradient = [],
    constraints = [],
    maxBias = 0.95,
    maxStepScale = 32.0,
    stepScaleExponent = 2.0
} = {}) {
    const objective = normalizeVector(objectiveGradient);
    const jointLimit = normalizeVector(jointLimitGradient);
    const wvfi = normalizeVector(wvfiGradient);
    const width = Math.max(objective.length, jointLimit.length, wvfi.length);
    const severityInfo = evaluateHumanoidJogActiveLimitSeverity(constraints);
    const clampedMaxBias = Math.max(0, Math.min(1, asFinite(maxBias, 0.95)));
    const clampedMaxStepScale = Math.max(1, asFinite(maxStepScale, 16.0));
    const clampedStepScaleExponent = Math.max(1, asFinite(stepScaleExponent, 2.0));

    const retreatRaw = new Array(width).fill(0);
    for (let i = 0; i < width; i += 1) {
        retreatRaw[i] = asFinite(jointLimit[i]) + asFinite(wvfi[i]);
    }
    const retreat = normalizeVector(retreatRaw);

    if (!(severityInfo.severity > 0) || vectorNorm(retreat) <= 1e-10) {
        return {
            gradient: objective.slice(),
            bias: 0,
            stepScale: 1,
            ...severityInfo
        };
    }

    if (vectorNorm(objective) <= 1e-10) {
        return {
            gradient: retreat.slice(),
            bias: clampedMaxBias,
            stepScale: clampedMaxStepScale,
            ...severityInfo
        };
    }

    const bias = Math.min(clampedMaxBias, severityInfo.severity * clampedMaxBias);
    const stepScale = 1 + ((clampedMaxStepScale - 1) * Math.pow(severityInfo.severity, clampedStepScaleExponent));
    const blendedRaw = new Array(width).fill(0);
    for (let i = 0; i < width; i += 1) {
        blendedRaw[i] = (1 - bias) * asFinite(objective[i]) + bias * asFinite(retreat[i]);
    }

    return {
        gradient: normalizeVector(blendedRaw),
        bias,
        stepScale,
        ...severityInfo
    };
}

export function buildHumanoidJogActiveLimitRetreatBounds({
    qCurrent = [],
    step = [],
    activeLimits = [],
    constraints = [],
    holdFraction = 0.5,
    minRetreatDistance = 1e-3,
    targetThresholdFraction = 0.4,
    maxRetreatDistance = Number.POSITIVE_INFINITY,
    stepTolerance = 1e-9
} = {}) {
    const width = Math.max(
        Array.isArray(qCurrent) ? qCurrent.length : 0,
        Array.isArray(step) ? step.length : 0,
        Array.isArray(activeLimits) ? activeLimits.length : 0
    );
    const lowerBounds = new Array(width).fill(Number.NEGATIVE_INFINITY);
    const upperBounds = new Array(width).fill(Number.POSITIVE_INFINITY);
    const activeIndices = new Set();

    for (const constraint of Array.isArray(constraints) ? constraints : []) {
        if (constraint?.source !== 'joint_limit' || !Array.isArray(constraint?.row)) continue;
        let bestIndex = -1;
        let bestMagnitude = 0;
        for (let i = 0; i < Math.min(width, constraint.row.length); i += 1) {
            const magnitude = Math.abs(asFinite(constraint.row[i]));
            if (magnitude > bestMagnitude) {
                bestMagnitude = magnitude;
                bestIndex = i;
            }
        }
        if (bestIndex >= 0 && bestMagnitude > 1e-6) {
            activeIndices.add(bestIndex);
        }
    }

    const clampedHoldFraction = Math.max(0, asFinite(holdFraction, 0.5));
    const clampedMinRetreatDistance = Math.max(0, asFinite(minRetreatDistance, 1e-3));
    const clampedTargetThresholdFraction = Math.max(0, asFinite(targetThresholdFraction, 0.2));
    const clampedMaxRetreatDistance = Number.isFinite(maxRetreatDistance)
        ? Math.max(clampedMinRetreatDistance, asFinite(maxRetreatDistance, Number.POSITIVE_INFINITY))
        : Number.POSITIVE_INFINITY;
    const lockedIndices = [];
    const targetRetreatDistances = new Array(width).fill(clampedMinRetreatDistance);

    for (const constraint of Array.isArray(constraints) ? constraints : []) {
        if (constraint?.source !== 'joint_limit' || !Array.isArray(constraint?.row)) continue;
        let bestIndex = -1;
        let bestMagnitude = 0;
        for (let i = 0; i < Math.min(width, constraint.row.length); i += 1) {
            const magnitude = Math.abs(asFinite(constraint.row[i]));
            if (magnitude > bestMagnitude) {
                bestMagnitude = magnitude;
                bestIndex = i;
            }
        }
        if (bestIndex < 0 || !(bestMagnitude > 1e-6)) continue;
        const distance = asFinite(constraint?.distance, Number.NaN);
        const threshold = asFinite(constraint?.threshold, Number.NaN);
        const severity = Number.isFinite(distance) && Number.isFinite(threshold) && threshold > 0
            ? Math.max(0, Math.min(1, 1 - (distance / threshold)))
            : 0;
        const targetDistance = Number.isFinite(threshold) && threshold > 0
            ? Math.max(
                clampedMinRetreatDistance,
                threshold * clampedTargetThresholdFraction * severity
            )
            : clampedMinRetreatDistance;
        targetRetreatDistances[bestIndex] = Math.max(
            targetRetreatDistances[bestIndex],
            Math.min(clampedMaxRetreatDistance, targetDistance)
        );
    }

    for (let i = 0; i < width; i += 1) {
        if (activeIndices.size > 0 && !activeIndices.has(i)) continue;
        const limit = activeLimits?.[i];
        if (!limit) continue;

        const qi = asFinite(qCurrent?.[i], Number.NaN);
        const dq = asFinite(step?.[i], 0);
        const lower = Number.isFinite(limit?.lower) ? limit.lower : Number.NEGATIVE_INFINITY;
        const upper = Number.isFinite(limit?.upper) ? limit.upper : Number.POSITIVE_INFINITY;
        if (!Number.isFinite(qi)) continue;

        const targetRetreatDistance = Math.max(
            clampedMinRetreatDistance,
            asFinite(targetRetreatDistances[i], clampedMinRetreatDistance)
        );
        const distanceToLower = Number.isFinite(lower) ? qi - lower : Number.POSITIVE_INFINITY;
        const distanceToUpper = Number.isFinite(upper) ? upper - qi : Number.POSITIVE_INFINITY;
        const retreatsFromLower = distanceToLower <= distanceToUpper && dq > stepTolerance;
        const retreatsFromUpper = distanceToUpper < distanceToLower && dq < -stepTolerance;
        const forceRetreatFromLower = distanceToLower <= distanceToUpper && distanceToLower <= targetRetreatDistance + stepTolerance;
        const forceRetreatFromUpper = distanceToUpper < distanceToLower && distanceToUpper <= targetRetreatDistance + stepTolerance;
        if (!retreatsFromLower && !retreatsFromUpper && !forceRetreatFromLower && !forceRetreatFromUpper) continue;

        const retreatMagnitude = Math.abs(dq);
        const retainedMagnitude = Math.max(
            targetRetreatDistance,
            (retreatsFromLower || retreatsFromUpper)
                ? clampedHoldFraction * retreatMagnitude
                : 0
        );
        const cappedRetainedMagnitude = Math.min(clampedMaxRetreatDistance, retainedMagnitude);
        if (!(cappedRetainedMagnitude > stepTolerance)) continue;

        if (retreatsFromLower || forceRetreatFromLower) {
            const lockedLower = Math.min(upper, qi + cappedRetainedMagnitude);
            if (lockedLower > lower + stepTolerance) {
                lowerBounds[i] = lockedLower;
                upperBounds[i] = upper;
                lockedIndices.push(i);
            }
            continue;
        }

        const lockedUpper = Math.max(lower, qi - cappedRetainedMagnitude);
        if (lockedUpper < upper - stepTolerance) {
            lowerBounds[i] = lower;
            upperBounds[i] = lockedUpper;
            lockedIndices.push(i);
        }
    }

    return {
        active: lockedIndices.length > 0,
        lowerBounds,
        upperBounds,
        lockedIndices
    };
}
