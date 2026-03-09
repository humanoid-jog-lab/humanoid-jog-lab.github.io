function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function asFiniteNumber(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

export function computeJointLimitPressure(qActive, limits, options = {}) {
    const activationRatio = clamp(asFiniteNumber(options.activationRatio, 0.28), 0.01, 0.95);
    const exponent = Math.max(1, asFiniteNumber(options.exponent, 2));
    const pressure = new Array(Array.isArray(qActive) ? qActive.length : 0).fill(0);
    let norm2 = 0;

    if (!Array.isArray(qActive) || !Array.isArray(limits) || qActive.length === 0) {
        return { pressure, norm: 0, maxAbs: 0 };
    }

    let maxAbs = 0;
    for (let i = 0; i < qActive.length && i < limits.length; i++) {
        const lim = limits[i];
        if (!lim) continue;
        const lower = asFiniteNumber(lim.lower, -Math.PI);
        const upper = asFiniteNumber(lim.upper, Math.PI);
        const range = upper - lower;
        if (!(range > 1e-9)) continue;

        const center = 0.5 * (upper + lower);
        const normalized = clamp((qActive[i] - center) / (0.5 * range), -1, 1);
        const magnitude = Math.abs(normalized);
        if (magnitude <= activationRatio) continue;

        const weight = ((magnitude - activationRatio) / (1 - activationRatio)) ** exponent;
        const value = -Math.sign(normalized) * weight;
        pressure[i] = value;
        norm2 += value * value;
        if (Math.abs(value) > maxAbs) maxAbs = Math.abs(value);
    }

    return {
        pressure,
        norm: Math.sqrt(norm2),
        maxAbs
    };
}

export function computeMinJointMarginRatio(qActive, limits) {
    if (!Array.isArray(qActive) || !Array.isArray(limits) || qActive.length === 0) return Number.POSITIVE_INFINITY;
    let minRatio = Number.POSITIVE_INFINITY;
    for (let i = 0; i < qActive.length && i < limits.length; i++) {
        const lim = limits[i];
        if (!lim) continue;
        const lower = asFiniteNumber(lim.lower, -Math.PI);
        const upper = asFiniteNumber(lim.upper, Math.PI);
        const range = upper - lower;
        if (!(range > 1e-9)) continue;
        const margin = Math.min(qActive[i] - lower, upper - qActive[i]);
        const ratio = clamp(margin / range, 0, 0.5);
        if (ratio < minRatio) minRatio = ratio;
    }
    return minRatio;
}

export function buildTaskEquivalentBranches(qActive, limits, options = {}) {
    const stepSize = Math.max(0, asFiniteNumber(options.stepSize, 0.04));
    const minPressureNorm = Math.max(0, asFiniteNumber(options.minPressureNorm, 1e-6));
    const scalesRaw = Array.isArray(options.scales) && options.scales.length > 0
        ? options.scales
        : [0, 0.5, 1.0, 1.5];
    const scales = scalesRaw
        .map((value) => Math.max(0, asFiniteNumber(value, 0)))
        .filter((value, index, arr) => arr.indexOf(value) === index)
        .sort((a, b) => a - b);

    const realBranch = {
        id: 'real',
        label: 'real',
        scale: 0,
        offset: new Array(Array.isArray(qActive) ? qActive.length : 0).fill(0),
        qSeed: Array.isArray(qActive) ? [...qActive] : [],
        pressureNorm: 0,
        minMarginRatio: computeMinJointMarginRatio(qActive, limits)
    };

    const pressureInfo = computeJointLimitPressure(qActive, limits, options);
    if (!Array.isArray(qActive) || !Array.isArray(limits) || pressureInfo.norm < minPressureNorm) {
        return [realBranch];
    }

    const branches = [realBranch];
    for (const scale of scales) {
        if (scale <= 0) continue;
        const offset = pressureInfo.pressure.map((value) => value * stepSize * scale);
        const qSeed = qActive.map((value, index) => {
            const lim = limits[index];
            if (!lim) return value + offset[index];
            return clamp(
                value + offset[index],
                asFiniteNumber(lim.lower, -Math.PI),
                asFiniteNumber(lim.upper, Math.PI)
            );
        });
        branches.push({
            id: `branch_${String(scale).replace('.', '_')}`,
            label: `phantom-${scale}`,
            scale,
            offset,
            qSeed,
            pressureNorm: pressureInfo.norm,
            minMarginRatio: computeMinJointMarginRatio(qSeed, limits)
        });
    }

    return branches;
}

function compareNumbersAsc(a, b) {
    const aFinite = Number.isFinite(a);
    const bFinite = Number.isFinite(b);
    if (aFinite && bFinite) {
        if (Math.abs(a - b) <= 1e-12) return 0;
        return a < b ? -1 : 1;
    }
    if (aFinite) return -1;
    if (bFinite) return 1;
    return 0;
}

function compareNumbersDesc(a, b) {
    return compareNumbersAsc(b, a);
}

export function rankTaskEquivalentBranchCandidates(candidates, options = {}) {
    const infeasibleSlackPenalty = Math.max(0, asFiniteNumber(options.infeasibleSlackPenalty, 1e3));
    const seedDistanceWeight = Math.max(0, asFiniteNumber(options.seedDistanceWeight, 0.05));
    const continuityWeight = Math.max(0, asFiniteNumber(options.continuityWeight, 0.1));
    const marginWeight = Math.max(0, asFiniteNumber(options.marginWeight, 0.5));

    return (Array.isArray(candidates) ? candidates : [])
        .map((candidate, index) => {
            const qpConverged = candidate?.qpConverged !== false;
            const maxViolationAfter = asFiniteNumber(candidate?.maxViolationAfter, Number.POSITIVE_INFINITY);
            const commandResidualNorm = asFiniteNumber(candidate?.commandResidualNorm, Number.POSITIVE_INFINITY);
            const invariantError = asFiniteNumber(candidate?.invariantError, Number.POSITIVE_INFINITY);
            const minJointMarginRatio = asFiniteNumber(candidate?.minJointMarginRatio, -1);
            const seedDistance = asFiniteNumber(candidate?.seedDistance, Number.POSITIVE_INFINITY);
            const continuityCost = asFiniteNumber(candidate?.continuityCost, 0);
            const slackPenalty = qpConverged ? 0 : infeasibleSlackPenalty;
            const score = slackPenalty
                + maxViolationAfter
                + commandResidualNorm
                + invariantError
                + seedDistanceWeight * seedDistance
                + continuityWeight * continuityCost
                - marginWeight * minJointMarginRatio;

            return {
                ...candidate,
                qpConverged,
                maxViolationAfter,
                commandResidualNorm,
                invariantError,
                minJointMarginRatio,
                seedDistance,
                continuityCost,
                score,
                _stableIndex: index
            };
        })
        .sort((a, b) => {
            const cmpConverged = (a.qpConverged === b.qpConverged) ? 0 : (a.qpConverged ? -1 : 1);
            if (cmpConverged !== 0) return cmpConverged;

            const cmpViolation = compareNumbersAsc(a.maxViolationAfter, b.maxViolationAfter);
            if (cmpViolation !== 0) return cmpViolation;

            const cmpResidual = compareNumbersAsc(a.commandResidualNorm, b.commandResidualNorm);
            if (cmpResidual !== 0) return cmpResidual;

            const cmpInvariant = compareNumbersAsc(a.invariantError, b.invariantError);
            if (cmpInvariant !== 0) return cmpInvariant;

            const cmpMargin = compareNumbersDesc(a.minJointMarginRatio, b.minJointMarginRatio);
            if (cmpMargin !== 0) return cmpMargin;

            const cmpSeedDistance = compareNumbersAsc(a.seedDistance, b.seedDistance);
            if (cmpSeedDistance !== 0) return cmpSeedDistance;

            const cmpScore = compareNumbersAsc(a.score, b.score);
            if (cmpScore !== 0) return cmpScore;

            return a._stableIndex - b._stableIndex;
        })
        .map(({ _stableIndex, ...candidate }) => candidate);
}

export function selectBestTaskEquivalentBranchCandidate(candidates, options = {}) {
    const ranked = rankTaskEquivalentBranchCandidates(candidates, options);
    return ranked.length > 0 ? ranked[0] : null;
}

function getCandidateBranchId(candidate) {
    if (typeof candidate?.branchId === 'string' && candidate.branchId.length > 0) return candidate.branchId;
    if (typeof candidate?.id === 'string' && candidate.id.length > 0) return candidate.id;
    return null;
}

export function applyTaskEquivalentBranchHysteresis(candidates, previousState = null, options = {}) {
    const rankedCandidates = rankTaskEquivalentBranchCandidates(candidates, options);
    const best = rankedCandidates.length > 0 ? rankedCandidates[0] : null;
    if (!best) {
        return {
            rankedCandidates,
            selected: null,
            state: null,
            switched: false,
            keptByHysteresis: false
        };
    }

    const hysteresisAbsMargin = Math.max(0, asFiniteNumber(options.hysteresisAbsMargin, 5e-4));
    const hysteresisRelMargin = Math.max(0, asFiniteNumber(options.hysteresisRelMargin, 0.2));
    const minHoldSteps = Math.max(0, Math.floor(asFiniteNumber(options.minHoldSteps, 2)));
    const previousBranchId = getCandidateBranchId(previousState);
    const previousHoldStepsRemaining = Math.max(0, Math.floor(asFiniteNumber(previousState?.holdStepsRemaining, 0)));
    const previousCandidate = previousBranchId
        ? rankedCandidates.find((candidate) => getCandidateBranchId(candidate) === previousBranchId) || null
        : null;

    let selected = best;
    let keptByHysteresis = false;
    if (previousCandidate && previousCandidate.qpConverged) {
        const scoreScale = Math.max(hysteresisAbsMargin, Math.abs(best.score));
        const scoreWindow = hysteresisAbsMargin + hysteresisRelMargin * scoreScale;
        const previousWithinWindow = previousCandidate.score <= best.score + scoreWindow;
        if (previousWithinWindow) {
            selected = previousCandidate;
            keptByHysteresis = getCandidateBranchId(previousCandidate) !== getCandidateBranchId(best);
        }
    }

    const selectedBranchId = getCandidateBranchId(selected);
    const switched = Boolean(
        previousBranchId &&
        selectedBranchId &&
        previousBranchId !== selectedBranchId
    );
    const holdStepsRemaining = switched
        ? minHoldSteps
        : Math.max(0, previousHoldStepsRemaining - 1);

    return {
        rankedCandidates,
        selected,
        state: selectedBranchId
            ? {
                branchId: selectedBranchId,
                holdStepsRemaining
            }
            : null,
        switched,
        keptByHysteresis
    };
}

function formatCompactNumber(value) {
    if (!Number.isFinite(value)) return 'n/a';
    const abs = Math.abs(value);
    if (abs === 0) return '0';
    if (abs >= 1000 || abs < 1e-3) return value.toExponential(2);
    return value.toFixed(3);
}

export function summarizeTaskEquivalentBranchCandidates(candidates, options = {}) {
    const maxLines = Math.max(1, Math.floor(asFiniteNumber(options.maxLines, 4)));
    const selectedBranchId = getCandidateBranchId(options.selectedCandidate) || options.selectedBranchId || null;
    const heldBranchId = options.heldBranchId || null;
    const rankedCandidates = rankTaskEquivalentBranchCandidates(candidates, options);
    const lines = [];

    for (let i = 0; i < rankedCandidates.length && i < maxLines; i++) {
        const candidate = rankedCandidates[i];
        const branchId = getCandidateBranchId(candidate) || `branch_${i + 1}`;
        const markers = [
            branchId === selectedBranchId ? '*' : '-',
            branchId === heldBranchId ? 'H' : ' '
        ].join('');
        lines.push(
            `${markers}${i + 1} ${candidate.branchLabel || branchId}` +
            ` score=${formatCompactNumber(candidate.score)}` +
            ` qp=${candidate.qpConverged ? 'yes' : 'no'}` +
            ` res=${formatCompactNumber(candidate.commandResidualNorm)}` +
            ` inv=${formatCompactNumber(candidate.invariantError)}` +
            ` vio=${formatCompactNumber(candidate.maxViolationAfter)}` +
            ` margin=${formatCompactNumber(candidate.minJointMarginRatio)}` +
            ` seed=${formatCompactNumber(candidate.seedDistance)}` +
            ` cont=${formatCompactNumber(candidate.continuityCost)}`
        );
    }

    return lines;
}
