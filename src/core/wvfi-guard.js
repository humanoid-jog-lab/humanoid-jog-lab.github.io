function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

export function selectWVFILowerBound({
    includeWaist = false,
    lowerBoundWithWaist = 0.03,
    lowerBoundDefault = 0.001
} = {}) {
    return includeWaist
        ? Math.max(0, asFinite(lowerBoundWithWaist, 0.03))
        : Math.max(0, asFinite(lowerBoundDefault, 0.001));
}

export function enforceWVFILowerBound({
    dqCandidate,
    lowerBound = 0,
    evaluateWVFI,
    backtrackScales = [1, 0.5, 0.25, 0.125, 0.0625, 0.03125],
    tolerance = 1e-12
} = {}) {
    const dq = Array.isArray(dqCandidate) ? dqCandidate.map((value) => asFinite(value)) : null;
    const floor = Math.max(0, asFinite(lowerBound, 0));
    const scales = Array.isArray(backtrackScales) && backtrackScales.length > 0
        ? backtrackScales.filter((value) => Number.isFinite(value) && value > 0)
        : [1];

    if (!dq || typeof evaluateWVFI !== 'function') {
        return {
            accepted: false,
            reason: 'invalid_input',
            scale: 0,
            dq: null,
            wvfi: Number.NaN,
            backtracks: 0
        };
    }

    if (!(floor > 0)) {
        return {
            accepted: true,
            reason: 'disabled',
            scale: 1,
            dq,
            wvfi: Number.NaN,
            backtracks: 0
        };
    }

    let lastFiniteWVFI = Number.NaN;
    for (let i = 0; i < scales.length; i++) {
        const scale = scales[i];
        const scaledDQ = dq.map((value) => value * scale);
        const wvfi = evaluateWVFI(scaledDQ);
        if (Number.isFinite(wvfi)) lastFiniteWVFI = wvfi;
        if (Number.isFinite(wvfi) && wvfi + tolerance >= floor) {
            return {
                accepted: true,
                reason: scale === 1 ? 'accepted' : 'scaled',
                scale,
                dq: scaledDQ,
                wvfi,
                backtracks: i
            };
        }
    }

    return {
        accepted: false,
        reason: 'below_lower_bound',
        scale: 0,
        dq: null,
        wvfi: lastFiniteWVFI,
        backtracks: Math.max(0, scales.length - 1)
    };
}
