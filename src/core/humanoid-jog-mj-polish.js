function asFinite(value, fallback = Number.POSITIVE_INFINITY) {
    return Number.isFinite(value) ? value : fallback;
}

export function shouldAttemptHumanoidJogMJPolish({
    restorationResult = null,
    activationResidual = 1e-4
} = {}) {
    if (!restorationResult || restorationResult.converged === true) return false;
    const residual = asFinite(restorationResult.currentResidualNorm);
    const baseActivationResidual = Math.max(0, asFinite(activationResidual, 1e-4));
    const acceptedIterations = Math.max(0, asFinite(restorationResult.acceptedIterations, 0));
    const bestEffortActivationResidual = Math.max(baseActivationResidual, 5e-4);
    if (restorationResult.reason === 'best_effort' && acceptedIterations > 0) {
        return residual <= bestEffortActivationResidual;
    }
    return residual <= baseActivationResidual;
}

export function shouldApplyHumanoidJogMJPolish({
    restorationResult = null,
    polishResult = null
} = {}) {
    if (!polishResult || typeof polishResult !== 'object') return false;
    if (polishResult.converged === true) return true;
    const before = asFinite(restorationResult?.currentResidualNorm);
    const after = asFinite(polishResult.currentResidualNorm);
    return after + 1e-12 < before;
}
