function asFinite(value, fallback = Number.POSITIVE_INFINITY) {
    return Number.isFinite(value) ? value : fallback;
}

function vectorNorm(values) {
    const src = Array.isArray(values) ? values : [];
    let sum = 0;
    for (const value of src) sum += (Number(value) || 0) * (Number(value) || 0);
    return Math.sqrt(sum);
}

export function evaluateHumanoidJogManifoldAcceptance({
    restorationConverged = false,
    restorationAccepted = restorationConverged,
    absoluteResidual = [],
    relativeResidual = [],
    bodyResidual = [],
    inequalityViolation = 0,
    commandTolerance = 1e-12,
    relativeTolerance = 1e-12,
    practicalCommandTolerance = Number.NaN,
    practicalRelativeTolerance = Number.NaN,
    bodyTolerance = 5e-5,
    inequalityTolerance = 1e-12
} = {}) {
    const absoluteNorm = vectorNorm(absoluteResidual);
    const relativeNorm = vectorNorm(relativeResidual);
    const bodyNorm = vectorNorm(bodyResidual);
    const maxIneqViolation = asFinite(inequalityViolation, Number.POSITIVE_INFINITY);
    const hardSatisfied = (
        restorationAccepted === true &&
        absoluteNorm <= Math.max(0, asFinite(commandTolerance, 1e-12)) &&
        relativeNorm <= Math.max(0, asFinite(relativeTolerance, 1e-12)) &&
        bodyNorm <= Math.max(0, asFinite(bodyTolerance, 5e-5)) &&
        maxIneqViolation <= Math.max(0, asFinite(inequalityTolerance, 1e-12))
    );
    const practicalSatisfied = (
        restorationAccepted === true &&
        Number.isFinite(practicalCommandTolerance) &&
        absoluteNorm <= Math.max(0, asFinite(practicalCommandTolerance, Number.NaN)) &&
        (
            !Number.isFinite(practicalRelativeTolerance) ||
            relativeNorm <= Math.max(0, asFinite(practicalRelativeTolerance, Number.NaN))
        ) &&
        bodyNorm <= Math.max(0, asFinite(bodyTolerance, 5e-5)) &&
        maxIneqViolation <= Math.max(0, asFinite(inequalityTolerance, 1e-12))
    );
    const converged = hardSatisfied || practicalSatisfied;
    const reason = hardSatisfied
        ? 'converged'
        : (practicalSatisfied ? 'practical_converged' : 'hard_tol_not_reached');

    return {
        converged,
        reason,
        hardSatisfied,
        practicalSatisfied,
        absoluteNorm,
        relativeNorm,
        bodyNorm,
        inequalityViolation: maxIneqViolation
    };
}
