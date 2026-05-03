function asFiniteNumber(value, fallback = Number.NaN) {
    return Number.isFinite(value) ? value : fallback;
}

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

export function shouldRejectHumanoidJogForMismatch(options = {}) {
    const reason = typeof options.stopReason === 'string' && options.stopReason.length > 0
        ? options.stopReason
        : 'max_iter';
    const finalCommandNorm = asFiniteNumber(options.finalCommandNorm, Number.NaN);
    const initialCommandNorm = asFiniteNumber(options.initialCommandNorm, Number.NaN);
    const mismatchTol = Math.max(0, asFiniteNumber(options.mismatchTol, 1e-3));
    const mismatchRequiredReduction = clamp(asFiniteNumber(options.mismatchRequiredReduction, 0.8), 0, 0.99);
    const singleStepMismatchRequiredReduction = clamp(
        asFiniteNumber(options.singleStepMismatchRequiredReduction, 0.05),
        0,
        0.99
    );
    const explicitFailureReasons = options.explicitFailureReasons instanceof Set
        ? options.explicitFailureReasons
        : new Set(['fk_fail', 'jacobian_fail', 'body_task_fail', 'solver_fail']);
    const requiredReduction = reason === 'single_step'
        ? singleStepMismatchRequiredReduction
        : mismatchRequiredReduction;

    const reductionSatisfied = (
        Number.isFinite(initialCommandNorm) &&
        initialCommandNorm > 0 &&
        Number.isFinite(finalCommandNorm) &&
        finalCommandNorm <= initialCommandNorm * (1 - requiredReduction)
    );
    const shouldReject = (
        !explicitFailureReasons.has(reason) &&
        reason !== 'converged' &&
        reason !== 'practical_converged' &&
        reason !== 'best_effort' &&
        Number.isFinite(finalCommandNorm) &&
        finalCommandNorm > mismatchTol &&
        !reductionSatisfied
    );

    return {
        shouldReject,
        reductionSatisfied
    };
}

export function evaluateHumanoidJogCommandOutcome(options = {}) {
    let reason = typeof options.stopReason === 'string' && options.stopReason.length > 0
        ? options.stopReason
        : 'max_iter';
    const toleranceUpgradeableReasons = new Set([
        'max_iter',
        'best_effort',
        'best_effort_time_budget',
        'safety_limited',
        'secondary_limited'
    ]);

    if (reason === 'polish_converged') {
        reason = 'converged';
    } else if (reason === 'polish_best_effort') {
        reason = 'best_effort';
    }

    const finalCommandNorm = asFiniteNumber(options.finalCommandNorm, Number.NaN);
    const initialCommandNorm = asFiniteNumber(options.initialCommandNorm, Number.NaN);
    const finalInvariantNorm = asFiniteNumber(options.finalInvariantNorm, Number.NaN);
    const hardCommandTol = Math.max(0, asFiniteNumber(options.hardCommandTol, 1e-6));
    const hardInvariantTol = Math.max(0, asFiniteNumber(options.hardInvariantTol, Number.POSITIVE_INFINITY));
    const practicalCommandTol = Math.max(0, asFiniteNumber(options.practicalCommandTol, Number.NaN));
    const practicalInvariantTol = Math.max(0, asFiniteNumber(options.practicalInvariantTol, Number.NaN));
    const mismatchTol = Math.max(0, asFiniteNumber(options.mismatchTol, 1e-3));
    const mismatchRequiredReduction = clamp(asFiniteNumber(options.mismatchRequiredReduction, 0.8), 0, 0.99);
    const singleStepMismatchRequiredReduction = clamp(
        asFiniteNumber(options.singleStepMismatchRequiredReduction, 0.05),
        0,
        0.99
    );
    const requireHardConvergenceToApply = options.requireHardConvergenceToApply === true;

    const explicitFailureReasons = new Set(['fk_fail', 'jacobian_fail', 'body_task_fail', 'solver_fail']);
    const hardSatisfied = (
        Number.isFinite(finalCommandNorm) &&
        finalCommandNorm <= hardCommandTol &&
        (
            !Number.isFinite(hardInvariantTol) ||
            (Number.isFinite(finalInvariantNorm) && finalInvariantNorm <= hardInvariantTol)
        )
    );
    if (toleranceUpgradeableReasons.has(reason) && hardSatisfied) {
        reason = 'converged';
    }

    const practicalSatisfied = (
        reason !== 'converged' &&
        Number.isFinite(finalCommandNorm) &&
        Number.isFinite(practicalCommandTol) &&
        finalCommandNorm <= practicalCommandTol &&
        (
            !Number.isFinite(practicalInvariantTol) ||
            (Number.isFinite(finalInvariantNorm) && finalInvariantNorm <= practicalInvariantTol)
        )
    );
    if (toleranceUpgradeableReasons.has(reason) && practicalSatisfied) {
        reason = 'practical_converged';
    }

    const { shouldReject: shouldRejectForMismatch, reductionSatisfied } = shouldRejectHumanoidJogForMismatch({
        stopReason: reason,
        finalCommandNorm,
        initialCommandNorm,
        mismatchTol,
        mismatchRequiredReduction,
        singleStepMismatchRequiredReduction,
        explicitFailureReasons
    });
    if (shouldRejectForMismatch) {
        reason = 'command_mismatch';
    }

    if (
        requireHardConvergenceToApply &&
        !explicitFailureReasons.has(reason) &&
        reason !== 'converged' &&
        !hardSatisfied
    ) {
        reason = 'command_mismatch';
    }

    const converged = reason === 'converged' || reason === 'practical_converged' || reason === 'single_step';
    const applyResult = !explicitFailureReasons.has(reason) && reason !== 'command_mismatch';

    return {
        reason,
        converged,
        applyResult,
        hardSatisfied,
        practicalSatisfied: reason === 'practical_converged' || practicalSatisfied,
        reductionSatisfied
    };
}

export function resolveHumanoidJogCommandResult(options = {}) {
    const outcome = evaluateHumanoidJogCommandOutcome(options);
    const q = Array.isArray(options.q) ? options.q : [];
    const qInitial = Array.isArray(options.qInitial) ? options.qInitial : [];
    const messageMap = options.messageMap && typeof options.messageMap === 'object' ? options.messageMap : {};

    return {
        outcome,
        qApplied: outcome.applyResult ? q : qInitial,
        message: messageMap[outcome.reason] || options.defaultMessage || '已执行。'
    };
}
