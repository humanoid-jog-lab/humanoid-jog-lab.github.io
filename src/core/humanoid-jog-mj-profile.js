function asFinite(value, fallback) {
    return Number.isFinite(value) ? value : fallback;
}

function asPositiveInteger(value, fallback) {
    return Math.max(1, Math.floor(asFinite(value, fallback)));
}

function asNonNegative(value, fallback) {
    return Math.max(0, asFinite(value, fallback));
}

function asPositive(value, fallback) {
    return Math.max(1e-12, asFinite(value, fallback));
}

export function resolveHumanoidJogMJRestoreProfile({
    includeWaist = false,
    fallbackReason = '',
    config = {},
    controlMode = ''
} = {}) {
    const baseProfile = {
        maxIterations: asPositiveInteger(config?.mjRestoreMaxIter, 120),
        damping: asPositive(config?.mjRestoreDamping, 1e-4),
        referenceStiffness: asNonNegative(config?.mjRestoreReferenceStiffness, 1.0),
        referenceDamping: asNonNegative(config?.mjRestoreReferenceDamping, 0.35),
        polishMaxIterations: asPositiveInteger(config?.mjRestorePolishMaxIter, 64),
        polishActivationResidual: asNonNegative(config?.mjRestorePolishActivationResidual, 1e-4),
        polishDamping: asPositive(config?.mjRestorePolishDamping, 1e-6),
        tag: 'default'
    };

    if (controlMode === 'cooperative-ruled-surface-polishing') {
        return {
            ...baseProfile,
            maxIterations: asPositiveInteger(
                config?.mjRestoreCooperativeSurfacePolishingMaxIter,
                baseProfile.maxIterations
            ),
            damping: Math.max(
                baseProfile.damping,
                asPositive(config?.mjRestoreCooperativeSurfacePolishingDamping, 1e-4)
            ),
            referenceDamping: Math.min(
                baseProfile.referenceDamping,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingReferenceDamping, 0.3)
            ),
            tag: 'cooperative_surface_polishing'
        };
    }

    const surfaceSlideMode = controlMode === 'surface-slide'
        || controlMode === 'dual-independent-surface-slide';
    if (surfaceSlideMode) {
        return {
            ...baseProfile,
            maxIterations: Math.max(
                baseProfile.maxIterations,
                asPositiveInteger(config?.mjRestoreSurfaceSlideMaxIter, 160)
            ),
            damping: Math.min(
                baseProfile.damping,
                asPositive(config?.mjRestoreSurfaceSlideDamping, 5e-5)
            ),
            referenceDamping: Math.min(
                baseProfile.referenceDamping,
                asNonNegative(config?.mjRestoreSurfaceSlideReferenceDamping, 0.2)
            ),
            tag: 'surface_slide'
        };
    }

    const armOnlyFallback =
        includeWaist !== true &&
        typeof fallbackReason === 'string' &&
        fallbackReason.trim().length > 0;

    if (!armOnlyFallback) {
        return baseProfile;
    }

    return {
        ...baseProfile,
        maxIterations: Math.min(
            baseProfile.maxIterations,
            asPositiveInteger(config?.mjRestoreArmOnlyFallbackMaxIter, 12)
        ),
        referenceDamping: Math.min(
            baseProfile.referenceDamping,
            asNonNegative(config?.mjRestoreArmOnlyFallbackReferenceDamping, 0.2)
        ),
        polishActivationResidual: Math.max(
            baseProfile.polishActivationResidual,
            asNonNegative(config?.mjRestoreArmOnlyFallbackPolishActivationResidual, 5e-4)
        ),
        polishDamping: asPositive(
            config?.mjRestoreArmOnlyFallbackPolishDamping,
            1e-7
        ),
        tag: 'arm_only_fallback'
    };
}
