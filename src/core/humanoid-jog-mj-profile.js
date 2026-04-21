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
        maxDampingRetries: asPositiveInteger(config?.mjRestoreMaxDampingRetries, 8),
        dampingGrowthFactor: asPositive(config?.mjRestoreDampingGrowthFactor, 10),
        maxTrustRegionRetries: asPositiveInteger(config?.mjRestoreMaxTrustRegionRetries, 6),
        rhoAcceptanceThreshold: asNonNegative(config?.mjRestoreRhoAcceptanceThreshold, 0.05),
        rhoShrinkThreshold: asNonNegative(config?.mjRestoreRhoShrinkThreshold, 0.05),
        polishMaxIterations: asPositiveInteger(config?.mjRestorePolishMaxIter, 64),
        polishActivationResidual: asNonNegative(config?.mjRestorePolishActivationResidual, 1e-4),
        polishDamping: asPositive(config?.mjRestorePolishDamping, 1e-6),
        tag: 'default'
    };

    if (
        controlMode === 'cooperative-ruled-surface-polishing-dq-setpoint'
        || controlMode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only'
    ) {
        return {
            ...baseProfile,
            maxIterations: asPositiveInteger(
                config?.mjRestoreCooperativeSurfacePolishingDQSetpointMaxIter,
                80
            ),
            damping: Math.max(
                baseProfile.damping,
                asPositive(config?.mjRestoreCooperativeSurfacePolishingDQSetpointDamping, 1e-4)
            ),
            referenceDamping: Math.min(
                baseProfile.referenceDamping,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingDQSetpointReferenceDamping, 0.15)
            ),
            maxDampingRetries: Math.max(
                baseProfile.maxDampingRetries,
                asPositiveInteger(config?.mjRestoreCooperativeSurfacePolishingDQSetpointMaxDampingRetries, 10)
            ),
            dampingGrowthFactor: Math.min(
                baseProfile.dampingGrowthFactor,
                asPositive(config?.mjRestoreCooperativeSurfacePolishingDQSetpointDampingGrowthFactor, 4)
            ),
            maxTrustRegionRetries: Math.max(
                baseProfile.maxTrustRegionRetries,
                asPositiveInteger(config?.mjRestoreCooperativeSurfacePolishingDQSetpointMaxTrustRegionRetries, 10)
            ),
            rhoAcceptanceThreshold: Math.min(
                baseProfile.rhoAcceptanceThreshold,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingDQSetpointRhoAcceptanceThreshold, 0.02)
            ),
            rhoShrinkThreshold: Math.min(
                baseProfile.rhoShrinkThreshold,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingDQSetpointRhoShrinkThreshold, 0.02)
            ),
            polishActivationResidual: Math.max(
                baseProfile.polishActivationResidual,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingDQSetpointPolishActivationResidual, 5e-4)
            ),
            polishMaxIterations: Math.max(
                baseProfile.polishMaxIterations,
                asPositiveInteger(config?.mjRestoreCooperativeSurfacePolishingDQSetpointPolishMaxIter, 64)
            ),
            polishDamping: Math.min(
                baseProfile.polishDamping,
                asPositive(config?.mjRestoreCooperativeSurfacePolishingDQSetpointPolishDamping, 1e-7)
            ),
            tag: controlMode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only'
                ? 'cooperative_surface_polishing_dq_setpoint_arm_only'
                : 'cooperative_surface_polishing_dq_setpoint'
        };
    }

    if (
        controlMode === 'cooperative-ruled-surface-polishing'
        || controlMode === 'left-ruled-surface-polishing'
    ) {
        return {
            ...baseProfile,
            maxIterations: asPositiveInteger(
                config?.mjRestoreCooperativeSurfacePolishingMaxIter,
                160
            ),
            damping: Math.max(
                baseProfile.damping,
                asPositive(config?.mjRestoreCooperativeSurfacePolishingDamping, 1e-4)
            ),
            referenceDamping: Math.min(
                baseProfile.referenceDamping,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingReferenceDamping, 0.15)
            ),
            maxDampingRetries: Math.max(
                baseProfile.maxDampingRetries,
                asPositiveInteger(config?.mjRestoreCooperativeSurfacePolishingMaxDampingRetries, 12)
            ),
            dampingGrowthFactor: Math.min(
                baseProfile.dampingGrowthFactor,
                asPositive(config?.mjRestoreCooperativeSurfacePolishingDampingGrowthFactor, 4)
            ),
            maxTrustRegionRetries: Math.max(
                baseProfile.maxTrustRegionRetries,
                asPositiveInteger(config?.mjRestoreCooperativeSurfacePolishingMaxTrustRegionRetries, 12)
            ),
            rhoAcceptanceThreshold: Math.min(
                baseProfile.rhoAcceptanceThreshold,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingRhoAcceptanceThreshold, 0.02)
            ),
            rhoShrinkThreshold: Math.min(
                baseProfile.rhoShrinkThreshold,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingRhoShrinkThreshold, 0.02)
            ),
            polishActivationResidual: Math.max(
                baseProfile.polishActivationResidual,
                asNonNegative(config?.mjRestoreCooperativeSurfacePolishingPolishActivationResidual, 1)
            ),
            polishMaxIterations: Math.max(
                baseProfile.polishMaxIterations,
                asPositiveInteger(config?.mjRestoreCooperativeSurfacePolishingPolishMaxIter, 240)
            ),
            polishDamping: Math.min(
                baseProfile.polishDamping,
                asPositive(config?.mjRestoreCooperativeSurfacePolishingPolishDamping, 1e-8)
            ),
            tag: 'cooperative_surface_polishing'
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
