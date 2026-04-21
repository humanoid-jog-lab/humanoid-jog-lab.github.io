import test from 'node:test';
import assert from 'node:assert/strict';

import { resolveHumanoidJogMJRestoreProfile } from './humanoid-jog-mj-profile.js';

test('resolveHumanoidJogMJRestoreProfile keeps the default mj_restore profile for normal full-body solves', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: true,
        fallbackReason: '',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreReferenceDamping: 0.35,
            mjRestorePolishActivationResidual: 1e-4,
            mjRestorePolishDamping: 1e-6
        }
    });

    assert.equal(profile.maxIterations, 120);
    assert.equal(profile.referenceDamping, 0.35);
    assert.equal(profile.polishActivationResidual, 1e-4);
    assert.equal(profile.polishDamping, 1e-6);
    assert.equal(profile.tag, 'default');
});

test('resolveHumanoidJogMJRestoreProfile switches to a faster arm-only fallback profile', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: false,
        fallbackReason: 'body_already_satisfied',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreReferenceDamping: 0.35,
            mjRestorePolishActivationResidual: 1e-4,
            mjRestorePolishDamping: 1e-6
        }
    });

    assert.equal(profile.maxIterations, 12);
    assert.equal(profile.referenceDamping, 0.2);
    assert.equal(profile.polishActivationResidual, 5e-4);
    assert.equal(profile.polishDamping, 1e-7);
    assert.equal(profile.tag, 'arm_only_fallback');
});

test('resolveHumanoidJogMJRestoreProfile uses a dedicated steadier profile for cooperative ruled-surface polishing', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: true,
        fallbackReason: '',
        controlMode: 'cooperative-ruled-surface-polishing',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreDamping: 1e-4,
            mjRestoreReferenceDamping: 0.35,
            mjRestorePolishMaxIter: 64,
            mjRestorePolishDamping: 1e-6
        }
    });

    assert.equal(profile.maxIterations, 160);
    assert.equal(profile.damping, 1e-4);
    assert.equal(profile.referenceDamping, 0.15);
    assert.equal(profile.maxDampingRetries, 12);
    assert.equal(profile.dampingGrowthFactor, 4);
    assert.equal(profile.maxTrustRegionRetries, 12);
    assert.equal(profile.rhoAcceptanceThreshold, 0.02);
    assert.equal(profile.rhoShrinkThreshold, 0.02);
    assert.equal(profile.polishActivationResidual, 1);
    assert.equal(profile.polishMaxIterations, 240);
    assert.equal(profile.polishDamping, 1e-8);
    assert.equal(profile.tag, 'cooperative_surface_polishing');
});

test('resolveHumanoidJogMJRestoreProfile reuses the DQ set-point profile for the arm-only cooperative mode', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: false,
        fallbackReason: '',
        controlMode: 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only',
        config: {}
    });

    assert.equal(profile.maxIterations, 80);
    assert.equal(profile.maxDampingRetries, 10);
    assert.equal(profile.maxTrustRegionRetries, 10);
    assert.equal(profile.polishActivationResidual, 5e-4);
    assert.equal(profile.polishMaxIterations, 64);
    assert.equal(profile.tag, 'cooperative_surface_polishing_dq_setpoint_arm_only');
});

test('resolveHumanoidJogMJRestoreProfile reuses the cooperative polishing profile for left ruled-surface polishing', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: false,
        fallbackReason: '',
        controlMode: 'left-ruled-surface-polishing',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreDamping: 1e-4,
            mjRestoreReferenceDamping: 0.35,
            mjRestorePolishMaxIter: 64,
            mjRestorePolishDamping: 1e-6
        }
    });

    assert.equal(profile.maxIterations, 160);
    assert.equal(profile.damping, 1e-4);
    assert.equal(profile.referenceDamping, 0.15);
    assert.equal(profile.maxDampingRetries, 12);
    assert.equal(profile.dampingGrowthFactor, 4);
    assert.equal(profile.maxTrustRegionRetries, 12);
    assert.equal(profile.rhoAcceptanceThreshold, 0.02);
    assert.equal(profile.rhoShrinkThreshold, 0.02);
    assert.equal(profile.polishActivationResidual, 1);
    assert.equal(profile.polishMaxIterations, 240);
    assert.equal(profile.polishDamping, 1e-8);
    assert.equal(profile.tag, 'cooperative_surface_polishing');
});

test('resolveHumanoidJogMJRestoreProfile uses a lighter dedicated profile for cooperative ruled-surface polishing DQ set-point mode', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: true,
        fallbackReason: '',
        controlMode: 'cooperative-ruled-surface-polishing-dq-setpoint',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreDamping: 1e-4,
            mjRestoreReferenceDamping: 0.35,
            mjRestorePolishMaxIter: 64,
            mjRestorePolishDamping: 1e-6
        }
    });

    assert.equal(profile.maxIterations, 80);
    assert.equal(profile.damping, 1e-4);
    assert.equal(profile.referenceDamping, 0.15);
    assert.equal(profile.maxDampingRetries, 10);
    assert.equal(profile.dampingGrowthFactor, 4);
    assert.equal(profile.maxTrustRegionRetries, 10);
    assert.equal(profile.rhoAcceptanceThreshold, 0.02);
    assert.equal(profile.rhoShrinkThreshold, 0.02);
    assert.equal(profile.polishActivationResidual, 5e-4);
    assert.equal(profile.polishMaxIterations, 64);
    assert.equal(profile.polishDamping, 1e-7);
    assert.equal(profile.tag, 'cooperative_surface_polishing_dq_setpoint');
});
