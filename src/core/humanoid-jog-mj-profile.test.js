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

test('resolveHumanoidJogMJRestoreProfile uses a more aggressive single-arm profile for surface-slide', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: false,
        fallbackReason: '',
        controlMode: 'surface-slide',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreDamping: 1e-4,
            mjRestoreReferenceDamping: 0.35
        }
    });

    assert.equal(profile.maxIterations, 160);
    assert.equal(profile.damping, 5e-5);
    assert.equal(profile.referenceDamping, 0.2);
    assert.equal(profile.tag, 'surface_slide');
});

test('resolveHumanoidJogMJRestoreProfile uses the same aggressive profile for dual-independent surface-slide', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: true,
        fallbackReason: '',
        controlMode: 'dual-independent-surface-slide',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreDamping: 1e-4,
            mjRestoreReferenceDamping: 0.35
        }
    });

    assert.equal(profile.maxIterations, 160);
    assert.equal(profile.damping, 5e-5);
    assert.equal(profile.referenceDamping, 0.2);
    assert.equal(profile.tag, 'surface_slide');
});

test('resolveHumanoidJogMJRestoreProfile uses a dedicated steadier profile for cooperative ruled-surface polishing', () => {
    const profile = resolveHumanoidJogMJRestoreProfile({
        includeWaist: true,
        fallbackReason: '',
        controlMode: 'cooperative-ruled-surface-polishing',
        config: {
            mjRestoreMaxIter: 120,
            mjRestoreDamping: 1e-4,
            mjRestoreReferenceDamping: 0.35
        }
    });

    assert.equal(profile.maxIterations, 120);
    assert.equal(profile.damping, 1e-4);
    assert.equal(profile.referenceDamping, 0.3);
    assert.equal(profile.tag, 'cooperative_surface_polishing');
});
