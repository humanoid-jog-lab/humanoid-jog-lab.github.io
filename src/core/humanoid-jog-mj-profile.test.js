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
