import test from 'node:test';
import assert from 'node:assert/strict';

import {
    DEFAULT_HUMANOID_INITIAL_Q,
    DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2,
    normalizeHumanoidInitialQProfileId,
    resolveHumanoidInitialQ
} from './humanoid-default-q-profiles.js';

test('resolveHumanoidInitialQ uses the configured initial_q when it matches the humanoid joint count', () => {
    const configured = Array.from({ length: 20 }, (_, index) => index * 0.1);
    const resolved = resolveHumanoidInitialQ({ initial_q: configured }, 20);
    assert.deepEqual(resolved, configured);
    assert.notEqual(resolved, configured);
});

test('resolveHumanoidInitialQ falls back to the default humanoid pose when config.initial_q is absent', () => {
    const resolved = resolveHumanoidInitialQ({}, 20);
    assert.deepEqual(resolved, DEFAULT_HUMANOID_INITIAL_Q);
    assert.notEqual(resolved, DEFAULT_HUMANOID_INITIAL_Q);
});

test('resolveHumanoidInitialQ pads with zeros when the humanoid has more joints than the default pose definition', () => {
    const resolved = resolveHumanoidInitialQ({}, 22);
    assert.deepEqual(resolved.slice(0, 20), DEFAULT_HUMANOID_INITIAL_Q);
    assert.deepEqual(resolved.slice(20), [0, 0]);
});

test('resolveHumanoidInitialQ supports the second default humanoid pose profile', () => {
    const resolved = resolveHumanoidInitialQ({}, 20, 'default2');
    assert.deepEqual(resolved, DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2);
    assert.notEqual(resolved, DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2);
    assert.deepEqual(resolved, [
        -0.3, 0.9, -0.6, 0,
        -0.51865, 0.66105, 0.90855, -1.5711, -0.93562, 0.67402, 1.40137,
        0.56421, -0.68286, -0.98744, 1.58165, 1.00933, -0.71114, -1.35432,
        0, 0
    ]);
});

test('normalizeHumanoidInitialQProfileId falls back to default1 for unknown values', () => {
    assert.equal(normalizeHumanoidInitialQProfileId('default1'), 'default1');
    assert.equal(normalizeHumanoidInitialQProfileId('default2'), 'default2');
    assert.equal(normalizeHumanoidInitialQProfileId('weird-profile'), 'default1');
    assert.equal(normalizeHumanoidInitialQProfileId(null), 'default1');
});
