import test from 'node:test';
import assert from 'node:assert/strict';

import { computeVFIAttenuationMatrix } from './cctsm.js';

test('computeVFIAttenuationMatrix keeps joint-limit attenuation sensitive near exact limits instead of saturating early', () => {
    const exactLimit = computeVFIAttenuationMatrix([
        { row: [1], distance: 0, threshold: 0.15, source: 'joint_limit' }
    ], 1);
    const tinyRetreat = computeVFIAttenuationMatrix([
        { row: [1], distance: 1e-5, threshold: 0.15, source: 'joint_limit' }
    ], 1);

    assert.equal(exactLimit[0][0], 0);
    assert.ok(tinyRetreat[0][0] > 0);
    assert.ok(tinyRetreat[0][0] < 1e-3);
});
