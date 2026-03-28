import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogManifoldStepBounds } from './humanoid-jog-manifold-profile.js';

test('buildHumanoidJogManifoldStepBounds keeps arm-only maxStep unchanged', () => {
    const bounds = buildHumanoidJogManifoldStepBounds({
        maxStep: 0.1,
        includeWaist: false,
        keepBodyZVertical: false,
        waistCount: 4,
        activeCount: 14
    });

    assert.deepEqual(bounds, new Array(14).fill(0.1));
});

test('buildHumanoidJogManifoldStepBounds uses full waist step by default in manifold mode', () => {
    const bounds = buildHumanoidJogManifoldStepBounds({
        maxStep: 0.1,
        includeWaist: true,
        keepBodyZVertical: false,
        waistCount: 4,
        activeCount: 18
    });

    assert.deepEqual(bounds.slice(0, 4), [0.1, 0.1, 0.1, 0.1]);
    assert.deepEqual(bounds.slice(4), new Array(14).fill(0.1));
});

test('buildHumanoidJogManifoldStepBounds supports an explicit waist step fraction override', () => {
    const bounds = buildHumanoidJogManifoldStepBounds({
        maxStep: 0.1,
        includeWaist: true,
        keepBodyZVertical: false,
        waistCount: 2,
        activeCount: 6,
        waistStepFraction: 0.5
    });

    assert.deepEqual(bounds, [0.05, 0.05, 0.1, 0.1, 0.1, 0.1]);
});

test('buildHumanoidJogManifoldStepBounds ignores waist fraction when body_z must stay vertical', () => {
    const bounds = buildHumanoidJogManifoldStepBounds({
        maxStep: 0.1,
        includeWaist: true,
        keepBodyZVertical: true,
        waistCount: 3,
        activeCount: 7,
        waistStepFraction: 0.25
    });

    assert.deepEqual(bounds, new Array(7).fill(0.1));
});
