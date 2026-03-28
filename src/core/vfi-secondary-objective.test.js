import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildManipulabilityVFIConstraints,
    computeProtectedSecondaryGradient,
    projectSecondaryGradient,
    summarizeManipulabilityVFIConstraints
} from './vfi-secondary-objective.js';

test('buildManipulabilityVFIConstraints combines near joint-limit and extra VFI constraints', () => {
    const constraints = buildManipulabilityVFIConstraints({
        qActive: [0.95, 1.0],
        activeLimits: [
            { lower: 0, upper: 1 },
            { lower: -2, upper: 2 }
        ],
        activationThreshold: 0.1,
        extraConstraints: [
            { row: [0, 1], distance: 0.04, threshold: 0.2 }
        ]
    });

    assert.equal(constraints.length, 2);
    assert.deepEqual(constraints[0], {
        row: [1, 0],
        distance: 0.050000000000000044,
        threshold: 0.1,
        source: 'joint_limit'
    });
    assert.deepEqual(constraints[1], {
        row: [0, 1],
        distance: 0.04,
        threshold: 0.2,
        source: 'extra'
    });
});

test('summarizeManipulabilityVFIConstraints counts joint-limit and extra rows separately', () => {
    const summary = summarizeManipulabilityVFIConstraints([
        { row: [1, 0], distance: 0.05, threshold: 0.1, source: 'joint_limit' },
        { row: [0, 1], distance: 0.04, threshold: 0.2, source: 'extra' },
        { row: [1, 1], distance: 0.02, threshold: 0.2, source: 'extra' }
    ]);

    assert.deepEqual(summary, {
        jointLimitCount: 1,
        extraCount: 2,
        totalCount: 3,
        minDistance: 0.02
    });
});

test('projectSecondaryGradient removes protected row-space components and normalizes the remainder', () => {
    const projected = projectSecondaryGradient([3, 4], [[1, 0]], {
        damping: 1e-9,
        normalize: true
    });

    assert.ok(Math.abs(projected[0]) < 1e-8);
    assert.ok(Math.abs(projected[1] - 1) < 1e-8);
});

test('computeProtectedSecondaryGradient returns zero when the ascent direction is fully protected', () => {
    const gradient = computeProtectedSecondaryGradient({
        evalFn: (q) => ({ w_vfi: q[0] }),
        qActive: [0, 0],
        protectRows: [[1, 0]],
        eps: 1e-6,
        damping: 1e-9
    });

    assert.deepEqual(gradient, [0, 0]);
});

test('computeProtectedSecondaryGradient preserves ascent along unprotected directions', () => {
    const gradient = computeProtectedSecondaryGradient({
        evalFn: (q) => ({ w_vfi: 2 * q[0] + 3 * q[1] }),
        qActive: [0, 0],
        protectRows: [[1, 0]],
        eps: 1e-6,
        damping: 1e-9
    });

    assert.ok(Math.abs(gradient[0]) < 1e-6);
    assert.ok(Math.abs(gradient[1] - 1) < 1e-6);
});

test('projectSecondaryGradient keeps fully protected redundant row spaces at zero instead of normalizing residue', () => {
    const projected = projectSecondaryGradient([1, 2], [
        [1, 0],
        [0, 1],
        [1, 1]
    ], {
        damping: 1e-6,
        normalize: true
    });

    assert.deepEqual(projected, [0, 0]);
});
