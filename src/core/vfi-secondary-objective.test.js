import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildVFIMarginAscentGradient,
    buildManipulabilityVFIConstraints,
    computeProtectedForwardSecondaryGradient,
    computeProtectedSPSASecondaryGradient,
    computeProtectedSecondaryGradient,
    projectSecondaryGradient,
    summarizeManipulabilityVFIConstraints
} from './vfi-secondary-objective.js';

test('buildManipulabilityVFIConstraints collects all finite joint-limit distances and extra VFI constraints', () => {
    const constraints = buildManipulabilityVFIConstraints({
        qActive: [0.95, 0.0],
        activeLimits: [
            { lower: 0, upper: 1 },
            { lower: -2, upper: 2 }
        ],
        activationThreshold: 0.1,
        extraConstraints: [
            { row: [1, 1], distance: 0.04, threshold: 0.2 }
        ]
    });

    assert.equal(constraints.length, 3);
    assert.deepEqual(constraints[0], {
        row: [1, 0],
        distance: 0.050000000000000044,
        threshold: 0.1,
        source: 'joint_limit'
    });
    assert.deepEqual(constraints[1], {
        row: [0, 1],
        distance: 2,
        threshold: 0.1,
        source: 'joint_limit'
    });
    assert.deepEqual(constraints[2], {
        row: [1, 1],
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

test('buildVFIMarginAscentGradient points inward for joint limits and extra corridor rows', () => {
    const gradient = buildVFIMarginAscentGradient({
        qActive: [0.95, -0.95, 0],
        activeLimits: [
            { lower: 0, upper: 1 },
            { lower: -1, upper: 1 },
            { lower: -2, upper: 2 }
        ],
        activationThreshold: 0.1,
        extraConstraints: [
            { row: [0, 0, -1], distance: 0.02, threshold: 0.1 }
        ]
    });

    assert.ok(gradient[0] < 0, 'upper-limit margin should push negative');
    assert.ok(gradient[1] > 0, 'lower-limit margin should push positive');
    assert.ok(gradient[2] < 0, 'extra corridor row should be included');
    const norm = Math.sqrt(gradient.reduce((sum, value) => sum + value * value, 0));
    assert.ok(Math.abs(norm - 1) < 1e-9);
});

test('buildVFIMarginAscentGradient ignores margins outside the secondary activation distance', () => {
    const gradient = buildVFIMarginAscentGradient({
        qActive: [0.85, 0],
        activeLimits: [
            { lower: 0, upper: 1 },
            { lower: -1, upper: 1 }
        ],
        activationThreshold: 0.15,
        activationDistance: 0.08
    });

    assert.deepEqual(gradient, [0, 0]);
});

test('buildVFIMarginAscentGradient activates within the w_vfi joint-limit band', () => {
    const gradient = buildVFIMarginAscentGradient({
        qActive: [0.88],
        activeLimits: [
            { lower: 0, upper: 1 }
        ],
        activationThreshold: 0.15,
        activationDistance: 0.15
    });

    assert.ok(gradient[0] < 0, '0.12 rad upper-limit margin should still retreat inside the 0.15 rad w_vfi band');
});

test('buildVFIMarginAscentGradient log_margin prioritizes the closest active margin', () => {
    const options = {
        qActive: [0, 0],
        activeLimits: null,
        activationThreshold: 0.1,
        activationDistance: 0.1,
        extraConstraints: [
            { row: [1, 0], distance: 0.01, threshold: 0.1 },
            { row: [0, 1], distance: 0.08, threshold: 0.1 }
        ]
    };
    const marginGradient = buildVFIMarginAscentGradient(options);
    const logGradient = buildVFIMarginAscentGradient({
        ...options,
        objective: 'log_margin',
        logEpsilon: 1e-4
    });
    const marginRatio = Math.abs(marginGradient[0] / marginGradient[1]);
    const logRatio = Math.abs(logGradient[0] / logGradient[1]);

    assert.ok(logRatio > marginRatio * 2, 'log-margin weighting should favor the smaller clearance more strongly');
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

test('computeProtectedForwardSecondaryGradient uses one base plus one scalar evaluation per joint', () => {
    let evalCount = 0;
    const gradient = computeProtectedForwardSecondaryGradient({
        evalFn: (q) => {
            evalCount += 1;
            return { w_vfi: q[0] + 2 * q[1] + 3 * q[2] + 4 * q[3] };
        },
        qActive: [0, 0, 0, 0],
        protectRows: [],
        eps: 1e-5,
        damping: 1e-9
    });

    assert.equal(evalCount, 5);
    assert.equal(gradient.length, 4);
    const norm = Math.sqrt(1 + 4 + 9 + 16);
    assert.ok(Math.abs(gradient[0] - 1 / norm) < 1e-8);
    assert.ok(Math.abs(gradient[3] - 4 / norm) < 1e-8);
});

test('computeProtectedSPSASecondaryGradient uses two scalar evaluations per sample', () => {
    let evalCount = 0;
    const gradient = computeProtectedSPSASecondaryGradient({
        evalFn: (q) => {
            evalCount += 1;
            return { w_vfi: q[0] + 2 * q[1] + 3 * q[2] + 4 * q[3] };
        },
        qActive: [0, 0, 0, 0],
        protectRows: [],
        eps: 1e-4,
        samples: 2,
        damping: 1e-9
    });

    assert.equal(evalCount, 4);
    assert.equal(gradient.length, 4);
    const norm = Math.sqrt(gradient.reduce((sum, value) => sum + value * value, 0));
    assert.ok(Math.abs(norm - 1) < 1e-9);
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
