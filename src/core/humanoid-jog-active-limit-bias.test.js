import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidJogActiveLimitRetreatBounds,
    buildHumanoidJogActiveLimitRetreatGradient,
    evaluateHumanoidJogActiveLimitSeverity
} from './humanoid-jog-active-limit-bias.js';

test('evaluateHumanoidJogActiveLimitSeverity returns the strongest active joint-limit row', () => {
    const result = evaluateHumanoidJogActiveLimitSeverity([
        { source: 'extra', distance: 0.001, threshold: 0.1 },
        { source: 'joint_limit', distance: 0.09, threshold: 0.15 },
        { source: 'joint_limit', distance: 0.01, threshold: 0.15 }
    ]);

    assert.equal(result.active, true);
    assert.ok(Math.abs(result.severity - (1 - (0.01 / 0.15))) <= 1e-12);
    assert.equal(result.minDistance, 0.01);
    assert.equal(result.threshold, 0.15);
});

test('buildHumanoidJogActiveLimitRetreatGradient biases the direction toward the active limit retreat vector', () => {
    const result = buildHumanoidJogActiveLimitRetreatGradient({
        objectiveGradient: [-1, 0],
        jointLimitGradient: [1, 0],
        wvfiGradient: [1, 0],
        constraints: [{ source: 'joint_limit', distance: 0.0015, threshold: 0.15 }],
        maxBias: 0.95
    });

    assert.ok(result.bias > 0.9);
    assert.ok(result.stepScale > 24.0);
    assert.ok(result.gradient[0] > 0);
    assert.ok(Math.abs(result.gradient[1]) <= 1e-12);
});

test('buildHumanoidJogActiveLimitRetreatGradient leaves the original objective when no joint-limit row is active', () => {
    const result = buildHumanoidJogActiveLimitRetreatGradient({
        objectiveGradient: [0, 2],
        jointLimitGradient: [1, 0],
        wvfiGradient: [1, 0],
        constraints: [{ source: 'joint_limit', distance: 0.2, threshold: 0.15 }]
    });

    assert.deepEqual(result.gradient, [0, 1]);
    assert.equal(result.bias, 0);
    assert.equal(result.stepScale, 1);
    assert.equal(result.active, false);
});

test('buildHumanoidJogActiveLimitRetreatBounds keeps a retreating lower-bound joint away from the exact limit during restore', () => {
    const result = buildHumanoidJogActiveLimitRetreatBounds({
        qCurrent: [0, 0.2],
        step: [0.0014, -0.002],
        activeLimits: [
            { lower: 0, upper: 2 },
            { lower: -1, upper: 1 }
        ],
        constraints: [
            { source: 'joint_limit', row: [1, 0], distance: 0, threshold: 0.15 }
        ],
        holdFraction: 0.5,
        minRetreatDistance: 1e-3,
        targetThresholdFraction: 0.4
    });

    assert.equal(result.active, true);
    assert.deepEqual(result.lockedIndices, [0]);
    assert.ok(Math.abs(result.lowerBounds[0] - 0.06) <= 1e-12);
    assert.equal(result.upperBounds[0], 2);
});

test('buildHumanoidJogActiveLimitRetreatBounds keeps a retreating upper-bound joint away from the exact limit during restore', () => {
    const result = buildHumanoidJogActiveLimitRetreatBounds({
        qCurrent: [0.4, 1],
        step: [0, -0.0014],
        activeLimits: [
            { lower: -1, upper: 1 },
            { lower: -1, upper: 1 }
        ],
        constraints: [
            { source: 'joint_limit', row: [0, 1], distance: 0, threshold: 0.15 }
        ],
        holdFraction: 0.5,
        minRetreatDistance: 1e-3,
        targetThresholdFraction: 0.4
    });

    assert.equal(result.active, true);
    assert.deepEqual(result.lockedIndices, [1]);
    assert.equal(result.lowerBounds[1], -1);
    assert.ok(Math.abs(result.upperBounds[1] - 0.94) <= 1e-12);
});

test('buildHumanoidJogActiveLimitRetreatBounds still enforces a tiny inward bound when an active joint is exactly on its limit', () => {
    const result = buildHumanoidJogActiveLimitRetreatBounds({
        qCurrent: [0, 1],
        step: [0, 0],
        activeLimits: [
            { lower: 0, upper: 2 },
            { lower: -1, upper: 1 }
        ],
        constraints: [
            { source: 'joint_limit', row: [1, 0], distance: 0, threshold: 0.15 },
            { source: 'joint_limit', row: [0, 1], distance: 0, threshold: 0.15 }
        ],
        minRetreatDistance: 1e-3,
        targetThresholdFraction: 0.4
    });

    assert.equal(result.active, true);
    assert.deepEqual(result.lockedIndices, [0, 1]);
    assert.ok(Math.abs(result.lowerBounds[0] - 0.06) <= 1e-12);
    assert.ok(Math.abs(result.upperBounds[1] - 0.94) <= 1e-12);
});

test('buildHumanoidJogActiveLimitRetreatBounds lets the severity target dominate over a tiny nullspace step', () => {
    const result = buildHumanoidJogActiveLimitRetreatBounds({
        qCurrent: [0],
        step: [0.002],
        activeLimits: [{ lower: 0, upper: 2 }],
        constraints: [
            { source: 'joint_limit', row: [1], distance: 0, threshold: 0.15 }
        ],
        holdFraction: 0.5,
        minRetreatDistance: 1e-3,
        targetThresholdFraction: 0.4
    });

    assert.equal(result.active, true);
    assert.deepEqual(result.lockedIndices, [0]);
    assert.ok(Math.abs(result.lowerBounds[0] - 0.06) <= 1e-12);
});

test('buildHumanoidJogActiveLimitRetreatBounds can cap the inward retreat distance for projection-only micro-steps', () => {
    const result = buildHumanoidJogActiveLimitRetreatBounds({
        qCurrent: [0],
        step: [0.002],
        activeLimits: [{ lower: 0, upper: 2 }],
        constraints: [
            { source: 'joint_limit', row: [1], distance: 0, threshold: 0.15 }
        ],
        holdFraction: 0.5,
        minRetreatDistance: 1e-3,
        targetThresholdFraction: 0.2,
        maxRetreatDistance: 0.002
    });

    assert.equal(result.active, true);
    assert.deepEqual(result.lockedIndices, [0]);
    assert.ok(Math.abs(result.lowerBounds[0] - 0.002) <= 1e-12);
});
