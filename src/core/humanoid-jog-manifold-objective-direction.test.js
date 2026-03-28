import test from 'node:test';
import assert from 'node:assert/strict';

import {
    evaluateHumanoidJogManifoldObjectiveDirection,
    evaluateHumanoidJogManifoldNonDegradation
} from './humanoid-jog-manifold-objective-direction.js';

test('evaluateHumanoidJogManifoldObjectiveDirection accepts an increase only when the metric rises', () => {
    const accepted = evaluateHumanoidJogManifoldObjectiveDirection({
        before: 0.005,
        after: 0.006,
        direction: 'increase'
    });
    const rejected = evaluateHumanoidJogManifoldObjectiveDirection({
        before: 0.005,
        after: 0.004,
        direction: 'increase'
    });

    assert.equal(accepted.applied, true);
    assert.equal(rejected.applied, false);
});

test('evaluateHumanoidJogManifoldObjectiveDirection accepts a decrease only when the metric falls', () => {
    const accepted = evaluateHumanoidJogManifoldObjectiveDirection({
        before: 0.005,
        after: 0.004,
        direction: 'decrease'
    });
    const rejected = evaluateHumanoidJogManifoldObjectiveDirection({
        before: 0.005,
        after: 0.006,
        direction: 'decrease'
    });

    assert.equal(accepted.applied, true);
    assert.equal(rejected.applied, false);
});

test('evaluateHumanoidJogManifoldObjectiveDirection reports delta and reason', () => {
    const result = evaluateHumanoidJogManifoldObjectiveDirection({
        before: 0.005,
        after: 0.005,
        direction: 'increase'
    });

    assert.equal(result.applied, false);
    assert.equal(result.reason, 'not_improved');
    assert.equal(result.delta, 0);
});

test('evaluateHumanoidJogManifoldNonDegradation accepts unchanged or improved protected metrics', () => {
    const unchanged = evaluateHumanoidJogManifoldNonDegradation({
        before: 0.005,
        after: 0.005
    });
    const improved = evaluateHumanoidJogManifoldNonDegradation({
        before: 0.005,
        after: 0.006
    });

    assert.equal(unchanged.applied, true);
    assert.equal(unchanged.reason, 'protected');
    assert.equal(improved.applied, true);
    assert.equal(improved.reason, 'protected');
});

test('evaluateHumanoidJogManifoldNonDegradation rejects protected-metric regressions for increase mode', () => {
    const degraded = evaluateHumanoidJogManifoldNonDegradation({
        before: 0.005,
        after: 0.004
    });

    assert.equal(degraded.applied, false);
    assert.equal(degraded.reason, 'degraded');
    assert.equal(degraded.delta, -0.001);
});

test('evaluateHumanoidJogManifoldNonDegradation respects decrease direction', () => {
    const protectedDecrease = evaluateHumanoidJogManifoldNonDegradation({
        before: 0.005,
        after: 0.004,
        direction: 'decrease'
    });
    const degradedDecrease = evaluateHumanoidJogManifoldNonDegradation({
        before: 0.005,
        after: 0.006,
        direction: 'decrease'
    });

    assert.equal(protectedDecrease.applied, true);
    assert.equal(protectedDecrease.reason, 'protected');
    assert.equal(degradedDecrease.applied, false);
    assert.equal(degradedDecrease.reason, 'degraded');
});
