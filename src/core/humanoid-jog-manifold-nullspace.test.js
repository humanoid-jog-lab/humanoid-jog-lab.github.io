import test from 'node:test';
import assert from 'node:assert/strict';

import {
    computeHumanoidJogManifoldNullspaceStep,
    computeHumanoidJogManifoldTaskCorrection
} from './humanoid-jog-manifold-nullspace.js';

function matVec(A, x) {
    return A.map((row) => row.reduce((sum, value, index) => sum + value * x[index], 0));
}

test('computeHumanoidJogManifoldNullspaceStep keeps the first-order equality residual unchanged', () => {
    const result = computeHumanoidJogManifoldNullspaceStep({
        qCurrent: [0, 0],
        equalityJacobian: [[1, 0]],
        wvfiGradient: [1, 2],
        stepSize: 0.5,
        lowerBounds: [-10, -10],
        upperBounds: [10, 10],
        stepBounds: [1, 1]
    });

    assert.equal(result.accepted, true);
    assert.ok(result.step[1] > 0);
    assert.ok(Math.abs(result.step[0]) <= 1e-9);
    const residualChange = matVec([[1, 0]], result.step);
    assert.ok(Math.abs(residualChange[0]) <= 1e-9);
});

test('computeHumanoidJogManifoldNullspaceStep vanishes when there is no nullspace', () => {
    const result = computeHumanoidJogManifoldNullspaceStep({
        qCurrent: [0, 0],
        equalityJacobian: [[1, 0], [0, 1]],
        wvfiGradient: [1, 2],
        stepSize: 0.5,
        lowerBounds: [-10, -10],
        upperBounds: [10, 10],
        stepBounds: [1, 1]
    });

    assert.equal(result.accepted, false);
    assert.equal(result.reason, 'no_nullspace');
    assert.deepEqual(result.step, [0, 0]);
});

test('computeHumanoidJogManifoldNullspaceStep rejects steps that violate the w_vfi floor', () => {
    const result = computeHumanoidJogManifoldNullspaceStep({
        qCurrent: [0, 0.2],
        equalityJacobian: [[1, 0]],
        wvfiGradient: [0, -1],
        stepSize: 0.5,
        lowerBounds: [-10, -10],
        upperBounds: [10, 10],
        stepBounds: [1, 1],
        evaluateWVFI: (q) => q[1],
        wvfiMin: 0.1
    });

    assert.equal(result.accepted, false);
    assert.equal(result.reason, 'wvfi_lower_bound');
    assert.deepEqual(result.step, [0, 0]);
});

test('computeHumanoidJogManifoldNullspaceStep supports descending w_vfi in the equality nullspace', () => {
    const result = computeHumanoidJogManifoldNullspaceStep({
        qCurrent: [0, 0],
        equalityJacobian: [[1, 0]],
        wvfiGradient: [0, 1],
        stepSize: -0.5,
        lowerBounds: [-10, -10],
        upperBounds: [10, 10],
        stepBounds: [1, 1]
    });

    assert.equal(result.accepted, true);
    assert.ok(result.step[1] < 0);
    assert.ok(Math.abs(result.step[0]) <= 1e-9);
});

test('computeHumanoidJogManifoldNullspaceStep accepts a generic objective gradient for plain manipulability ascent', () => {
    const result = computeHumanoidJogManifoldNullspaceStep({
        qCurrent: [0, 0],
        equalityJacobian: [[1, 0]],
        objectiveGradient: [0, 1],
        stepSize: 0.5,
        lowerBounds: [-10, -10],
        upperBounds: [10, 10],
        stepBounds: [1, 1]
    });

    assert.equal(result.accepted, true);
    assert.ok(result.step[1] > 0);
    assert.ok(Math.abs(result.step[0]) <= 1e-9);
});

test('computeHumanoidJogManifoldNullspaceStep falls back to wvfi gradient in nullspace', () => {
    const result = computeHumanoidJogManifoldNullspaceStep({
        qCurrent: [0, 0],
        equalityJacobian: [[1, 0]],
        objectiveGradient: [0, 0],
        wvfiGradient: [0, 1],
        stepSize: 0.5,
        lowerBounds: [-10, -10],
        upperBounds: [10, 10],
        stepBounds: [1, 1]
    });

    assert.equal(result.accepted, true);
    assert.equal(result.gradientSource, 'wvfi');
    assert.ok(result.step[1] > 0);
    assert.ok(result.step[0] <= 1e-9);
});

test('computeHumanoidJogManifoldNullspaceStep can prefer the wvfi gradient when the objective step clips into a joint limit', () => {
    const result = computeHumanoidJogManifoldNullspaceStep({
        qCurrent: [0.001],
        equalityJacobian: [],
        objectiveGradient: [-1],
        wvfiGradient: [1],
        stepSize: 0.01,
        lowerBounds: [0],
        upperBounds: [1],
        stepBounds: [0.1],
        preferWVFIFallbackOnClip: true
    });

    assert.equal(result.accepted, true);
    assert.equal(result.gradientSource, 'wvfi');
    assert.ok(result.step[0] > 0);
    assert.ok(result.qNext[0] > 0.001);
});

test('computeHumanoidJogManifoldTaskCorrection can preserve safety-critical joints with regularization weights', () => {
    const unweighted = computeHumanoidJogManifoldTaskCorrection({
        qCurrent: [0, -0.02, 0],
        equalityJacobian: [[1, 10, 1]],
        residual: [0.2],
        lowerBounds: [-1, -1, -1],
        upperBounds: [1, 1, 1],
        stepBounds: [1, 1, 1]
    });
    const weighted = computeHumanoidJogManifoldTaskCorrection({
        qCurrent: [0, -0.02, 0],
        equalityJacobian: [[1, 10, 1]],
        residual: [0.2],
        lowerBounds: [-1, -1, -1],
        upperBounds: [1, 1, 1],
        stepBounds: [1, 1, 1],
        regularizationWeights: [1, 1000, 1]
    });

    assert.equal(unweighted.accepted, true);
    assert.equal(weighted.accepted, true);
    assert.ok(Math.abs(weighted.step[1]) < Math.abs(unweighted.step[1]) * 0.1);
});
