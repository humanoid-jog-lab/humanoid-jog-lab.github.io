import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogMJDistanceProblem } from './humanoid-jog-mj-distance-problem.js';

function assertCloseArray(actual, expected, tolerance = 1e-12) {
    assert.equal(actual.length, expected.length);
    for (let i = 0; i < actual.length; i++) {
        assert.ok(Math.abs(actual[i] - expected[i]) <= tolerance, `index ${i}: ${actual[i]} != ${expected[i]}`);
    }
}

test('buildHumanoidJogMJDistanceProblem assembles a distance command block plus invariant reference coefficients', () => {
    const problem = buildHumanoidJogMJDistanceProblem({
        q: [0.1, -0.2],
        qCurrent: [0, 0],
        distanceTarget: 0.49,
        currentDistance: 0.36,
        distanceJacobian: () => [[0.4, 0.6]],
        absoluteTarget: [1.0, 0.2],
        absolutePose: () => [0.7, -0.1],
        absoluteJacobian: () => [[1, 0], [0, 1]],
        relativeRotationTarget: [0.5],
        relativeRotation: () => [0.45],
        relativeRotationJacobian: () => [[0.3, 0.1]],
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.3, 0.4],
        referenceModel: {
            absolute: { stiffness: 1.0, damping: 0.25 },
            relative: { stiffness: 1.0, damping: 0.35 },
            bodyZ: { stiffness: 0.6, damping: 0.45 }
        }
    });

    assertCloseArray(problem.equalities.residual, [0.13, 0.3, 0.3, 0.05]);
    assert.equal(problem.equalities.blocks.command.length, 1);
    assert.equal(problem.equalities.blocks.absolute.length, 2);
    assert.equal(problem.equalities.blocks.relative.length, 1);
    assert.equal(problem.referenceModel.absolute.offset, 0);
    assert.equal(problem.referenceModel.relative.offset, 3);
    assert.equal(problem.referenceModel.bodyZ.offset, 4);
    assert.deepEqual(problem.referenceModel.stiffnessRows, [1.0, 1.0, 1.0, 1.0]);
    assert.deepEqual(problem.referenceModel.dampingRows, [0.25, 0.25, 0.25, 0.35]);
    assert.equal(problem.inequalities.wvfiFloor.enabled, false);
});

test('buildHumanoidJogMJDistanceProblem preserves waist corridor inequalities when provided', () => {
    const problem = buildHumanoidJogMJDistanceProblem({
        q: [0.1, -0.2],
        qCurrent: [0, 0],
        distanceTarget: 0.49,
        currentDistance: 0.36,
        distanceJacobian: () => [[0.4, 0.6]],
        absoluteTarget: [1.0],
        absolutePose: () => [0.7],
        absoluteJacobian: () => [[1, 0]],
        relativeRotationTarget: [],
        relativeRotation: () => [],
        relativeRotationJacobian: () => [],
        waistXCorridor: {
            enabled: true,
            min: -0.05,
            max: 0.05,
            tolerance: 1e-3,
            vfiRows: [[1, 0]],
            rhs: [0.01]
        }
    });

    assert.equal(problem.inequalities.waistXCorridor.enabled, true);
    assert.deepEqual(problem.inequalities.waistXCorridor.vfiRows, [[1, 0]]);
    assert.deepEqual(problem.inequalities.waistXCorridor.rhs, [0.01]);
});
