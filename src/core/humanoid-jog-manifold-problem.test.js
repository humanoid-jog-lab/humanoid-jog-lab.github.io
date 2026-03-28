import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogManifoldProblem } from './humanoid-jog-manifold-problem.js';

function assertApproxArray(actual, expected, tolerance = 1e-12) {
    assert.equal(actual.length, expected.length);
    for (let i = 0; i < actual.length; i++) {
        assert.ok(Math.abs(actual[i] - expected[i]) <= tolerance, `index ${i}: expected ${expected[i]}, got ${actual[i]}`);
    }
}

test('buildHumanoidJogManifoldProblem stacks terminal equalities and inequality metadata', () => {
    const problem = buildHumanoidJogManifoldProblem({
        q: [0.1, -0.2],
        qCurrent: [0, 0],
        absoluteTarget: [1, 2],
        relativeTarget: [-1, 0.5],
        absolutePose: () => [0.7, 1.5],
        relativePose: () => [-0.75, 0.8],
        absoluteJacobian: () => [[1, 0], [0, 1]],
        relativeJacobian: () => [[0.5, 0], [0, 0.25]],
        includeBodyZ: true,
        bodyZConstraint: () => ({
            residual: [0.25],
            jacobian: [[0.3, 0.4]]
        }),
        wvfiState: () => ({
            value: 0.012,
            gradient: [0.5, -0.25]
        }),
        wvfiLowerBound: 0.02,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.3, 0.4],
        waistXCorridor: {
            enabled: true,
            min: -0.05,
            max: 0.1,
            evaluateX: () => 0.15
        }
    });

    assertApproxArray(problem.equalities.residual, [0.3, 0.5, -0.25, -0.3, 0.25]);
    assert.deepEqual(problem.equalities.jacobian[0], [1, 0]);
    assert.deepEqual(problem.equalities.jacobian[4], [0.3, 0.4]);
    assert.equal(problem.equalities.blocks.absolute.offset, 0);
    assert.equal(problem.equalities.blocks.relative.offset, 2);
    assert.equal(problem.equalities.blocks.bodyZ.offset, 4);
    assert.equal(problem.equalities.rowCount, 5);
    assert.equal(problem.equalities.columnCount, 2);
    assert.ok(problem.equalities.residualNorm > 0);

    assert.equal(problem.inequalities.wvfiFloor.enabled, true);
    assert.ok(Math.abs(problem.inequalities.wvfiFloor.slack - (-0.008)) <= 1e-12);
    assert.ok(Math.abs(problem.inequalities.wvfiFloor.violation - 0.008) <= 1e-12);
    assert.deepEqual(problem.inequalities.wvfiFloor.gradient, [0.5, -0.25]);

    assert.equal(problem.inequalities.waistXCorridor.enabled, true);
    assert.ok(Math.abs(problem.inequalities.waistXCorridor.value - 0.15) <= 1e-12);
    assert.ok(Math.abs(problem.inequalities.waistXCorridor.violation - 0.05) <= 1e-12);

    assert.deepEqual(problem.bounds.lower, [-1, -1]);
    assert.deepEqual(problem.bounds.upper, [1, 1]);
    assert.deepEqual(problem.bounds.step, [0.3, 0.4]);
    assert.deepEqual(problem.bounds.effectiveLower, [-0.3, -0.4]);
    assert.deepEqual(problem.bounds.effectiveUpper, [0.3, 0.4]);
});

test('buildHumanoidJogManifoldProblem omits optional blocks when they are disabled', () => {
    let bodyCalls = 0;
    let waistCalls = 0;
    const problem = buildHumanoidJogManifoldProblem({
        q: [0.2, 0.3, 0.4],
        qCurrent: [0.1, 0.1, 0.1],
        absoluteTarget: [0, 0, 0],
        relativeTarget: [1, 1, 1],
        absolutePose: () => [0.1, -0.2, 0.3],
        relativePose: () => [1.1, 0.8, 1.3],
        absoluteJacobian: () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        relativeJacobian: () => [[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]],
        includeBodyZ: false,
        bodyZConstraint: () => {
            bodyCalls += 1;
            return { residual: [1], jacobian: [[1, 1, 1]] };
        },
        waistXCorridor: {
            enabled: false,
            evaluateX: () => {
                waistCalls += 1;
                return 0;
            }
        }
    });

    assert.equal(bodyCalls, 0);
    assert.equal(waistCalls, 0);
    assert.equal(problem.equalities.blocks.bodyZ, null);
    assert.equal(problem.inequalities.wvfiFloor.enabled, false);
    assert.equal(problem.inequalities.waistXCorridor.enabled, false);
    assertApproxArray(problem.equalities.residual, [-0.1, 0.2, -0.3, -0.1, 0.2, -0.3]);
});

test('buildHumanoidJogManifoldProblem gives the waist x corridor a tolerance deadband near the limit', () => {
    const problem = buildHumanoidJogManifoldProblem({
        q: [0.1],
        qCurrent: [0.1],
        absoluteTarget: [0],
        relativeTarget: [0],
        absolutePose: () => [0],
        relativePose: () => [0],
        absoluteJacobian: () => [[1]],
        relativeJacobian: () => [[1]],
        waistXCorridor: {
            enabled: true,
            min: -0.05,
            max: 0.1,
            tolerance: 5e-4,
            evaluateX: () => 0.1003
        }
    });

    assert.equal(problem.inequalities.waistXCorridor.enabled, true);
    assert.ok(Math.abs(problem.inequalities.waistXCorridor.value - 0.1003) <= 1e-12);
    assert.ok(Math.abs(problem.inequalities.waistXCorridor.violation) <= 1e-12);
});

test('buildHumanoidJogManifoldProblem preserves waist x corridor VFI rows for smooth boundary slowdown', () => {
    const problem = buildHumanoidJogManifoldProblem({
        q: [0.1],
        qCurrent: [0.1],
        absoluteTarget: [0],
        relativeTarget: [0],
        absolutePose: () => [0],
        relativePose: () => [0],
        absoluteJacobian: () => [[1]],
        relativeJacobian: () => [[1]],
        waistXCorridor: {
            enabled: true,
            min: -0.05,
            max: 0.1,
            tolerance: 5e-4,
            evaluateX: () => 0.099,
            vfiRows: [[1]],
            rhs: [0.002]
        }
    });

    assert.equal(problem.inequalities.waistXCorridor.enabled, true);
    assert.deepEqual(problem.inequalities.waistXCorridor.vfiRows, [[1]]);
    assert.deepEqual(problem.inequalities.waistXCorridor.rhs, [0.002]);
});

test('buildHumanoidJogManifoldProblem exposes weighted equalities for restoration', () => {
    const problem = buildHumanoidJogManifoldProblem({
        q: [0.1, -0.2],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [0.7],
        relativePose: () => [1.5],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        includeBodyZ: true,
        bodyZConstraint: () => ({
            residual: [0.25],
            jacobian: [[0.3, 0.4]]
        }),
        absoluteWeight: 1,
        relativeWeight: 2,
        bodyWeight: 4
    });

    assertApproxArray(problem.equalities.residual, [0.3, 0.5, 0.25]);
    assertApproxArray(problem.equalities.weightedResidual, [0.3, 1.0, 1.0]);
    assert.deepEqual(problem.equalities.weightedJacobian[0], [1, 0]);
    assert.deepEqual(problem.equalities.weightedJacobian[1], [0, 2]);
    assert.deepEqual(problem.equalities.weightedJacobian[2], [1.2, 1.6]);
});

test('buildHumanoidJogManifoldProblem removes body_z from active restoration rows once it is within tolerance', () => {
    const problem = buildHumanoidJogManifoldProblem({
        q: [0.1, -0.2],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [0.7],
        relativePose: () => [1.5],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        includeBodyZ: true,
        bodyZConstraint: () => ({
            residual: [2e-8],
            jacobian: [[0.3, 0.4]]
        }),
        bodyEqualityTolerance: 5e-5
    });

    assertApproxArray(problem.equalities.residual, [0.3, 0.5, 2e-8]);
    assertApproxArray(problem.equalities.weightedResidual, [0.3, 0.5]);
    assertApproxArray(problem.equalities.activeResidual, [0.3, 0.5]);
    assert.equal(problem.equalities.weightedJacobian.length, 2);
    assert.equal(problem.equalities.activeJacobian.length, 2);
    assert.ok(Math.abs(problem.equalities.activeResidualNorm - Math.hypot(0.3, 0.5)) <= 1e-12);
    assert.equal(problem.equalities.blocks.bodyZ.length, 1);
});
