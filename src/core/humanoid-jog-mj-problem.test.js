import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogMJProblem } from './humanoid-jog-mj-problem.js';

function assertCloseArray(actual, expected, tolerance = 1e-12) {
    assert.equal(actual.length, expected.length);
    for (let i = 0; i < actual.length; i++) {
        assert.ok(Math.abs(actual[i] - expected[i]) <= tolerance, `index ${i}: ${actual[i]} != ${expected[i]}`);
    }
}

test('buildHumanoidJogMJProblem assembles equality blocks and reference coefficients', () => {
    const problem = buildHumanoidJogMJProblem({
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
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.3, 0.4],
        wvfiState: () => ({ value: 0.01, gradient: [0.5, -0.2] }),
        wvfiLowerBound: 0.02,
        referenceModel: {
            absolute: { stiffness: 1.0, damping: 0.25 },
            relative: { stiffness: 1.0, damping: 0.35 },
            bodyZ: { stiffness: 0.6, damping: 0.45 }
        }
    });

    assertCloseArray(problem.equalities.residual, [0.3, 0.5, 0.25]);
    assert.equal(problem.referenceModel.absolute.offset, 0);
    assert.equal(problem.referenceModel.relative.offset, 1);
    assert.equal(problem.referenceModel.bodyZ.offset, 2);
    assert.deepEqual(problem.referenceModel.stiffnessRows, [1.0, 1.0, 0.6]);
    assert.deepEqual(problem.referenceModel.dampingRows, [0.25, 0.35, 0.45]);
    assert.equal(problem.inequalities.wvfiFloor.enabled, true);
    assert.equal(problem.bounds.effectiveUpper[0], 0.3);
});

test('buildHumanoidJogMJProblem disables the w_vfi floor when lower bound is zero', () => {
    const problem = buildHumanoidJogMJProblem({
        q: [0.1, -0.2],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [0.7],
        relativePose: () => [1.5],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        wvfiState: () => ({ value: 0.01, gradient: [0.5, -0.2] }),
        wvfiLowerBound: 0
    });

    assert.equal(problem.inequalities.wvfiFloor.enabled, false);
    assert.equal(problem.inequalities.wvfiFloor.violation, 0);
});

test('buildHumanoidJogMJProblem exposes joint-limit VFI step rows for TR restoration', () => {
    const problem = buildHumanoidJogMJProblem({
        q: [0.8, -0.9],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [0.7],
        relativePose: () => [1.5],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        jointLimitEta: 2
    });

    assert.deepEqual(problem.inequalities.jointLimitVFI.rows, [
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1]
    ]);
    assertCloseArray(problem.inequalities.jointLimitVFI.rhs, [0.4, 1, 1, 0.2]);
    assert.equal(problem.inequalities.jointLimitVFI.count, 4);
});

test('buildHumanoidJogMJProblem stops joint-limit VFI inward motion inside the edge deadband', () => {
    const problem = buildHumanoidJogMJProblem({
        q: [0.99995, -0.99995],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [0.7],
        relativePose: () => [1.5],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        jointLimitEta: 5,
        jointLimitEdgeStopDistance: 1e-4
    });

    assert.deepEqual(problem.inequalities.jointLimitVFI.rows, [
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1]
    ]);
    assertCloseArray(problem.inequalities.jointLimitVFI.rhs, [0, 1, 1, 0]);
});

test('buildHumanoidJogMJProblem can disable joint-limit VFI rows while preserving other problem data', () => {
    const problem = buildHumanoidJogMJProblem({
        q: [0.8, -0.9],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [0.7],
        relativePose: () => [1.5],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        jointLimitEta: 2,
        enableJointLimitVFI: false
    });

    assert.equal(problem.inequalities.jointLimitVFI.enabled, false);
    assert.deepEqual(problem.inequalities.jointLimitVFI.rows, []);
    assert.deepEqual(problem.inequalities.jointLimitVFI.rhs, []);
    assert.equal(problem.inequalities.jointLimitVFI.count, 0);
    assert.equal(problem.bounds.effectiveUpper[0], 1);
    assert.equal(problem.bounds.effectiveLower[1], -1);
});

test('buildHumanoidJogMJProblem builds safety push rows away from tight joint limits', () => {
    const problem = buildHumanoidJogMJProblem({
        q: [0.96, -0.97, 0],
        qCurrent: [0, 0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [1],
        relativePose: () => [2],
        absoluteJacobian: () => [[1, 0, 0]],
        relativeJacobian: () => [[0, 1, 0]],
        lowerBounds: [-1, -1, -1],
        upperBounds: [1, 1, 1],
        stepBounds: [1, 1, 1],
        mjRestoreSafetyPushEnabled: true,
        jointLimitActivationThreshold: 0.2,
        safetyPushTargetFraction: 0.25,
        safetyPushMaxDistanceStep: 0.01
    });

    assert.equal(problem.safetyObjective.enabled, true);
    assert.deepEqual(problem.safetyObjective.rows, [
        [-1, 0, 0],
        [0, 1, 0]
    ]);
    assertCloseArray(problem.safetyObjective.targets, [0.01, 0.01]);
    assert.equal(problem.safetyObjective.count, 2);
});

test('buildHumanoidJogMJProblem builds safety push rows away from waist x corridor bounds', () => {
    const problem = buildHumanoidJogMJProblem({
        q: [0.46, 0],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [1],
        relativePose: () => [2],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        waistXCorridor: {
            enabled: true,
            evaluateX: (q) => q[0],
            min: -0.5,
            max: 0.5,
            vfiRows: [[1, 0], [-1, 0]],
            rhs: [0.04, 0.96],
            vfiConstraints: [
                { row: [-1, 0], distance: 0.04, threshold: 0.2 },
                { row: [1, 0], distance: 0.96, threshold: 0.2 }
            ]
        },
        mjRestoreSafetyPushEnabled: true,
        safetyPushTargetFraction: 0.25,
        safetyPushMaxDistanceStep: 0.01
    });

    assert.equal(problem.safetyObjective.enabled, true);
    assert.deepEqual(problem.safetyObjective.rows, [[-1, 0]]);
    assertCloseArray(problem.safetyObjective.targets, [0.01]);
    assert.equal(problem.safetyObjective.count, 1);
});

test('buildHumanoidJogMJProblem can exclude waist corridor from safety push while keeping joint retreat', () => {
    const problem = buildHumanoidJogMJProblem({
        q: [0.96, 0.46],
        qCurrent: [0, 0],
        absoluteTarget: [1],
        relativeTarget: [2],
        absolutePose: () => [1],
        relativePose: () => [2],
        absoluteJacobian: () => [[1, 0]],
        relativeJacobian: () => [[0, 1]],
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        waistXCorridor: {
            enabled: true,
            evaluateX: (q) => q[1],
            min: -0.5,
            max: 0.5,
            vfiRows: [[0, 1], [0, -1]],
            rhs: [0.04, 0.96],
            vfiConstraints: [
                { row: [0, -1], distance: 0.04, threshold: 0.2 },
                { row: [0, 1], distance: 0.96, threshold: 0.2 }
            ]
        },
        mjRestoreSafetyPushEnabled: true,
        safetyPushIncludeWaistCorridor: false,
        jointLimitActivationThreshold: 0.2,
        safetyPushTargetFraction: 0.25,
        safetyPushMaxDistanceStep: 0.01
    });

    assert.equal(problem.safetyObjective.enabled, true);
    assert.deepEqual(problem.safetyObjective.rows, [[-1, 0]]);
    assertCloseArray(problem.safetyObjective.targets, [0.01]);
    assert.equal(problem.safetyObjective.count, 1);
});
