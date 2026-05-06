import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildBodyVerticalLineTask,
    computeLineDirectionMismatchSquared,
    extractBodyZAxisLineDirectionVec4
} from './body-vertical-utils.js';

test('extractBodyZAxisLineDirectionVec4 returns the third rotation-matrix column as a pure quaternion', () => {
    const rotMat = [
        [1, 0, 0.2],
        [0, 1, -0.3],
        [0, 0, 0.9327379053088815]
    ];

    assert.deepEqual(extractBodyZAxisLineDirectionVec4(rotMat), [0, 0.2, -0.3, 0.9327379053088815]);
});

test('computeLineDirectionMismatchSquared matches the squared distance to the reference line direction', () => {
    const current = [0, 0.2, -0.3, 0.9];
    const reference = [0, 0, 0, 1];

    const { diffVec4, errorScalar } = computeLineDirectionMismatchSquared(current, reference);

    assert.equal(diffVec4[0], 0);
    assert.ok(Math.abs(diffVec4[1] + 0.2) < 1e-12);
    assert.ok(Math.abs(diffVec4[2] - 0.3) < 1e-12);
    assert.ok(Math.abs(diffVec4[3] - 0.1) < 1e-12);
    assert.ok(Math.abs(errorScalar - 0.14) < 1e-12);
});

test('buildBodyVerticalLineTask Jacobian matches the scalar tilt residual derivative', () => {
    const rotMat = [
        [1, 0, 0.2],
        [0, 1, -0.3],
        [0, 0, 0.9]
    ];
    const lineJacobianFull = [
        [0.5, -0.5],
        [1.0, 2.0],
        [3.0, 4.0],
        [5.0, 6.0]
    ];

    const task = buildBodyVerticalLineTask({
        rotMat,
        lineJacobianFull,
        armStart: 0,
        activeCount: 2
    });

    assert.deepEqual(task.currentVec4, [0, 0.2, -0.3, 0.9]);
    assert.equal(task.diffVec4[0], 0);
    assert.ok(Math.abs(task.diffVec4[1] + 0.2) < 1e-12);
    assert.ok(Math.abs(task.diffVec4[2] - 0.3) < 1e-12);
    assert.ok(Math.abs(task.diffVec4[3] - 0.1) < 1e-12);
    assert.equal(task.error.length, 1);
    assert.ok(Math.abs(task.mismatchSquared - 0.14) < 1e-12);
    assert.ok(Math.abs(task.error[0] + 0.14) < 1e-12);
    assert.ok(Math.abs(task.errorNorm - Math.sqrt(0.14)) < 1e-12);
    assert.equal(task.Jrows.length, 1);
    assert.ok(Math.abs(task.Jrows[0][0] + 2.4) < 1e-12);
    assert.ok(Math.abs(task.Jrows[0][1] + 2.8) < 1e-12);
});

test('buildBodyVerticalLineTask step direction reduces a small body-z tilt', () => {
    const tiltX = 0.2;
    const rotMat = [
        [1, 0, tiltX],
        [0, 1, 0],
        [0, 0, Math.sqrt(1 - tiltX * tiltX)]
    ];
    const lineJacobianFull = [
        [0],
        [-1],
        [0],
        [0]
    ];

    const task = buildBodyVerticalLineTask({
        rotMat,
        lineJacobianFull,
        activeCount: 1
    });

    const step = task.error[0] / task.Jrows[0][0];
    const nextTiltX = tiltX + lineJacobianFull[1][0] * step;

    assert.ok(nextTiltX > 0);
    assert.ok(nextTiltX < tiltX);
});

test('buildBodyVerticalLineTask respects the active-joint slice', () => {
    const rotMat = [
        [1, 0, 0.1],
        [0, 1, 0.2],
        [0, 0, 0.975]
    ];
    const lineJacobianFull = [
        [10, 11, 12],
        [1, 2, 3],
        [4, 5, 6],
        [7, 8, 9]
    ];

    const task = buildBodyVerticalLineTask({
        rotMat,
        lineJacobianFull,
        armStart: 1,
        activeCount: 2
    });

    assert.deepEqual(task.directionRows, [
        [11, 12],
        [2, 3],
        [5, 6],
        [8, 9]
    ]);
    assert.equal(task.Jrows[0].length, 2);
});
