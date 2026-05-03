import test from 'node:test';
import assert from 'node:assert/strict';

import {
    computeExtendedConditionManipulability,
    computeTwistJacobian,
    computeVFIAttenuationMatrix
} from './cctsm.js';

function multiplyQuaternions(a, b) {
    const [aw, ax, ay, az] = a;
    const [bw, bx, by, bz] = b;
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw
    ];
}

function buildDualQuaternion(rotationQuaternion, translationVector) {
    const translationQuaternion = [0, ...translationVector];
    const dualPart = multiplyQuaternions(
        translationQuaternion.map((value) => value * 0.5),
        rotationQuaternion
    );
    return [...rotationQuaternion, ...dualPart];
}

function dqConjugate(x) {
    return [
        x[0],
        -x[1],
        -x[2],
        -x[3],
        -x[4],
        x[5],
        x[6],
        x[7]
    ];
}

function multiplyDualQuaternions(a, b) {
    const ar = a.slice(0, 4);
    const ad = a.slice(4, 8);
    const br = b.slice(0, 4);
    const bd = b.slice(4, 8);
    const real = multiplyQuaternions(ar, br);
    const dualLeft = multiplyQuaternions(ar, bd);
    const dualRight = multiplyQuaternions(ad, br);
    return real.map((value, index) => value).concat(
        dualLeft.map((value, index) => value + dualRight[index])
    );
}

test('computeVFIAttenuationMatrix keeps joint-limit attenuation sensitive near exact limits instead of saturating early', () => {
    const exactLimit = computeVFIAttenuationMatrix([
        { row: [1], distance: 0, threshold: 0.15, source: 'joint_limit' }
    ], 1);
    const tinyRetreat = computeVFIAttenuationMatrix([
        { row: [1], distance: 1e-5, threshold: 0.15, source: 'joint_limit' }
    ], 1);

    assert.equal(exactLimit[0][0], 0);
    assert.ok(tinyRetreat[0][0] > 0);
    assert.ok(tinyRetreat[0][0] < 1e-3);
});

test('computeVFIAttenuationMatrix remains weakly active away from the boundary', () => {
    const farFromLimit = computeVFIAttenuationMatrix([
        { row: [1], distance: 1, threshold: 0.15, source: 'joint_limit' }
    ], 1);

    assert.ok(farFromLimit[0][0] > 0.8);
    assert.ok(farFromLimit[0][0] < 1);
});

test('computeVFIAttenuationMatrix stays positive semidefinite for repeated non-orthogonal rows', () => {
    const W = computeVFIAttenuationMatrix([
        { row: [1, 0], distance: 0, threshold: 0.15, source: 'joint_limit' },
        { row: [1, 0], distance: 0, threshold: 0.15, source: 'joint_limit' },
        { row: [1, 1], distance: 0.02, threshold: 0.15, source: 'joint_limit' }
    ], 2);

    assert.ok(W[0][0] >= -1e-12);
    assert.ok(W[1][1] >= -1e-12);
    assert.ok(W[0][0] * W[1][1] - W[0][1] * W[1][0] >= -1e-12);
});

test('computeExtendedConditionManipulability reduces to the plain condition score away from constraints', () => {
    const J = [
        [2, 0],
        [0, 1]
    ];

    const plain = computeExtendedConditionManipulability(J, []);
    const far = computeExtendedConditionManipulability(J, [
        { row: [1, 0], distance: 10, threshold: 0.1, source: 'joint_limit' }
    ]);

    assert.ok(Math.abs(plain.value - 0.5) < 1e-12);
    assert.ok(Math.abs(far.value - plain.value) < 0.01);
});

test('computeExtendedConditionManipulability penalizes task directions that reduce a near active margin', () => {
    const J = [
        [1, 0],
        [0, 1]
    ];

    const free = computeExtendedConditionManipulability(J, []);
    const nearUpper = computeExtendedConditionManipulability(J, [
        { row: [-1, 0], distance: 0.001, threshold: 0.1, source: 'joint_limit' }
    ]);

    assert.ok(nearUpper.value < free.value);
    assert.ok(nearUpper.worstGamma.some((value) => value > 0));
});

test('computeExtendedConditionManipulability accepts waist-corridor distance rows as obstacle-like margins', () => {
    const J = [
        [1, 0.2],
        [0.1, 1]
    ];

    const looseCorridor = computeExtendedConditionManipulability(J, [
        { row: [0.8, 0.1], distance: 0.5, threshold: 0.025, source: 'waist_corridor' }
    ]);
    const tightCorridor = computeExtendedConditionManipulability(J, [
        { row: [0.8, 0.1], distance: 0.001, threshold: 0.025, source: 'waist_corridor' }
    ]);

    assert.ok(tightCorridor.value < looseCorridor.value);
});

test('computeTwistJacobian matches the paper H8- right-product extraction', () => {
    const rotationQuaternion = [0.9238795325, 0, 0.3826834324, 0];
    const pose = buildDualQuaternion(rotationQuaternion, [0.2, -0.1, 0.3]);
    const conjugate = dqConjugate(pose);
    const J8 = [
        [0.01, -0.02],
        [0.03, 0.04],
        [-0.05, 0.06],
        [0.07, -0.08],
        [0.02, 0.01],
        [-0.03, 0.05],
        [0.04, -0.06],
        [-0.07, 0.09]
    ];

    const actual = computeTwistJacobian(J8, pose);
    const expected = Array.from({ length: 6 }, () => new Array(J8[0].length).fill(0));
    for (let col = 0; col < J8[0].length; col++) {
        const xDot = J8.map((row) => row[col]);
        const xi = multiplyDualQuaternions(xDot, conjugate).map((value) => 2 * value);
        const twist = [xi[1], xi[2], xi[3], xi[5], xi[6], xi[7]];
        for (let row = 0; row < twist.length; row++) {
            expected[row][col] = twist[row];
        }
    }

    assert.equal(actual.length, expected.length);
    for (let row = 0; row < expected.length; row++) {
        for (let col = 0; col < expected[row].length; col++) {
            assert.ok(Math.abs(actual[row][col] - expected[row][col]) < 1e-12);
        }
    }
});
