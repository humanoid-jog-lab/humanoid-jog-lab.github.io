import test from 'node:test';
import assert from 'node:assert/strict';

import {
    appendBandConstraints,
    appendEqualityConstraints,
    appendVFIErrorTubeConstraints,
    appendComponentBounds,
    appendJointLimitStepConstraints,
    projectOntoHalfspaces,
    solveQPProjectedGradient,
    solveQPNumericActiveSet
} from './vfi-utils.js';

function assertNear(actual, expected, eps = 1e-12) {
    assert.ok(Math.abs(actual - expected) <= eps, `expected ${actual} ~= ${expected}`);
}

test('appendBandConstraints expands equality-band constraints to halfspaces', () => {
    const A = [];
    const b = [];
    const J = [
        [1, 2],
        [3, 4]
    ];
    const target = [0.5, -0.25];
    const band = [0.1, 0.2];

    appendBandConstraints(A, b, J, target, band);

    assert.equal(A.length, 4);
    assert.equal(b.length, 4);

    assert.deepEqual(A[0], [1, 2]);
    assert.equal(b[0], 0.6);
    assert.deepEqual(A[1], [-1, -2]);
    assert.equal(b[1], -0.4);

    assert.deepEqual(A[2], [3, 4]);
    assertNear(b[2], -0.05);
    assert.deepEqual(A[3], [-3, -4]);
    assertNear(b[3], 0.45);
});

test('appendVFIErrorTubeConstraints builds velocity-damper-style tube inequalities', () => {
    const A = [];
    const b = [];
    const J = [
        [1, 0],
        [0, 1]
    ];
    const error = [0.2, -0.1];

    appendVFIErrorTubeConstraints(A, b, J, error, {
        eta: 2,
        marginBase: 0.05,
        marginGain: 0
    });

    // row0:  [1,0] dq <= 2*(0.05-0.2) = -0.3
    // row1: [-1,0] dq <= 2*(0.05+0.2) =  0.5
    // row2:  [0,1] dq <= 2*(0.05-(-0.1)) = 0.3
    // row3: [0,-1] dq <= 2*(0.05+(-0.1)) = -0.1
    assert.equal(A.length, 4);
    assert.equal(b.length, 4);
    assert.deepEqual(A[0], [1, 0]);
    assertNear(b[0], -0.3);
    assertNear(A[1][0], -1);
    assertNear(A[1][1], 0);
    assertNear(b[1], 0.5);
    assert.deepEqual(A[2], [0, 1]);
    assertNear(b[2], 0.3);
    assertNear(A[3][0], 0);
    assertNear(A[3][1], -1);
    assertNear(b[3], -0.1);
});

test('appendEqualityConstraints encodes equalities as two-sided inequalities', () => {
    const A = [];
    const b = [];
    const W2 = [
        [1, 2],
        [0, 1]
    ];
    const w2 = [3, -1];

    appendEqualityConstraints(A, b, W2, w2, { tolerance: 0.01 });

    assert.equal(A.length, 4);
    assert.equal(b.length, 4);
    assert.deepEqual(A[0], [1, 2]);
    assertNear(b[0], 3.01);
    assert.deepEqual(A[1], [-1, -2]);
    assertNear(b[1], -2.99);
    assert.deepEqual(A[2], [0, 1]);
    assertNear(b[2], -0.99);
    assertNear(A[3][0], 0);
    assertNear(A[3][1], -1);
    assertNear(b[3], 1.01);
});

test('appendJointLimitStepConstraints applies distance-aware upper/lower step caps', () => {
    const A = [];
    const b = [];
    const q = [0.5, 0.95];
    const limits = [
        { lower: 0, upper: 1 },
        { lower: 0, upper: 1 }
    ];

    appendJointLimitStepConstraints(A, b, q, limits, { eta: 1.0, maxStep: 0.1 });

    assert.equal(A.length, 4);
    assert.equal(b.length, 4);
    assert.deepEqual(A[0], [1, 0]);
    assert.equal(b[0], 0.1);
    assert.deepEqual(A[1], [-1, 0]);
    assert.equal(b[1], 0.1);
    assert.deepEqual(A[2], [0, 1]);
    assertNear(b[2], 0.05);
    assert.deepEqual(A[3], [0, -1]);
    assert.equal(b[3], 0.1);
});

test('appendComponentBounds adds symmetric box constraints', () => {
    const A = [];
    const b = [];

    appendComponentBounds(A, b, 2, -0.5, 0.25);

    assert.equal(A.length, 4);
    assert.equal(b.length, 4);
    assert.deepEqual(A[0], [1, 0]);
    assert.equal(b[0], 0.25);
    assert.deepEqual(A[1], [-1, 0]);
    assert.equal(b[1], 0.5);
    assert.deepEqual(A[2], [0, 1]);
    assert.equal(b[2], 0.25);
    assert.deepEqual(A[3], [0, -1]);
    assert.equal(b[3], 0.5);
});

test('projectOntoHalfspaces projects infeasible points to satisfy all linear inequalities', () => {
    const A = [
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1]
    ];
    const b = [1, 1, 1, 1];
    const x0 = [2, -3];

    const projected = projectOntoHalfspaces(x0, A, b, { passes: 12 });

    for (let i = 0; i < A.length; i++) {
        const lhs = A[i][0] * projected[0] + A[i][1] * projected[1];
        assert.ok(lhs <= b[i] + 1e-9);
    }
    assert.ok(Math.abs(projected[0] - 1) < 1e-6);
    assert.ok(Math.abs(projected[1] + 1) < 1e-6);
});

test('solveQPProjectedGradient handles unconstrained quadratic minimization', () => {
    // min 0.5 * x^T (2I) x + [-2, 4]^T x => x* = [1, -2]
    const H = [
        [2, 0],
        [0, 2]
    ];
    const f = [-2, 4];

    const result = solveQPProjectedGradient(H, f, [], [], [0, 0], {
        maxIter: 120,
        stepTol: 1e-10,
        gradTol: 1e-10
    });

    assert.ok(result.converged);
    assert.ok(Math.abs(result.x[0] - 1) < 1e-6);
    assert.ok(Math.abs(result.x[1] + 2) < 1e-6);
});

test('solveQPProjectedGradient respects inequality constraints', () => {
    // min (x-2)^2 -> 0.5*(2)x^2 + (-4)x, with constraint x <= 0.5
    const H = [[2]];
    const f = [-4];
    const A = [[1]];
    const b = [0.5];

    const result = solveQPProjectedGradient(H, f, A, b, [0], {
        maxIter: 100,
        stepTol: 1e-10,
        gradTol: 1e-10,
        projectionPasses: 8
    });

    assert.ok(result.x[0] <= 0.5 + 1e-9);
    assert.ok(Math.abs(result.x[0] - 0.5) < 1e-6);
});

test('solveQPNumericActiveSet respects inequality constraints', () => {
    // min (x-2)^2 + y^2, with x <= 0.5
    const H = [
        [2, 0],
        [0, 2]
    ];
    const f = [-4, 0];
    const A = [[1, 0]];
    const b = [0.5];

    const result = solveQPNumericActiveSet(H, f, A, b, [0, 0]);

    assert.ok(result.converged, `numeric QP should converge, msg=${result.solverMessage}`);
    assert.ok(result.x[0] <= 0.5 + 1e-9);
    assert.ok(Math.abs(result.x[0] - 0.5) < 1e-9);
    assert.ok(Math.abs(result.x[1]) < 1e-9);
});
