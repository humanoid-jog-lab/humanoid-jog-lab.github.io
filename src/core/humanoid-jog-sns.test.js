import test from 'node:test';
import assert from 'node:assert/strict';

import { solveHumanoidJogSNS } from './humanoid-jog-sns.js';

test('SNS follows the task when no bound saturates', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1, 0]], target: [0.03] }],
        hardConstraints: { lowerStep: [-0.1, -0.1], upperStep: [0.1, 0.1] },
        damping: 1e-9
    });

    assert.equal(result.solver, 'sns');
    assert.equal(result.converged, true);
    assert.equal(result.task_scale, 1);
    assert.ok(Math.abs(result.dq[0] - 0.03) < 1e-5);
    assert.deepEqual(result.saturated_joints, []);
});

test('SNS keeps the commanded block primary instead of averaging it with invariants', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [
            { label: 'command', J: [[1, 0]], target: [0.04] },
            { label: 'invariant', J: [[1, 0]], target: [0] }
        ],
        hardConstraints: { lowerStep: [-0.1, -0.1], upperStep: [0.1, 0.1] },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.task_scale, 1);
    assert.ok(Math.abs(result.dq[0] - 0.04) < 1e-5);
    assert.deepEqual(result.saturated_joints, []);
});

test('SNS scales the task when a critical joint saturates', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1, 0]], target: [0.1] }],
        hardConstraints: { lowerStep: [-0.05, -0.05], upperStep: [0.02, 0.05] },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.ok(result.task_scale > 0);
    assert.ok(result.task_scale < 1);
    assert.ok(Math.abs(result.dq[0] - 0.02) < 1e-5);
    assert.deepEqual(result.saturated_joints, [0]);
    assert.equal(result.critical_joint, 0);
});

test('SNS saturates a critical joint and recomputes in the remaining null space', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1, 1]], target: [1] }],
        hardConstraints: { lowerStep: [-1, -1], upperStep: [0.2, 1] },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.task_scale, 1);
    assert.ok(Math.abs(result.dq[0] - 0.2) < 1e-5);
    assert.ok(Math.abs(result.dq[1] - 0.8) < 1e-5);
    assert.ok(result.activeResidualNorm < 1e-5);
    assert.deepEqual(result.saturated_joints, [0]);
});

test('SNS accepts a damped rank-deficient task direction instead of returning zero scale', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1, 0], [1, 0]], target: [0.1, 0.1] }],
        hardConstraints: { lowerStep: [-1, -1], upperStep: [1, 1] },
        damping: 1e-3
    });

    assert.equal(result.converged, true);
    assert.equal(result.task_scale, 1);
    assert.ok(result.dq[0] > 0.09);
    assert.ok(result.activeResidualNorm < 1e-3);
    assert.deepEqual(result.saturated_joints, []);
});

test('SNS keeps a best-effort scaled step when saturated joints leave only a damped partial task direction', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1, 0], [0, 1], [1, 1]], target: [0.1, 0.1, 0.3] }],
        hardConstraints: { lowerStep: [0, -0.05], upperStep: [0, 0.02] },
        damping: 1e-3
    });

    assert.equal(result.converged, true);
    assert.ok(result.task_scale > 0);
    assert.ok(result.dq[1] > 0);
    assert.ok(result.activeResidualNorm < Math.hypot(0.1, 0.1, 0.3));
    assert.ok(result.saturated_joints.includes(0));
});

test('SNS repairs non-box hard rows such as waist-corridor halfspaces after box saturation', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1, 0], [0, 1]], target: [0.01, 0.01] }],
        hardConstraints: {
            lowerStep: [-1, -1],
            upperStep: [1, 1],
            A: [[1, 1]],
            b: [0.005]
        },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.ok(result.maxViolation <= 1e-8);
    assert.ok(result.dq[0] > 0);
    assert.ok(result.dq[1] > 0);
    assert.ok(result.dq[0] + result.dq[1] <= 0.005 + 1e-8);
    assert.ok(result.activeResidualNorm < Math.hypot(0.01, 0.01));
});

test('SNS falls back to a feasible zero step instead of returning a violating general-row step', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1]], target: [0.1] }],
        hardConstraints: { A: [[1]], b: [0] },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.dq[0], 0);
    assert.equal(result.maxViolation, 0);
});

test('SNS applies a corrective hard-row repair when zero is infeasible but a feasible step exists', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1]], target: [0.1] }],
        hardConstraints: {
            lowerStep: [-0.1],
            upperStep: [0.1],
            A: [[1]],
            b: [-0.01]
        },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.ok(result.dq[0] <= -0.01 + 1e-8);
    assert.ok(result.maxViolation <= 1e-8);
});

test('SNS reports infeasible when no corrective hard-row step exists', () => {
    const result = solveHumanoidJogSNS({
        taskBlocks: [{ J: [[1]], target: [0.1] }],
        hardConstraints: {
            lowerStep: [0],
            upperStep: [0],
            A: [[1]],
            b: [-0.01]
        },
        damping: 1e-9
    });

    assert.equal(result.converged, false);
    assert.equal(result.dq[0], 0);
    assert.ok(result.maxViolation > 0);
});
