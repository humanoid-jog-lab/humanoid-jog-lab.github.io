import test from 'node:test';
import assert from 'node:assert/strict';

import { solveHumanoidJogPredictiveIK } from './humanoid-jog-predictive-ik.js';

test('Predictive IK tracks an unconstrained command with full task scale', () => {
    const result = solveHumanoidJogPredictiveIK({
        taskBlocks: [{
            label: 'command',
            J: [[1]],
            target: [0.006]
        }],
        hardConstraints: {
            lowerStep: [-0.1],
            upperStep: [0.1]
        },
        horizonSteps: 3,
        damping: 1e-9
    });

    assert.equal(result.solver, 'predictive_ik');
    assert.equal(result.converged, true);
    assert.ok(Math.abs(result.dq[0] - 0.006) < 1e-5);
    assert.ok(result.task_scale > 0.99);
    assert.ok(result.maxViolation <= 1e-8);
});

test('Predictive IK downscales the task instead of violating a hard step bound', () => {
    const result = solveHumanoidJogPredictiveIK({
        taskBlocks: [{
            label: 'command',
            J: [[1]],
            target: [0.006]
        }],
        hardConstraints: {
            lowerStep: [-0.1],
            upperStep: [0.003]
        },
        horizonSteps: 3,
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.ok(result.dq[0] <= 0.003 + 1e-8);
    assert.ok(result.task_scale < 0.75);
    assert.ok(result.task_scale > 0.45);
    assert.ok(result.maxViolation <= 1e-8);
});

test('Predictive IK keeps invariant rows as lower-priority tracking objectives', () => {
    const result = solveHumanoidJogPredictiveIK({
        taskBlocks: [
            {
                label: 'command',
                J: [[1, 0]],
                target: [0.04]
            },
            {
                label: 'invariant',
                J: [[0, 1]],
                target: [0.02]
            }
        ],
        hardConstraints: {
            lowerStep: [-0.1, -0.1],
            upperStep: [0.1, 0.1]
        },
        horizonSteps: 2,
        damping: 1e-9,
        smoothnessWeight: 0
    });

    assert.equal(result.converged, true);
    assert.ok(Math.abs(result.dq[0] - 0.04) < 1e-5);
    assert.ok(Math.abs(result.dq[1] - 0.02) < 1e-5);
    assert.ok(result.task_scale > 0.99);
});

test('Predictive IK remains well-conditioned when some command target rows are zero', () => {
    const n = 18;
    const J = Array.from({ length: 8 }, (_, r) => {
        return Array.from({ length: n }, (_, c) => ((r + c) % 5 === 0 ? 1 : 0.2));
    });
    const result = solveHumanoidJogPredictiveIK({
        taskBlocks: [{
            label: 'command',
            J,
            target: [0, 0, 0, 0, 3e-4, 2e-4, 1e-4, 1e-4]
        }],
        hardConstraints: {
            lowerStep: Array(n).fill(-0.03),
            upperStep: Array(n).fill(0.03)
        },
        horizonSteps: 3,
        damping: 0
    });

    assert.equal(result.qp_status, 'solved');
    assert.doesNotMatch(result.qp_message || '', /positive definite/i);
    assert.ok(result.task_scale > 0.1);
    assert.ok(result.stepNorm > 0);
});

test('Predictive IK constrains every task scale variable to [0, 1]', () => {
    const result = solveHumanoidJogPredictiveIK({
        taskBlocks: [{
            label: 'command',
            J: [[1]],
            target: [0.004]
        }],
        hardConstraints: {
            lowerStep: [-0.1],
            upperStep: [0.1]
        },
        horizonSteps: 3,
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.ok(result.scaleSequence.every((value) => value >= -1e-8 && value <= 1 + 1e-8));
    assert.equal(result.scale_bound_rows, 2);
});

test('Predictive IK tracks secondary rows across the full horizon', () => {
    const result = solveHumanoidJogPredictiveIK({
        taskBlocks: [
            {
                label: 'command',
                J: [[1, 0]],
                target: [0.02]
            },
            {
                label: 'invariant',
                J: [[0, 1]],
                target: [0.01]
            }
        ],
        hardConstraints: {
            lowerStep: [-0.1, -0.1],
            upperStep: [0.1, 0.1]
        },
        horizonSteps: 4,
        damping: 1e-9,
        smoothnessWeight: 0
    });

    assert.equal(result.converged, true);
    assert.equal(result.secondary_horizon_rows, 4);
    assert.ok(Math.abs(result.dq[1] - 0.01) < 1e-5);
});
