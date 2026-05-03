import test from 'node:test';
import assert from 'node:assert/strict';

import { solveHumanoidJogKinematicWBCQP } from './humanoid-jog-wbc-qp.js';

test('Kinematic WBC-QP tracks a toy task inside hard bounds', () => {
    const result = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1, 0]], target: [0.04] }],
        hardConstraints: { lowerStep: [-0.1, -0.1], upperStep: [0.1, 0.1] },
        damping: 1e-9
    });

    assert.equal(result.solver, 'wbc_qp');
    assert.equal(result.converged, true);
    assert.equal(result.qp_status, 'solved');
    assert.ok(Math.abs(result.dq[0] - 0.04) < 1e-5);
    assert.ok(result.tracking_norm < 1e-4);
    assert.ok(result.constraint_violation <= 1e-8);
});

test('Kinematic WBC-QP obeys hard constraints when tracking asks too much', () => {
    const result = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1]], target: [0.5] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.03] },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.ok(result.dq[0] <= 0.03 + 1e-8);
    assert.ok(result.tracking_norm > 0.4);
    assert.ok(result.constraint_violation <= 1e-8);
});

test('Kinematic WBC-QP exposes lightweight velocity state for the next tick', () => {
    const first = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1]], target: [0.05] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.1] },
        damping: 1e-9
    });
    const second = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1]], target: [0.05] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.1] },
        damping: 1e-9,
        state: first.nextState,
        gains: { kp: 1, kd: 0.5 }
    });

    assert.ok(Array.isArray(first.nextState.dqLast));
    assert.ok(Array.isArray(first.nextState.taskVelocity));
    assert.ok(Math.abs(second.dq[0]) < Math.abs(first.dq[0]));
});

test('Kinematic WBC-QP uses robust task integral feedback from the desired task velocity', () => {
    const first = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1]], target: [0.06] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.1] },
        damping: 1e-9,
        gains: { kp: 1, kd: 0, ki: 0 }
    });
    const second = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1]], target: [0.06] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.1] },
        damping: 1e-9,
        state: first.nextState,
        gains: { kp: 1, kd: 0, ki: 0.5 }
    });

    assert.ok(Math.abs(first.dq[0] - 0.06) < 1e-5);
    assert.ok(Math.abs(second.dq[0] - 0.03) < 1e-5);
    assert.ok(Math.abs(second.robust_task_integral_norm - 0.03) < 1e-5);
});

test('Kinematic WBC-QP tightens active inequality rows with robust constraint integral feedback', () => {
    const first = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1]], target: [0.2] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.05] },
        damping: 1e-9,
        gains: { kp: 1, kd: 0, ki: 0, constraintKi: 0 }
    });
    const second = solveHumanoidJogKinematicWBCQP({
        taskBlocks: [{ J: [[1]], target: [0.2] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.05] },
        damping: 1e-9,
        state: first.nextState,
        gains: { kp: 1, kd: 0, ki: 0, constraintKi: 0.5 }
    });

    assert.ok(Math.abs(first.dq[0] - 0.05) < 1e-5);
    assert.ok(second.dq[0] <= 0.025 + 1e-5);
    assert.ok(Math.abs(second.robust_constraint_integral_norm - 0.025) < 1e-5);
    assert.ok(second.constraint_violation <= 1e-8);
});
