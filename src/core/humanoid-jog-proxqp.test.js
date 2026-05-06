import test from 'node:test';
import assert from 'node:assert/strict';

import { solveHumanoidJogProxQP } from './humanoid-jog-proxqp.js';

test('ProxQP tracks an unconstrained kinematic jog target', () => {
    const result = solveHumanoidJogProxQP({
        taskBlocks: [{
            label: 'command',
            J: [[1, 0]],
            target: [0.05]
        }],
        hardConstraints: { lowerStep: [-0.1, -0.1], upperStep: [0.1, 0.1] },
        damping: 1e-9,
        tolerance: 1e-7
    });

    assert.equal(result.solver, 'proxqp');
    assert.equal(result.converged, true);
    assert.ok(Math.abs(result.dq[0] - 0.05) < 1e-5);
    assert.ok(Math.abs(result.dq[1]) < 1e-6);
    assert.ok(result.maxViolation <= 1e-8);
    assert.ok(Number.isFinite(result.proxqp_primal_residual));
    assert.ok(Number.isFinite(result.proxqp_dual_residual));
});

test('ProxQP respects coupled halfspace and component step constraints', () => {
    const result = solveHumanoidJogProxQP({
        taskBlocks: [{
            label: 'command',
            J: [[1, 1]],
            target: [0.2]
        }],
        hardConstraints: {
            A: [[1, 1]],
            b: [0.06],
            lowerStep: [-0.1, -0.1],
            upperStep: [0.1, 0.1]
        },
        damping: 1e-8,
        tolerance: 1e-7
    });

    assert.equal(result.converged, true);
    assert.ok(result.dq[0] + result.dq[1] <= 0.06001);
    assert.ok(result.dq[0] <= 0.1 + 1e-8);
    assert.ok(result.dq[1] <= 0.1 + 1e-8);
    assert.ok(result.maxViolation <= 1e-7);
    assert.ok(result.activeResidualNorm > 0.1);
});

test('ProxQP returns finite best effort diagnostics for inconsistent constraints', () => {
    const result = solveHumanoidJogProxQP({
        taskBlocks: [{ label: 'command', J: [[1]], target: [0.04] }],
        hardConstraints: {
            A: [[1], [-1]],
            b: [0, -0.02]
        },
        damping: 1e-8,
        tolerance: 1e-9,
        outerMaxIterations: 4,
        innerMaxIterations: 4
    });

    assert.equal(result.solver, 'proxqp');
    assert.equal(result.converged, false);
    assert.equal(result.reason, 'constraint_violation');
    assert.ok(Array.isArray(result.dq));
    assert.ok(result.dq.every(Number.isFinite));
    assert.ok(result.maxViolation > 0);
    assert.ok(result.qp_message.includes('best_effort'));
});

test('ProxQP applies a caller-provided linear secondary objective', () => {
    const result = solveHumanoidJogProxQP({
        qActive: [0, 0],
        taskBlocks: [{
            label: 'hold-primary',
            J: [[1, 0]],
            target: [0],
            weight: 1
        }],
        hardConstraints: {
            A: [
                [1, 0],
                [-1, 0],
                [0, 1],
                [0, -1]
            ],
            b: [0.2, 0.2, 0.2, 0.2]
        },
        damping: 1e-3,
        linearObjective: [0, -1],
        tolerance: 1e-8,
        feasibleAcceptanceTolerance: 1e-8,
        outerMaxIterations: 12,
        innerMaxIterations: 16
    });

    assert.equal(result.solver, 'proxqp');
    assert.ok(Math.abs(result.dq[0]) < 1e-5);
    assert.ok(result.dq[1] > 0.05);
});
