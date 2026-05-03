import test from 'node:test';
import assert from 'node:assert/strict';

import { solveHumanoidJogHQP } from './humanoid-jog-hqp.js';

test('HQP solves a primary equality-like tracking level', () => {
    const result = solveHumanoidJogHQP({
        taskBlocks: [{
            label: 'cmd',
            J: [[1, 0]],
            target: [0.05]
        }],
        hardConstraints: { lowerStep: [-0.1, -0.1], upperStep: [0.1, 0.1] },
        damping: 1e-9
    });

    assert.equal(result.solver, 'hqp');
    assert.equal(result.converged, true);
    assert.ok(Math.abs(result.dq[0] - 0.05) < 1e-5);
    assert.ok(Math.abs(result.dq[1]) < 1e-6);
    assert.equal(result.hqp_levels, 1);
    assert.ok(result.active_rows >= 4);
});

test('HQP preserves a higher-priority level while solving a lower-priority level', () => {
    const result = solveHumanoidJogHQP({
        levels: [
            { label: 'hold', taskBlocks: [{ J: [[1, 1]], target: [0.02] }] },
            { label: 'shape', taskBlocks: [{ J: [[0, 1]], target: [0.05] }] }
        ],
        hardConstraints: { lowerStep: [-0.2, -0.2], upperStep: [0.2, 0.2] },
        damping: 1e-9,
        equalityTolerance: 1e-6
    });

    assert.equal(result.converged, true);
    assert.ok(Math.abs(result.dq[0] + result.dq[1] - 0.02) < 2e-5);
    assert.ok(result.dq[1] > 0.015);
    assert.equal(result.stages.length, 2);
});

test('HQP keeps the higher-priority step when a lower-priority solve violates protected rows', () => {
    let calls = 0;
    const result = solveHumanoidJogHQP({
        levels: [
            { label: 'command', taskBlocks: [{ J: [[1]], target: [0.04] }] },
            { label: 'shape', taskBlocks: [{ J: [[1]], target: [0] }] }
        ],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.1] },
        damping: 1e-9,
        equalityTolerance: 1e-8,
        feasibleAcceptanceTolerance: 1e-8,
        qpOptions: {
            disableFallback: true,
            solutionTolerance: 1e-10,
            solveQPBackend: () => {
                calls += 1;
                return {
                    solution: calls === 1 ? [0.04] : [0],
                    message: '',
                    iterations: [1, 0]
                };
            }
        }
    });

    assert.equal(result.converged, true);
    assert.match(result.qp_message, /lower_priority_limited/);
    assert.ok(Math.abs(result.dq[0] - 0.04) < 1e-8);
    assert.ok(result.slack_norm <= 1e-8);
    assert.equal(result.stages.length, 2);
    assert.equal(result.stages[1].qpConverged, false);
});

test('HQP respects hard step bounds instead of silently violating them', () => {
    const result = solveHumanoidJogHQP({
        taskBlocks: [{ J: [[1]], target: [1] }],
        hardConstraints: { lowerStep: [-0.1], upperStep: [0.2] },
        damping: 1e-9
    });

    assert.equal(result.converged, true);
    assert.ok(result.dq[0] <= 0.2 + 1e-8);
    assert.ok(result.activeResidualNorm > 0.7);
    assert.ok(result.maxViolation <= 1e-8);
});

test('HQP accepts a numerically feasible active-set result within controller feasibility tolerance', () => {
    const result = solveHumanoidJogHQP({
        taskBlocks: [{ J: [[1]], target: [0.01] }],
        hardConstraints: {
            A: [[1], [-1]],
            b: [0, -5e-9]
        },
        damping: 1e-9,
        tolerance: 1e-12,
        feasibleAcceptanceTolerance: 1e-8,
        qpOptions: {
            solutionTolerance: 1e-12,
            solveQPBackend: () => ({
                solution: [2.5e-9],
                message: 'iteration limit',
                iterations: [8, 0]
            })
        }
    });

    assert.equal(result.converged, true);
    assert.ok(result.maxViolation <= 1e-8);
    assert.match(result.qp_message, /accepted_feasible_warning/);
});
