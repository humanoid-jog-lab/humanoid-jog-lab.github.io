import test from 'node:test';
import assert from 'node:assert/strict';

import { evaluateHumanoidJogCommandOutcome, resolveHumanoidJogCommandResult } from './humanoid-jog-outcome.js';

test('evaluateHumanoidJogCommandOutcome rejects large command mismatch without enough reduction', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'max_iter',
        initialCommandNorm: 0.1,
        finalCommandNorm: 0.08,
        finalInvariantNorm: 1e-5,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-3,
        mismatchRequiredReduction: 0.8
    });

    assert.equal(outcome.reason, 'command_mismatch');
    assert.equal(outcome.applyResult, false);
    assert.equal(outcome.converged, false);
});

test('evaluateHumanoidJogCommandOutcome accepts practical convergence before hard convergence', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'max_iter',
        initialCommandNorm: 0.02,
        finalCommandNorm: 8e-5,
        finalInvariantNorm: 2e-4,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-3,
        mismatchRequiredReduction: 0.8
    });

    assert.equal(outcome.reason, 'practical_converged');
    assert.equal(outcome.applyResult, true);
    assert.equal(outcome.converged, true);
});

test('evaluateHumanoidJogCommandOutcome keeps partial progress when mismatch tolerance is satisfied', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'max_iter',
        initialCommandNorm: 0.02,
        finalCommandNorm: 7e-4,
        finalInvariantNorm: 1e-4,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-3,
        mismatchRequiredReduction: 0.8
    });

    assert.equal(outcome.reason, 'max_iter');
    assert.equal(outcome.applyResult, true);
    assert.equal(outcome.converged, false);
});

test('evaluateHumanoidJogCommandOutcome does not apply solver output after a Jacobian failure', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'jacobian_fail',
        initialCommandNorm: 0.02,
        finalCommandNorm: 5e-4,
        finalInvariantNorm: 1e-4
    });

    assert.equal(outcome.reason, 'jacobian_fail');
    assert.equal(outcome.applyResult, false);
    assert.equal(outcome.converged, false);
});

test('resolveHumanoidJogCommandResult falls back to the initial joints on command mismatch', () => {
    const result = resolveHumanoidJogCommandResult({
        stopReason: 'max_iter',
        initialCommandNorm: 0.1,
        finalCommandNorm: 0.08,
        finalInvariantNorm: 1e-5,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-3,
        mismatchRequiredReduction: 0.8,
        q: [1, 2, 3],
        qInitial: [4, 5, 6],
        messageMap: {
            command_mismatch: '无法到达该位置（目标未达成）。'
        }
    });

    assert.equal(result.outcome.reason, 'command_mismatch');
    assert.deepEqual(result.qApplied, [4, 5, 6]);
    assert.equal(result.message, '无法到达该位置（目标未达成）。');
});
