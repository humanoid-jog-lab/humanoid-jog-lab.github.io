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

test('evaluateHumanoidJogCommandOutcome keeps a single-step result when it makes clear progress even without 80% reduction', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'single_step',
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

    assert.equal(outcome.reason, 'single_step');
    assert.equal(outcome.applyResult, true);
    assert.equal(outcome.converged, true);
});

test('evaluateHumanoidJogCommandOutcome rejects a single-step result when it makes almost no progress', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'single_step',
        initialCommandNorm: 0.1,
        finalCommandNorm: 0.0995,
        finalInvariantNorm: 1e-5,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-3,
        mismatchRequiredReduction: 0.8,
        singleStepMismatchRequiredReduction: 0.05
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

test('evaluateHumanoidJogCommandOutcome upgrades best-effort progress to practical convergence when practical tolerances are satisfied', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'best_effort',
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

test('evaluateHumanoidJogCommandOutcome upgrades time-budget best-effort progress when tolerances are satisfied', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'best_effort_time_budget',
        initialCommandNorm: 0.02,
        finalCommandNorm: 5.9e-8,
        finalInvariantNorm: 1.56e-6,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 5e-5,
        mismatchTol: 1e-3,
        mismatchRequiredReduction: 0.8
    });

    assert.equal(outcome.reason, 'converged');
    assert.equal(outcome.applyResult, true);
    assert.equal(outcome.converged, true);
});

test('evaluateHumanoidJogCommandOutcome upgrades polish-best-effort progress to practical convergence when practical tolerances are satisfied', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'polish_best_effort',
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

test('evaluateHumanoidJogCommandOutcome treats polish-converged as converged', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'polish_converged',
        initialCommandNorm: 0.02,
        finalCommandNorm: 8e-7,
        finalInvariantNorm: 2e-6,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-3,
        mismatchRequiredReduction: 0.8
    });

    assert.equal(outcome.reason, 'converged');
    assert.equal(outcome.applyResult, true);
    assert.equal(outcome.converged, true);
});

test('evaluateHumanoidJogCommandOutcome accepts secondary-limited restoration when the task residuals are satisfied', () => {
    const result = resolveHumanoidJogCommandResult({
        stopReason: 'secondary_limited',
        initialCommandNorm: 0.334,
        finalCommandNorm: 3.21e-9,
        finalInvariantNorm: 2.45e-8,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-12,
        mismatchRequiredReduction: 0.8,
        q: [1, 2, 3],
        qInitial: [4, 5, 6]
    });

    assert.equal(result.outcome.reason, 'converged');
    assert.equal(result.outcome.applyResult, true);
    assert.equal(result.outcome.converged, true);
    assert.deepEqual(result.qApplied, [1, 2, 3]);
});

test('evaluateHumanoidJogCommandOutcome accepts safety-limited restoration when the task residuals are satisfied', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'safety_limited',
        initialCommandNorm: 0.01,
        finalCommandNorm: 5e-7,
        finalInvariantNorm: 1e-6,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-12,
        mismatchRequiredReduction: 0.8
    });

    assert.equal(outcome.reason, 'converged');
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

test('resolveHumanoidJogCommandResult keeps best-effort progress instead of rolling back to qInitial', () => {
    const result = resolveHumanoidJogCommandResult({
        stopReason: 'best_effort',
        initialCommandNorm: 0.002,
        finalCommandNorm: 0.001,
        finalInvariantNorm: 0,
        hardCommandTol: 1e-6,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-12,
        mismatchRequiredReduction: 0.8,
        singleStepMismatchRequiredReduction: 0.05,
        q: [1, 2, 3],
        qInitial: [4, 5, 6],
        messageMap: {
            best_effort: '保留最后可行小步。'
        }
    });

    assert.equal(result.outcome.reason, 'best_effort');
    assert.equal(result.outcome.applyResult, true);
    assert.equal(result.outcome.converged, false);
    assert.deepEqual(result.qApplied, [1, 2, 3]);
    assert.equal(result.message, '保留最后可行小步。');
});

test('evaluateHumanoidJogCommandOutcome keeps best-effort progress even when the reduction is smaller than the single-step mismatch threshold', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'best_effort',
        initialCommandNorm: 0.0021,
        finalCommandNorm: 0.0020,
        finalInvariantNorm: 0,
        hardCommandTol: 1e-14,
        hardInvariantTol: 5e-5,
        practicalCommandTol: 1e-4,
        practicalInvariantTol: 2.5e-4,
        mismatchTol: 1e-12,
        mismatchRequiredReduction: 0.8,
        singleStepMismatchRequiredReduction: 0.05
    });

    assert.equal(outcome.reason, 'best_effort');
    assert.equal(outcome.applyResult, true);
    assert.equal(outcome.converged, false);
});

test('evaluateHumanoidJogCommandOutcome can require hard convergence before applying a best-effort SQP result', () => {
    const outcome = evaluateHumanoidJogCommandOutcome({
        stopReason: 'best_effort',
        initialCommandNorm: 0.0021,
        finalCommandNorm: 9e-4,
        finalInvariantNorm: 2e-8,
        hardCommandTol: 1e-12,
        hardInvariantTol: 1e-12,
        practicalCommandTol: 1e-12,
        practicalInvariantTol: 1e-12,
        mismatchTol: 1e-12,
        mismatchRequiredReduction: 0.8,
        singleStepMismatchRequiredReduction: 0.05,
        requireHardConvergenceToApply: true
    });

    assert.equal(outcome.reason, 'command_mismatch');
    assert.equal(outcome.applyResult, false);
    assert.equal(outcome.converged, false);
});
