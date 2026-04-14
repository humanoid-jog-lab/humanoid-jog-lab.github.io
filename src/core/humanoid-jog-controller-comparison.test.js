import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidControllerComparisonTasks,
    evaluateHumanoidControllerComparisonRelativePoseChange,
    evaluateHumanoidControllerComparisonCompletion,
    evaluateHumanoidControllerComparisonStep,
    resolveHumanoidControllerComparisonControllerOptions,
    getHumanoidControllerComparisonColumns,
    summarizeHumanoidControllerComparisonRows
} from './humanoid-jog-controller-comparison.js';

test('buildHumanoidControllerComparisonTasks returns the 16 controller comparison scenarios', () => {
    const tasks = buildHumanoidControllerComparisonTasks();
    assert.equal(tasks.length, 16);
    assert.deepEqual(
        tasks.slice(0, 4).map((task) => task.scenarioLabel),
        [
            'absolute | nominal | new qpJ',
            'absolute | nominal | 零空间投影',
            'absolute | nominal | Ours',
            'absolute | nominal | 恢复优化(TR)'
        ]
    );
    assert.deepEqual(
        tasks.slice(-4).map((task) => task.scenarioLabel),
        [
            'decoupled-upright | near-limit | new qpJ',
            'decoupled-upright | near-limit | 零空间投影',
            'decoupled-upright | near-limit | Ours',
            'decoupled-upright | near-limit | 恢复优化(TR)'
        ]
    );
});

test('summarizeHumanoidControllerComparisonRows reports completion rate, completion ticks, and mean solve time by controller', () => {
    const rows = summarizeHumanoidControllerComparisonRows([
        { mode: 'absolute', profile: 'nominal', controllerKey: 'qp', completed: true, completionTicks: 11, meanSolveMs: 2.35 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'qp', completed: false, completionTicks: Number.NaN, meanSolveMs: 3.85 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'nullspace', completed: false, completionTicks: Number.NaN, meanSolveMs: 5.5 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'ours', completed: true, completionTicks: 27, meanSolveMs: 7.1 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'restore_tr', completed: true, completionTicks: 17, meanSolveMs: 4.2 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'restore_tr', completed: true, completionTicks: 19, meanSolveMs: 5.0 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'qp', completed: false, completionTicks: Number.NaN, meanSolveMs: 2.8 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'nullspace', completed: false, completionTicks: Number.NaN, meanSolveMs: 4.6 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'ours', completed: true, completionTicks: 14, meanSolveMs: 8.2 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'restore_tr', completed: true, completionTicks: 18, meanSolveMs: 6.4 }
    ]);

    assert.deepEqual(rows, [
        {
            mode: 'absolute',
            start_posture: 'nominal',
            qp: '50.0% (11, 3.1 ms)',
            nullspace: '0.0% (n/a, 5.5 ms)',
            ours: '100.0% (27, 7.1 ms)',
            restore_tr: '100.0% (18, 4.6 ms)'
        },
        {
            mode: 'decoupled-upright',
            start_posture: 'near-limit',
            qp: '0.0% (n/a, 2.8 ms)',
            nullspace: '0.0% (n/a, 4.6 ms)',
            ours: '100.0% (14, 8.2 ms)',
            restore_tr: '100.0% (18, 6.4 ms)'
        }
    ]);
});

test('getHumanoidControllerComparisonColumns exposes the table schema for the paper benchmark', () => {
    const columns = getHumanoidControllerComparisonColumns();
    assert.deepEqual(
        columns.map((column) => column.label),
        ['Mode', 'Start posture', 'new qpJ', 'Null-space', 'Ours', '恢复优化(TR)']
    );
});

test('resolveHumanoidControllerComparisonControllerOptions relaxes QP benchmark tolerance and keeps it alive through the tick budget', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'qp',
        mode: 'decoupled-upright',
        includeWaist: true,
        maxTicks: 60,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.commandTolerance, 2e-3);
    assert.equal(result.maxStallTicks, 60);
    assert.equal(result.maxTicks, 60);
    assert.equal(result.targetXMeters, 0.30);
    assert.equal(result.jointLimitAvoidanceEnabled, true);
    assert.equal(result.qpUseInvariantEquality, true);
});

test('resolveHumanoidControllerComparisonControllerOptions disables joint-limit avoidance for null-space', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'nullspace',
        mode: 'decoupled-upright',
        includeWaist: true,
        maxTicks: 60,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.commandTolerance, 5e-4);
    assert.equal(result.maxStallTicks, 3);
    assert.equal(result.maxTicks, 60);
    assert.equal(result.targetXMeters, 0.30);
    assert.equal(result.jointLimitAvoidanceEnabled, false);
});

test('resolveHumanoidControllerComparisonControllerOptions shortens the arm-only absolute benchmark and loosens the QP absolute setup', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'qp',
        mode: 'absolute',
        includeWaist: false,
        maxTicks: 60,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.maxTicks, 15);
    assert.equal(result.maxStallTicks, 15);
    assert.equal(result.targetXMeters, 0.075);
    assert.equal(result.commandTolerance, 2e-3);
    assert.equal(result.jointLimitAvoidanceEnabled, false);
    assert.equal(result.qpUseInvariantEquality, false);
});

test('evaluateHumanoidControllerComparisonCompletion requires target reach, command convergence, body convergence, and corridor feasibility', () => {
    const result = evaluateHumanoidControllerComparisonCompletion({
        xProgress: 0.301,
        targetXMeters: 0.30,
        commandError: 4.5e-4,
        commandTolerance: 5e-4,
        bodyError: 4.0e-5,
        bodyTolerance: 5e-5,
        waistXViolation: 0
    });

    assert.equal(result.reachedTarget, true);
    assert.equal(result.commandConverged, true);
    assert.equal(result.bodyConverged, true);
    assert.equal(result.waistFeasible, true);
    assert.equal(result.completed, true);
    assert.equal(result.progressRatio, 1.0);
});

test('evaluateHumanoidControllerComparisonCompletion rejects target reach when command error is still too large', () => {
    const result = evaluateHumanoidControllerComparisonCompletion({
        xProgress: 0.305,
        targetXMeters: 0.30,
        commandError: 1.2e-3,
        commandTolerance: 5e-4,
        bodyError: 0,
        bodyTolerance: 5e-5,
        waistXViolation: 0
    });

    assert.equal(result.reachedTarget, true);
    assert.equal(result.commandConverged, false);
    assert.equal(result.completed, false);
    assert.equal(result.progressRatio, 1.0);
});

test('evaluateHumanoidControllerComparisonCompletion treats tiny sub-millimeter target shortfall as reached for benchmark purposes', () => {
    const result = evaluateHumanoidControllerComparisonCompletion({
        xProgress: 0.2998,
        targetXMeters: 0.30,
        commandError: 1e-6,
        commandTolerance: 5e-4,
        bodyError: 4e-5,
        bodyTolerance: 5e-5,
        waistXViolation: 0
    });

    assert.equal(result.reachedTarget, true);
    assert.equal(result.completed, true);
});

test('evaluateHumanoidControllerComparisonRelativePoseChange measures signed relative-pose drift with the provided norm', () => {
    const result = evaluateHumanoidControllerComparisonRelativePoseChange({
        initialRelativePose: [1, 2],
        currentRelativePose: [4, 6],
        signedPoseError: (target, current) => current.map((value, index) => value - target[index]),
        poseErrorNorm: (error) => Math.sqrt(error.reduce((sum, value) => sum + value * value, 0))
    });

    assert.equal(result, 5);
});

test('evaluateHumanoidControllerComparisonRelativePoseChange returns NaN when the relative pose samples are unavailable', () => {
    const result = evaluateHumanoidControllerComparisonRelativePoseChange({
        initialRelativePose: null,
        currentRelativePose: [4, 6],
        signedPoseError: () => [0, 0],
        poseErrorNorm: () => 0
    });

    assert.equal(Number.isNaN(result), true);
});

test('evaluateHumanoidControllerComparisonStep keeps progressing max_iter steps alive', () => {
    const result = evaluateHumanoidControllerComparisonStep({
        stepProgress: 4.9e-3,
        minEffectiveProgress: 1e-5,
        stallTicks: 0,
        maxStallTicks: 3,
        status: {
            converged: false,
            reason: 'max_iter'
        }
    });

    assert.equal(result.progressed, true);
    assert.equal(result.ok, true);
    assert.equal(result.shouldStop, false);
    assert.equal(result.nextStallTicks, 0);
});
