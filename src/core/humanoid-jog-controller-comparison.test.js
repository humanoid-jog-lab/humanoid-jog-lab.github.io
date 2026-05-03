import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidControllerComparisonTasks,
    evaluateHumanoidControllerComparisonRelativePoseChange,
    evaluateHumanoidControllerComparisonCompletion,
    evaluateHumanoidControllerComparisonStep,
    resolveHumanoidControllerComparisonControllerOptions,
    resolveHumanoidControllerComparisonDefaultMaxTicks,
    resolveHumanoidControllerComparisonSpeedPercent,
    resolveHumanoidControllerComparisonTaskValidTolerance,
    getHumanoidControllerComparisonColumns,
    summarizeHumanoidControllerComparisonRows
} from './humanoid-jog-controller-comparison.js';

test('buildHumanoidControllerComparisonTasks returns the 9 three-controller comparison scenarios', () => {
    const tasks = buildHumanoidControllerComparisonTasks();
    assert.equal(tasks.length, 9);
    assert.equal(tasks.some((task) => task.solverMode === 'mj_restore_tr'), true);
    assert.equal(tasks.some((task) => task.solverMode === 'mj_restore_tr_w'), true);
    assert.equal(tasks.some((task) => task.solverMode === 'mj_restore_tr_safe_wvfi'), true);
    assert.equal(tasks.some((task) => task.solverMode === 'proxqp'), false);
    assert.equal(tasks.some((task) => task.solverMode === 'mj_restore_tr_safe'), false);
    assert.deepEqual(
        tasks.slice(0, 3).map((task) => task.scenarioLabel),
        [
            'absolute | nominal | Recovery Optimization (TR)',
            'absolute | nominal | Recovery Optimization (TR+w)',
            'absolute | nominal | Recovery Optimization (TR+w_vfi)'
        ]
    );
    assert.deepEqual(
        tasks.slice(-3).map((task) => task.scenarioLabel),
        [
            'waist-distance | nominal | Recovery Optimization (TR)',
            'waist-distance | nominal | Recovery Optimization (TR+w)',
            'waist-distance | nominal | Recovery Optimization (TR+w_vfi)'
        ]
    );
    assert.deepEqual(
        tasks.slice(3, 6).map((task) => task.scenarioLabel),
        [
            'decoupled-upright | nominal | Recovery Optimization (TR)',
            'decoupled-upright | nominal | Recovery Optimization (TR+w)',
            'decoupled-upright | nominal | Recovery Optimization (TR+w_vfi)'
        ]
    );
    assert.equal(tasks.at(-1).controlMode, 'distance-fullbody');
    assert.equal(tasks.at(-1).jogDirection, 'negative');
    assert.equal(tasks.at(-1).progressKind, 'relative-distance');
});

test('summarizeHumanoidControllerComparisonRows reports long-form table metrics by scenario and controller', () => {
    const rows = summarizeHumanoidControllerComparisonRows([
        {
            mode: 'absolute',
            profile: 'nominal',
            controllerKey: 'restore_tr',
            controllerLabel: 'Recovery Optimization (TR)',
            completed: true,
            completionTicks: 2,
            finalXProgress: 0.012,
            targetXMeters: 0.01,
            terminalReason: 'completed',
            steps: [
                {
                    commandError: 1e-4,
                    invariantError: 2e-5,
                    jointClearanceMin: 0.20,
                    waistClearanceMin: Number.NaN,
                    maxViolation: 0,
                    tickSolveMs: 2,
                    taskScale: 1
                },
                {
                    commandError: 3e-4,
                    invariantError: 4e-5,
                    jointClearanceMin: 0.10,
                    waistClearanceMin: Number.NaN,
                    maxViolation: 1e-7,
                    tickSolveMs: 4,
                    taskScale: 0.8
                }
            ]
        },
        {
            mode: 'decoupled-upright',
            profile: 'nominal',
            controllerKey: 'restore_tr_w',
            controllerLabel: 'Recovery Optimization (TR+w)',
            completed: false,
            completionTicks: Number.NaN,
            finalXProgress: 0.20,
            finalCommandError: 5e-4,
            finalInvariantError: 8e-5,
            finalRelativePoseChange: 2e-5,
            terminalReason: 'stall',
            steps: [
                {
                    commandError: 4e-4,
                    invariantError: 6e-5,
                    relativePoseChange: 1e-5,
                    bodyError: 7e-5,
                    jointClearanceMin: 0.30,
                    waistClearanceMin: 0.04,
                    maxViolation: 0,
                    tickSolveMs: 10
                },
                {
                    commandError: 6e-4,
                    invariantError: 8e-5,
                    relativePoseChange: 2e-5,
                    bodyError: 1e-4,
                    jointClearanceMin: 0.20,
                    waistClearanceMin: 0.02,
                    maxViolation: 3e-6,
                    tickSolveMs: 20
                }
            ]
        },
        {
            mode: 'waist-distance',
            profile: 'nominal',
            controllerKey: 'restore_tr_wvfi',
            controllerLabel: 'Recovery Optimization (TR+w_vfi)',
            completed: true,
            completionTicks: 3,
            finalDistanceProgress: 0.04,
            finalCommandError: 7e-7,
            finalInvariantError: 9e-5,
            terminalReason: 'completed',
            steps: [
                {
                    commandError: 6e-7,
                    invariantError: 1e-4,
                    jointClearanceMin: 0.11,
                    waistClearanceMin: 0.06,
                    maxViolation: 0,
                    tickSolveMs: 7
                },
                {
                    commandError: 8e-7,
                    invariantError: 8e-5,
                    jointClearanceMin: 0.09,
                    waistClearanceMin: 0.05,
                    maxViolation: 0,
                    tickSolveMs: 9
                }
            ]
        }
    ]);

    assert.deepEqual(rows, [
        {
            scenario: 'absolute',
            start_posture: 'nominal',
            controller: 'Recovery Optimization (TR)',
            success_ticks: 'yes / 2',
            task_valid_ticks: '0 / 2',
            progress_m: 0.01,
            avg_task_deviation: 2e-4,
            avg_invariant_deviation: 3e-5,
            avg_joint_clearance_rad: 0.15,
            min_joint_clearance_rad: 0.10,
            avg_waist_clearance_m: Number.NaN,
            min_waist_clearance_m: Number.NaN,
            max_violation: 1e-7,
            mean_solve_ms: 3,
            p95_solve_ms: 4,
            mean_p95_solve_ms: '3.0 / 4.0',
            final_task_deviation: 3e-4,
            final_invariant_deviation: 4e-5,
            terminal_reason: 'completed',
            task_scale: 0.9,
            tr_safety: 'n/a'
        },
        {
            scenario: 'decoupled-upright',
            start_posture: 'nominal',
            controller: 'Recovery Optimization (TR+w)',
            success_ticks: 'no / n/a',
            task_valid_ticks: '0 / 2',
            progress_m: 0.20,
            avg_task_deviation: 5e-4,
            avg_invariant_deviation: 1.5e-5,
            avg_joint_clearance_rad: 0.25,
            min_joint_clearance_rad: 0.20,
            avg_waist_clearance_m: 0.03,
            min_waist_clearance_m: 0.02,
            max_violation: 3e-6,
            mean_solve_ms: 15,
            p95_solve_ms: 20,
            mean_p95_solve_ms: '15.0 / 20.0',
            final_task_deviation: 5e-4,
            final_invariant_deviation: 2e-5,
            terminal_reason: 'stall',
            task_scale: Number.NaN,
            tr_safety: 'n/a'
        },
        {
            scenario: 'waist-distance',
            start_posture: 'nominal',
            controller: 'Recovery Optimization (TR+w_vfi)',
            success_ticks: 'yes / 3',
            task_valid_ticks: '2 / 2',
            progress_m: 0.04,
            avg_task_deviation: 7e-7,
            avg_invariant_deviation: 9e-5,
            avg_joint_clearance_rad: 0.10,
            min_joint_clearance_rad: 0.09,
            avg_waist_clearance_m: 0.055,
            min_waist_clearance_m: 0.05,
            max_violation: 0,
            mean_solve_ms: 8,
            p95_solve_ms: 9,
            mean_p95_solve_ms: '8.0 / 9.0',
            final_task_deviation: 7e-7,
            final_invariant_deviation: 9e-5,
            terminal_reason: 'completed',
            task_scale: Number.NaN,
            tr_safety: 'n/a'
        }
    ]);
});

test('summarizeHumanoidControllerComparisonRows supports configurable task-valid tolerance', () => {
    const rows = summarizeHumanoidControllerComparisonRows([
        {
            mode: 'absolute',
            profile: 'nominal',
            controllerKey: 'restore_tr',
            completed: false,
            steps: [
                { commandError: 8e-7 },
                { commandError: 2e-6 },
                { commandError: Number.NaN }
            ]
        }
    ], { taskValidTolerance: 2e-6 });

    assert.equal(rows[0].task_valid_ticks, '2 / 3');
});

test('summarizeHumanoidControllerComparisonRows reports raw progress for fixed-tick runs', () => {
    const rows = summarizeHumanoidControllerComparisonRows([
        {
            mode: 'absolute',
            profile: 'nominal',
            controllerKey: 'restore_tr',
            completed: true,
            completionTicks: 30,
            targetXMeters: 0.075,
            finalXProgress: 0.105,
            fixedTicks: true,
            terminalReason: 'fixed_ticks_completed',
            steps: [{ commandError: 1e-7 }]
        }
    ]);

    assert.equal(rows[0].progress_m, 0.105);
});

test('getHumanoidControllerComparisonColumns exposes the table schema for the paper benchmark', () => {
    const columns = getHumanoidControllerComparisonColumns();
    assert.deepEqual(
        columns.map((column) => column.label),
        [
            'Scenario',
            'Controller',
            'Success / ticks',
            'Task-valid ticks',
            'Progress (m)',
            'Avg. task deviation',
            'Avg. invariant deviation',
            'Avg. joint-limit clearance (rad)',
            'Avg. waist-corridor clearance (m)',
            'Mean / P95 solve time (ms)'
        ]
    );
});

test('resolveHumanoidControllerComparisonControllerOptions keeps TR benchmark settings on the requested full-body budget', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr_w',
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
    assert.equal(result.jointLimitAvoidanceEnabled, true);
    assert.equal(result.qpUseInvariantEquality, true);
});

test('resolveHumanoidControllerComparisonControllerOptions keeps TR methods under shared hard limits', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr_wvfi',
        mode: 'decoupled-upright',
        includeWaist: true,
        maxTicks: 120,
        teachSpeedPercent: 50,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.commandTolerance, 5e-4);
    assert.equal(result.maxStallTicks, 3);
    assert.equal(result.maxTicks, 120);
    assert.equal(result.targetXMeters, 0.30);
    assert.equal(result.jointLimitAvoidanceEnabled, true);
    assert.equal(result.qpUseInvariantEquality, true);
});

test('resolveHumanoidControllerComparisonControllerOptions honors the requested full-body target distance', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr',
        mode: 'decoupled-upright',
        includeWaist: true,
        maxTicks: 30,
        teachSpeedPercent: 50,
        targetXMeters: 0.075,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.maxTicks, 30);
    assert.equal(result.targetXMeters, 0.075);
});

test('resolveHumanoidControllerComparisonControllerOptions uses a short d-minus horizon for waist distance', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr_wvfi',
        mode: 'waist-distance',
        includeWaist: true,
        maxTicks: 120,
        teachSpeedPercent: 50,
        targetXMeters: 0.30,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.maxTicks, 30);
    assert.equal(result.targetXMeters, 0.075);
    assert.equal(result.jointLimitAvoidanceEnabled, true);
    assert.equal(result.qpUseInvariantEquality, true);
});

test('resolveHumanoidControllerComparisonControllerOptions can preserve fixed-tick benchmark budgets', () => {
    const armOnly = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr',
        mode: 'absolute',
        includeWaist: false,
        maxTicks: 50,
        teachSpeedPercent: 50,
        fixedTicks: true
    });
    const waistDistance = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr_wvfi',
        mode: 'waist-distance',
        includeWaist: true,
        maxTicks: 100,
        teachSpeedPercent: 50,
        fixedTicks: true
    });

    assert.equal(armOnly.maxTicks, 50);
    assert.equal(waistDistance.maxTicks, 100);
    assert.equal(armOnly.targetXMeters, 0.075);
    assert.equal(waistDistance.targetXMeters, 0.075);
});

test('resolveHumanoidControllerComparisonControllerOptions shortens the arm-only absolute benchmark for TR+w', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr_w',
        mode: 'absolute',
        includeWaist: false,
        maxTicks: 120,
        teachSpeedPercent: 50,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.maxTicks, 30);
    assert.equal(result.maxStallTicks, 3);
    assert.equal(result.targetXMeters, 0.075);
    assert.equal(result.commandTolerance, 5e-4);
    assert.equal(result.jointLimitAvoidanceEnabled, true);
    assert.equal(result.qpUseInvariantEquality, true);
});

test('resolveHumanoidControllerComparisonControllerOptions matches arm-only Restore TR web jogging without joint-limit VFI', () => {
    const result = resolveHumanoidControllerComparisonControllerOptions({
        controllerKey: 'restore_tr',
        mode: 'absolute',
        includeWaist: false,
        maxTicks: 120,
        teachSpeedPercent: 50,
        commandTolerance: 5e-4,
        maxStallTicks: 3
    });

    assert.equal(result.maxTicks, 30);
    assert.equal(result.targetXMeters, 0.075);
    assert.equal(result.jointLimitAvoidanceEnabled, false);
    assert.equal(result.qpUseInvariantEquality, true);
});

test('resolveHumanoidControllerComparisonSpeedPercent clamps and defaults benchmark teach speed', () => {
    assert.equal(resolveHumanoidControllerComparisonSpeedPercent(), 50);
    assert.equal(resolveHumanoidControllerComparisonSpeedPercent('50'), 50);
    assert.equal(resolveHumanoidControllerComparisonSpeedPercent(0), 1);
    assert.equal(resolveHumanoidControllerComparisonSpeedPercent(130), 100);
    assert.equal(resolveHumanoidControllerComparisonSpeedPercent('not-a-number'), 50);
});

test('resolveHumanoidControllerComparisonTaskValidTolerance clamps invalid values to the strict default', () => {
    assert.equal(resolveHumanoidControllerComparisonTaskValidTolerance('2e-6'), 2e-6);
    assert.equal(resolveHumanoidControllerComparisonTaskValidTolerance('-1'), 0);
    assert.equal(resolveHumanoidControllerComparisonTaskValidTolerance('bad'), 1e-6);
});

test('resolveHumanoidControllerComparisonDefaultMaxTicks preserves the same physical horizon at lower teach speed', () => {
    assert.equal(resolveHumanoidControllerComparisonDefaultMaxTicks(100), 60);
    assert.equal(resolveHumanoidControllerComparisonDefaultMaxTicks(50), 120);
    assert.equal(resolveHumanoidControllerComparisonDefaultMaxTicks(20), 300);
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
