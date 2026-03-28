import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidJogMJWorkerSecondaryMetricRequest,
    computeHumanoidJogMJNullspaceRefine,
    shouldComputeHumanoidJogMJInitialManipData,
    shouldComputeHumanoidJogMJTrialManipData,
    shouldEnableHumanoidJogMJNullspaceRefine,
    shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt,
    shouldUseHumanoidJogMJProjectionOnlyMetricStep
} from './humanoid-jog-mj-nullspace.js';

test('computeHumanoidJogMJNullspaceRefine skips when restoration is not converged', () => {
    const result = computeHumanoidJogMJNullspaceRefine({
        restorationConverged: false
    });

    assert.equal(result.applied, false);
    assert.equal(result.state, 'skipped(waiting_for_restore)');
});

test('computeHumanoidJogMJNullspaceRefine skips when nullspace refine is disabled', () => {
    const result = computeHumanoidJogMJNullspaceRefine({
        restorationConverged: true,
        enabled: false
    });

    assert.equal(result.applied, false);
    assert.equal(result.state, 'skipped(disabled_for_mj_restore)');
});

test('computeHumanoidJogMJNullspaceRefine passes through accepted metric attempts', () => {
    const result = computeHumanoidJogMJNullspaceRefine({
        restorationConverged: true,
        metricAttempt: {
            state: 'active',
            applied: true,
            before: 0.1,
            after: 0.2,
            axisLabel: 'x',
            nullspaceResult: { accepted: true },
            refineResult: { converged: true },
            finalProblem: { value: true }
        }
    });

    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.after, 0.2);
    assert.equal(result.axisLabel, 'x');
});

test('shouldEnableHumanoidJogMJNullspaceRefine allows hierarchical metrics even when mj_restore nullspace refine is globally disabled', () => {
    assert.equal(shouldEnableHumanoidJogMJNullspaceRefine({
        enabled: false,
        metric: 'w_abs_arm'
    }), true);

    assert.equal(shouldEnableHumanoidJogMJNullspaceRefine({
        enabled: false,
        metric: 'w_abs_hier'
    }), true);

    assert.equal(shouldEnableHumanoidJogMJNullspaceRefine({
        enabled: false,
        metric: 'w_abs_hier_vfi'
    }), true);

    assert.equal(shouldEnableHumanoidJogMJNullspaceRefine({
        enabled: false,
        metric: 'w_abs_vfi'
    }), true);

    assert.equal(shouldEnableHumanoidJogMJNullspaceRefine({
        enabled: false,
        metric: 'w_abs'
    }), false);
});

test('shouldComputeHumanoidJogMJTrialManipData skips trial manipulability data for mj_restore none path', () => {
    assert.equal(shouldComputeHumanoidJogMJTrialManipData({
        metric: 'none',
        nullspaceRefineEnabled: false,
        wvfiLowerBound: 0
    }), false);

    assert.equal(shouldComputeHumanoidJogMJTrialManipData({
        metric: 'w_abs_hier_vfi',
        nullspaceRefineEnabled: true,
        wvfiLowerBound: 0
    }), true);

    assert.equal(shouldComputeHumanoidJogMJTrialManipData({
        metric: 'none',
        nullspaceRefineEnabled: false,
        wvfiLowerBound: 0.001
    }), true);
});

test('shouldComputeHumanoidJogMJInitialManipData skips initial manipulability precompute when worker handles hierarchical metrics', () => {
    assert.equal(shouldComputeHumanoidJogMJInitialManipData({
        metric: 'w_abs_arm',
        nullspaceRefineEnabled: true,
        wvfiLowerBound: 0,
        workerHandlesMetric: true
    }), false);

    assert.equal(shouldComputeHumanoidJogMJInitialManipData({
        metric: 'w_abs_hier',
        nullspaceRefineEnabled: true,
        wvfiLowerBound: 0,
        workerHandlesMetric: true
    }), false);

    assert.equal(shouldComputeHumanoidJogMJInitialManipData({
        metric: 'w_abs_hier_vfi',
        nullspaceRefineEnabled: true,
        wvfiLowerBound: 0,
        workerHandlesMetric: true
    }), false);

    assert.equal(shouldComputeHumanoidJogMJInitialManipData({
        metric: 'w_abs_hier_vfi',
        nullspaceRefineEnabled: true,
        wvfiLowerBound: 0,
        workerHandlesMetric: false
    }), true);
});

test('buildHumanoidJogMJWorkerSecondaryMetricRequest includes hierarchical absolute metrics that already have worker-side parity', () => {
    const request = buildHumanoidJogMJWorkerSecondaryMetricRequest({
        metric: 'w_abs_hier_vfi',
        metricMode: 'relative',
        preferWorker: true,
        workerAvailable: true,
        nullspaceRefineEnabled: true,
        stepSize: 0.01,
        restoreMaxIterations: 24,
        waistActiveCount: 0,
        hierarchicalVfiBlend: 0.35
    });

    assert.deepEqual(request, {
        metric: 'w_abs_hier_vfi',
        metricMode: 'relative',
        stepSize: 0.01,
        restoreMaxIterations: 24,
        waistActiveCount: 0,
        hierarchicalVfiBlend: 0.35
    });
});

test('buildHumanoidJogMJWorkerSecondaryMetricRequest returns null when the worker should not own the secondary step', () => {
    assert.equal(buildHumanoidJogMJWorkerSecondaryMetricRequest({
        metric: 'w_abs',
        preferWorker: true,
        workerAvailable: true,
        nullspaceRefineEnabled: true
    }), null);
});

test('shouldUseHumanoidJogMJProjectionOnlyMetricStep enables projection-only microsteps for metric-only hierarchical VFI stages', () => {
    assert.equal(shouldUseHumanoidJogMJProjectionOnlyMetricStep({
        isMetricOnlyJogRequest: true,
        requestedMetric: 'w_abs_hier_vfi',
        stageMetric: 'w_abs_arm_vfi'
    }), true);

    assert.equal(shouldUseHumanoidJogMJProjectionOnlyMetricStep({
        isMetricOnlyJogRequest: true,
        requestedMetric: 'w_abs_hier_vfi',
        stageMetric: 'w_abs_hier_vfi'
    }), true);
});

test('shouldUseHumanoidJogMJProjectionOnlyMetricStep stays off for non-hierarchical or non-metric-only stages', () => {
    assert.equal(shouldUseHumanoidJogMJProjectionOnlyMetricStep({
        isMetricOnlyJogRequest: false,
        requestedMetric: 'w_abs_hier_vfi',
        stageMetric: 'w_abs_arm_vfi'
    }), false);

    assert.equal(shouldUseHumanoidJogMJProjectionOnlyMetricStep({
        isMetricOnlyJogRequest: true,
        requestedMetric: 'w_abs_hier',
        stageMetric: 'w_abs_arm_vfi'
    }), false);

    assert.equal(shouldUseHumanoidJogMJProjectionOnlyMetricStep({
        isMetricOnlyJogRequest: true,
        requestedMetric: 'w_abs_hier_vfi',
        stageMetric: 'w_abs'
    }), false);
});

test('shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt only retries metric-only hierarchical VFI requests after a hard-constraint worker rejection', () => {
    assert.equal(shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt({
        metric: 'w_abs_hier_vfi',
        isMetricOnlyJogRequest: true,
        workerMetricAttempt: {
            state: 'rejected(hard_constraints)',
            applied: false
        }
    }), true);

    assert.equal(shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt({
        metric: 'w_abs_hier_vfi',
        isMetricOnlyJogRequest: true,
        workerMetricAttempt: {
            state: 'active',
            applied: true
        }
    }), false);

    assert.equal(shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt({
        metric: 'w_abs_hier',
        isMetricOnlyJogRequest: true,
        workerMetricAttempt: {
            state: 'rejected(hard_constraints)',
            applied: false
        }
    }), false);
});
