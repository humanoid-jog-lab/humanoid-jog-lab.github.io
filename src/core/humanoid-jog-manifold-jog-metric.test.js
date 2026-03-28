import test from 'node:test';
import assert from 'node:assert/strict';

import {
    normalizeHumanoidJogManifoldJogMetric,
    resolveHumanoidJogManifoldJogMetric,
    resolveHumanoidJogManifoldJogMetricFallback
} from './humanoid-jog-manifold-jog-metric.js';

test('normalizeHumanoidJogManifoldJogMetric recognizes every supported selector value', () => {
    assert.equal(normalizeHumanoidJogManifoldJogMetric('none'), 'none');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_vfi'), 'w_vfi');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_cmd'), 'w_cmd');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_abs'), 'w_abs');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_abs_arm'), 'w_abs_arm');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_abs_arm_vfi'), 'w_abs_arm_vfi');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_abs_vfi'), 'w_abs_vfi');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_abs_hier'), 'w_abs_hier');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('w_abs_hier_vfi'), 'w_abs_hier_vfi');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('sigma'), 'sigma');
});

test('normalizeHumanoidJogManifoldJogMetric falls back to none for unsupported values', () => {
    assert.equal(normalizeHumanoidJogManifoldJogMetric(''), 'none');
    assert.equal(normalizeHumanoidJogManifoldJogMetric('unknown'), 'none');
    assert.equal(normalizeHumanoidJogManifoldJogMetric(null), 'none');
});

test('resolveHumanoidJogManifoldJogMetric classifies direct metrics', () => {
    const resolved = resolveHumanoidJogManifoldJogMetric('w_abs_vfi');

    assert.deepEqual(resolved, {
        requestedMetric: 'w_abs_vfi',
        metric: 'w_abs_vfi',
        family: 'direct',
        shouldSkip: false,
        diagnosticMetric: 'w_abs_vfi'
    });
});

test('resolveHumanoidJogManifoldJogMetric classifies hierarchical metrics', () => {
    const resolved = resolveHumanoidJogManifoldJogMetric('w_abs_hier_vfi');

    assert.deepEqual(resolved, {
        requestedMetric: 'w_abs_hier_vfi',
        metric: 'w_abs_hier_vfi',
        family: 'hierarchical',
        shouldSkip: false,
        diagnosticMetric: 'w_abs_hier_vfi'
    });
});

test('resolveHumanoidJogManifoldJogMetric keeps hierarchical absolute VFI routing when the blend is 1', () => {
    const resolved = resolveHumanoidJogManifoldJogMetric('w_abs_hier_vfi', {
        hierarchicalVfiBlend: 1
    });

    assert.deepEqual(resolved, {
        requestedMetric: 'w_abs_hier_vfi',
        metric: 'w_abs_hier_vfi',
        family: 'hierarchical',
        shouldSkip: false,
        diagnosticMetric: 'w_abs_hier_vfi'
    });
});

test('resolveHumanoidJogManifoldJogMetric keeps hierarchical absolute VFI routing when the blend is 0', () => {
    const resolved = resolveHumanoidJogManifoldJogMetric('w_abs_hier_vfi', {
        hierarchicalVfiBlend: 0
    });

    assert.deepEqual(resolved, {
        requestedMetric: 'w_abs_hier_vfi',
        metric: 'w_abs_hier_vfi',
        family: 'hierarchical',
        shouldSkip: false,
        diagnosticMetric: 'w_abs_hier_vfi'
    });
});

test('resolveHumanoidJogManifoldJogMetric classifies sigma as directional', () => {
    const resolved = resolveHumanoidJogManifoldJogMetric('sigma');

    assert.deepEqual(resolved, {
        requestedMetric: 'sigma',
        metric: 'sigma',
        family: 'directional',
        shouldSkip: false,
        diagnosticMetric: 'sigma'
    });
});

test('resolveHumanoidJogManifoldJogMetric marks none as skipped', () => {
    const resolved = resolveHumanoidJogManifoldJogMetric('none');

    assert.deepEqual(resolved, {
        requestedMetric: 'none',
        metric: 'none',
        family: 'none',
        shouldSkip: true,
        diagnosticMetric: 'none'
    });
});

test('resolveHumanoidJogManifoldJogMetricFallback falls back to restoration when a selected metric would block a converged jog', () => {
    const fallback = resolveHumanoidJogManifoldJogMetricFallback({
        metric: 'w_abs',
        restorationAccepted: true,
        effectiveAccepted: false,
        jogMetricState: 'rejected(direction_mismatch)'
    });

    assert.deepEqual(fallback, {
        shouldFallback: true,
        state: 'rejected(direction_mismatch); fallback(restoration_only)'
    });
});

test('resolveHumanoidJogManifoldJogMetricFallback stays inert for the none selector', () => {
    const fallback = resolveHumanoidJogManifoldJogMetricFallback({
        metric: 'none',
        restorationAccepted: true,
        effectiveAccepted: false,
        jogMetricState: 'skipped(none)'
    });

    assert.deepEqual(fallback, {
        shouldFallback: false,
        state: 'skipped(none)'
    });
});

test('resolveHumanoidJogManifoldJogMetricFallback preserves hierarchical primary-only progress instead of discarding it', () => {
    const fallback = resolveHumanoidJogManifoldJogMetricFallback({
        metric: 'w_abs_hier_vfi',
        restorationAccepted: true,
        effectiveAccepted: false,
        jogMetricState: 'active(primary_only; secondary=arm_guard)'
    });

    assert.deepEqual(fallback, {
        shouldFallback: false,
        state: 'active(primary_only; secondary=arm_guard)'
    });
});

test('resolveHumanoidJogManifoldJogMetricFallback stays inert for projection-only metric states', () => {
    const fallback = resolveHumanoidJogManifoldJogMetricFallback({
        metric: 'w_abs_hier_vfi',
        restorationAccepted: true,
        effectiveAccepted: false,
        jogMetricState: 'skipped(projection_only_no_progress)'
    });

    assert.deepEqual(fallback, {
        shouldFallback: false,
        state: 'skipped(projection_only_no_progress)'
    });
});
