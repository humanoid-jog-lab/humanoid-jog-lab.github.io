import test from 'node:test';
import assert from 'node:assert/strict';

import { resolveHumanoidJogMJMetricTolerances } from './humanoid-jog-mj-metric-tolerances.js';

test('resolveHumanoidJogMJMetricTolerances keeps metric-only hierarchical VFI acceptance strict on pose preservation', () => {
    const result = resolveHumanoidJogMJMetricTolerances({
        isMetricOnlyJogRequest: true,
        isHierarchicalVFIJogMetric: true,
        hardTolerance: 1e-8,
        metricButtonCommandTolerance: 1e-3,
        metricButtonInvariantTolerance: 1e-4,
        activeLimitRetreatCommandTolerance: 1e-2,
        activeLimitRetreatInvariantTolerance: 2e-3
    });

    assert.equal(result.metricRefinementTolerance, 1e-4);
    assert.equal(result.acceptanceCommandTolerance, 1e-3);
    assert.equal(result.acceptanceRelativeTolerance, 1e-4);
});

test('resolveHumanoidJogMJMetricTolerances keeps metric-only direct w_abs_vfi acceptance strict on pose preservation', () => {
    const result = resolveHumanoidJogMJMetricTolerances({
        isMetricOnlyJogRequest: true,
        isHierarchicalVFIJogMetric: true,
        hardTolerance: 1e-8,
        metricButtonCommandTolerance: 1e-3,
        metricButtonInvariantTolerance: 1e-4,
        activeLimitRetreatCommandTolerance: 1e-2,
        activeLimitRetreatInvariantTolerance: 2e-3
    });

    assert.equal(result.metricRefinementTolerance, 1e-4);
    assert.equal(result.acceptanceCommandTolerance, 1e-3);
    assert.equal(result.acceptanceRelativeTolerance, 1e-4);
});

test('resolveHumanoidJogMJMetricTolerances keeps hard tolerances for non metric-only restore requests', () => {
    const result = resolveHumanoidJogMJMetricTolerances({
        isMetricOnlyJogRequest: false,
        isHierarchicalVFIJogMetric: true,
        hardTolerance: 1e-8,
        metricButtonCommandTolerance: 1e-3,
        metricButtonInvariantTolerance: 1e-4,
        activeLimitRetreatCommandTolerance: 1e-2,
        activeLimitRetreatInvariantTolerance: 2e-3
    });

    assert.equal(result.metricRefinementTolerance, 1e-8);
    assert.equal(result.acceptanceCommandTolerance, 1e-8);
    assert.equal(result.acceptanceRelativeTolerance, 1e-8);
});
