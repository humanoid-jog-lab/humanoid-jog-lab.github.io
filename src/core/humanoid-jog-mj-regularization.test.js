import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogMJRegularizationWeights } from './humanoid-jog-mj-regularization.js';

test('buildHumanoidJogMJRegularizationWeights prefers waist-min-motion for full-body upright pose jogs', () => {
    const weights = buildHumanoidJogMJRegularizationWeights({
        activeCount: 18,
        waistActiveCount: 4,
        includeWaist: true,
        keepBodyZVertical: true,
        isMetricOnlyJogRequest: false,
        metricShouldSkip: true,
        config: {
            mjRestoreWaistMotionRegularization: 6
        }
    });

    assert.deepEqual(weights, [6, 6, 6, 6, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]);
});

test('buildHumanoidJogMJRegularizationWeights stays disabled for metric-only jogs', () => {
    const weights = buildHumanoidJogMJRegularizationWeights({
        activeCount: 18,
        waistActiveCount: 4,
        includeWaist: true,
        keepBodyZVertical: true,
        isMetricOnlyJogRequest: true,
        metricShouldSkip: false,
        config: {
            mjRestoreWaistMotionRegularization: 6
        }
    });

    assert.equal(weights, null);
});
