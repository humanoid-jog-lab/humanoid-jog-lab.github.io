import test from 'node:test';
import assert from 'node:assert/strict';

import {
    getHumanoidTaskMetricDisplayName,
    getHumanoidTaskMetricOptionLabels
} from './humanoid-jog-task-metric-labels.js';

test('getHumanoidTaskMetricDisplayName maps hierarchical metrics to relative task labels', () => {
    assert.equal(getHumanoidTaskMetricDisplayName('w_abs_hier', 'relative'), 'w_rel');
    assert.equal(getHumanoidTaskMetricDisplayName('w_abs_hier_vfi', 'relative-fullbody'), 'w_rel_vfi');
});

test('getHumanoidTaskMetricDisplayName maps hierarchical metrics to robot and distance task labels', () => {
    assert.equal(getHumanoidTaskMetricDisplayName('w_abs_hier', 'robot1'), 'w_left');
    assert.equal(getHumanoidTaskMetricDisplayName('w_abs_hier_vfi', 'robot2-target'), 'w_right_vfi');
    assert.equal(getHumanoidTaskMetricDisplayName('w_abs_hier', 'distance-fullbody'), 'w_rel');
    assert.equal(getHumanoidTaskMetricDisplayName('w_abs_hier_vfi', 'waist-body-link-xz'), 'w_waist_vfi');
});

test('getHumanoidTaskMetricOptionLabels preserves absolute labels and only changes visible task prefixes', () => {
    assert.deepEqual(getHumanoidTaskMetricOptionLabels('absolute'), {
        w_abs_hier: 'w_abs',
        w_abs_hier_vfi: 'w_abs_vfi'
    });
    assert.deepEqual(getHumanoidTaskMetricOptionLabels('distance'), {
        w_abs_hier: 'w_rel',
        w_abs_hier_vfi: 'w_rel_vfi'
    });
});
