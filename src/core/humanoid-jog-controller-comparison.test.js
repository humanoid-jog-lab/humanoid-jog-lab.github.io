import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidControllerComparisonTasks,
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
            'absolute | nominal | Ours + VFI-w'
        ]
    );
    assert.deepEqual(
        tasks.slice(-4).map((task) => task.scenarioLabel),
        [
            'decoupled-upright | near-limit | new qpJ',
            'decoupled-upright | near-limit | 零空间投影',
            'decoupled-upright | near-limit | Ours',
            'decoupled-upright | near-limit | Ours + VFI-w'
        ]
    );
});

test('summarizeHumanoidControllerComparisonRows pivots sustainable ticks by controller', () => {
    const rows = summarizeHumanoidControllerComparisonRows([
        { mode: 'absolute', profile: 'nominal', controllerKey: 'qp', sustainableTicks: 11 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'nullspace', sustainableTicks: 19 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'ours', sustainableTicks: 27 },
        { mode: 'absolute', profile: 'nominal', controllerKey: 'ours_vfiw', sustainableTicks: 31 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'qp', sustainableTicks: 4 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'nullspace', sustainableTicks: 8 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'ours', sustainableTicks: 14 },
        { mode: 'decoupled-upright', profile: 'near-limit', controllerKey: 'ours_vfiw', sustainableTicks: 18 }
    ]);

    assert.deepEqual(rows, [
        {
            mode: 'absolute',
            start_posture: 'nominal',
            qp: 11,
            nullspace: 19,
            ours: 27,
            ours_vfiw: 31
        },
        {
            mode: 'decoupled-upright',
            start_posture: 'near-limit',
            qp: 4,
            nullspace: 8,
            ours: 14,
            ours_vfiw: 18
        }
    ]);
});

test('getHumanoidControllerComparisonColumns exposes the table schema for the paper benchmark', () => {
    const columns = getHumanoidControllerComparisonColumns();
    assert.deepEqual(
        columns.map((column) => column.label),
        ['Mode', 'Start posture', 'new qpJ', 'Null-space', 'Ours', 'Ours + VFI-w']
    );
});
