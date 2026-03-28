import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidStaticManipTasks,
    formatHumanoidStaticManipTaskLabel,
    summarizeHumanoidStaticManipRow
} from './humanoid-jog-static-w-table.js';

test('buildHumanoidStaticManipTasks returns the 16 static absolute/relative task variants', () => {
    const tasks = buildHumanoidStaticManipTasks();
    assert.equal(tasks.length, 16);
    assert.deepEqual(
        tasks.slice(0, 4).map((task) => formatHumanoidStaticManipTaskLabel(task)),
        [
            'absolute | dual-arm | w_hier+',
            'absolute | dual-arm | w_hier_vfi+',
            'absolute | dual-arm | locked | w_hier+',
            'absolute | dual-arm | locked | w_hier_vfi+'
        ]
    );
    assert.deepEqual(
        tasks.slice(-4).map((task) => formatHumanoidStaticManipTaskLabel(task)),
        [
            'relative | full-body | w_hier+',
            'relative | full-body | w_hier_vfi+',
            'relative | full-body | locked | w_hier+',
            'relative | full-body | locked | w_hier_vfi+'
        ]
    );
});

test('buildHumanoidStaticManipTasks maps full-body absolute to the configured control mode', () => {
    const tasks = buildHumanoidStaticManipTasks({ fullBodyAbsoluteMode: 'decoupled-upright' });
    const fullBodyAbsolute = tasks.filter((task) => task.taskKind === 'absolute' && task.structure === 'full-body');
    assert.equal(fullBodyAbsolute.length, 4);
    assert.deepEqual(
        [...new Set(fullBodyAbsolute.map((task) => task.controlMode))],
        ['decoupled-upright']
    );
});

test('buildHumanoidStaticManipTasks can keep only locked rows', () => {
    const tasks = buildHumanoidStaticManipTasks({ lockedOnly: true });
    assert.equal(tasks.length, 8);
    assert.ok(tasks.every((task) => task.locked === true));
    assert.deepEqual(
        tasks.slice(0, 4).map((task) => formatHumanoidStaticManipTaskLabel(task)),
        [
            'absolute | dual-arm | locked | w_hier+',
            'absolute | dual-arm | locked | w_hier_vfi+',
            'absolute | full-body | locked | w_hier+',
            'absolute | full-body | locked | w_hier_vfi+'
        ]
    );
});

test('summarizeHumanoidStaticManipRow computes deltas and uses the formatted task label', () => {
    const [task] = buildHumanoidStaticManipTasks();
    const row = summarizeHumanoidStaticManipRow({
        task,
        before: {
            wHier: 0.3,
            wHierVfi: 0.2,
            dmin: 0.01,
            cctsm: {
                w_abs_arm: 0.11,
                w_abs_arm_vfi: 0.07
            }
        },
        after: {
            wHier: 0.35,
            wHierVfi: 0.28,
            dmin: 0.04,
            cctsm: {
                w_abs_arm: 0.16,
                w_abs_arm_vfi: 0.13
            }
        }
    });

    assert.equal(row.taskDescriptor, 'absolute | dual-arm | w_hier+');
    assert.equal(row.w_hier_before, 0.11);
    assert.equal(row.w_hier_after, 0.16);
    assert.ok(Math.abs(row.delta_w_hier - 0.05) < 1e-12);
    assert.equal(row.w_hier_vfi_before, 0.07);
    assert.equal(row.w_hier_vfi_after, 0.13);
    assert.ok(Math.abs(row.delta_w_hier_vfi - 0.06) < 1e-12);
    assert.equal(row.dmin_before, 0.01);
    assert.equal(row.dmin_after, 0.04);
});
