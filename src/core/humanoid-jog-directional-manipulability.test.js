import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidJogDirectionalAxisVector,
    evaluateHumanoidJogAbsoluteManipulability,
    evaluateHumanoidJogAbsoluteArmManipulability,
    evaluateHumanoidJogAbsoluteArmVFIManipulability,
    evaluateHumanoidJogAbsoluteHierarchicalExtendedManipulability,
    evaluateHumanoidJogAbsoluteHierarchicalManipulability,
    evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability,
    evaluateHumanoidJogAbsoluteVFIManipulability,
    evaluateHumanoidJogDirectionalManipulability,
    evaluateHumanoidJogProjectedManipulability,
    evaluateHumanoidJogTaskArmManipulability,
    evaluateHumanoidJogTaskArmVFIManipulability,
    evaluateHumanoidJogTaskHierarchicalExtendedManipulability,
    evaluateHumanoidJogTaskHierarchicalManipulability,
    evaluateHumanoidJogTaskHierarchicalVFIManipulability,
    evaluateHumanoidJogTaskManipulability,
    evaluateHumanoidJogTaskVFIManipulability
} from './humanoid-jog-directional-manipulability.js';

test('buildHumanoidJogDirectionalAxisVector maps translation x to the absolute linear-x entry', () => {
    const vector = buildHumanoidJogDirectionalAxisVector({
        axis: 'x',
        gramSize: 12
    });

    assert.deepEqual(vector, [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]);
});

test('buildHumanoidJogDirectionalAxisVector maps rz to the absolute angular-z entry', () => {
    const vector = buildHumanoidJogDirectionalAxisVector({
        axis: 'rz',
        gramSize: 12
    });

    assert.deepEqual(vector, [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
});

test('evaluateHumanoidJogDirectionalManipulability reads the directional sigma from the absolute task block', () => {
    const sigma = evaluateHumanoidJogDirectionalManipulability({
        axis: 'x',
        cctsmResult: {
            Gram: [
                [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4]
            ]
        }
    });

    assert.equal(sigma, 2);
});

test('evaluateHumanoidJogProjectedManipulability prefers projected commanded-task w over combined w', () => {
    const value = evaluateHumanoidJogProjectedManipulability({
        w: 0.9,
        w_cmd: 0.25
    });

    assert.equal(value, 0.25);
});

test('evaluateHumanoidJogProjectedManipulability falls back to the combined w when projected w is unavailable', () => {
    const value = evaluateHumanoidJogProjectedManipulability({
        w: 0.9
    });

    assert.equal(value, 0.9);
});

test('evaluateHumanoidJogAbsoluteManipulability prefers absolute-task w_abs over combined w', () => {
    const value = evaluateHumanoidJogAbsoluteManipulability({
        w: 0.9,
        w_abs: 0.35
    });

    assert.equal(value, 0.35);
});

test('evaluateHumanoidJogAbsoluteArmManipulability prefers arm-only absolute-task w_abs_arm over w_abs and combined w', () => {
    const value = evaluateHumanoidJogAbsoluteArmManipulability({
        w: 0.9,
        w_abs: 0.35,
        w_abs_arm: 0.22
    });

    assert.equal(value, 0.22);
});

test('evaluateHumanoidJogAbsoluteHierarchicalManipulability prefers w_abs_hier over w_abs and combined w', () => {
    const value = evaluateHumanoidJogAbsoluteHierarchicalManipulability({
        w: 0.9,
        w_abs: 0.35,
        w_abs_hier: 0.31
    });

    assert.equal(value, 0.31);
});

test('evaluateHumanoidJogAbsoluteHierarchicalExtendedManipulability prefers explicit extended score', () => {
    const value = evaluateHumanoidJogAbsoluteHierarchicalExtendedManipulability({
        w_abs_hier: 0.31,
        w_abs_hier_ext: 0.42
    });

    assert.equal(value, 0.42);
});

test('evaluateHumanoidJogAbsoluteArmVFIManipulability prefers w_abs_arm_vfi over w_abs_vfi and w_abs_arm', () => {
    const value = evaluateHumanoidJogAbsoluteArmVFIManipulability({
        w: 0.9,
        w_abs_arm: 0.35,
        w_abs_vfi: 0.21,
        w_abs_arm_vfi: 0.17
    });

    assert.equal(value, 0.17);
});

test('evaluateHumanoidJogAbsoluteVFIManipulability prefers absolute-task w_abs_vfi over w_vfi and combined w', () => {
    const value = evaluateHumanoidJogAbsoluteVFIManipulability({
        w: 0.9,
        w_vfi: 0.3,
        w_abs_vfi: 0.21
    });

    assert.equal(value, 0.21);
});

test('evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability interpolates between w_abs and w_abs_vfi', () => {
    const value = evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability({
        w_abs: 0.4,
        w_abs_vfi: 0.1
    }, {
        hierarchicalVfiBlend: 0.5
    });

    assert.equal(value, 0.25);
});

test('evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability prefers explicit w_abs_hier_vfi when present', () => {
    const value = evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability({
        w_abs: 0.4,
        w_abs_hier: 0.4,
        w_abs_vfi: 0.1,
        w_abs_hier_vfi: 0.18
    }, {
        hierarchicalVfiBlend: 1
    });

    assert.equal(value, 0.18);
});

test('evaluateHumanoidJogTaskManipulability prefers generic current-task w_task over absolute-task and combined fields', () => {
    const value = evaluateHumanoidJogTaskManipulability({
        w: 0.9,
        w_abs: 0.35,
        w_task: 0.22
    });

    assert.equal(value, 0.22);
});

test('evaluateHumanoidJogTaskHierarchicalExtendedManipulability prefers current-task extended score', () => {
    const value = evaluateHumanoidJogTaskHierarchicalExtendedManipulability({
        w_abs_hier_ext: 0.42,
        w_task_hier_ext: 0.24
    });

    assert.equal(value, 0.24);
});

test('evaluateHumanoidJogTaskArmManipulability prefers generic current-task w_task_arm over task and absolute-arm fields', () => {
    const value = evaluateHumanoidJogTaskArmManipulability({
        w_task: 0.33,
        w_abs_arm: 0.28,
        w_task_arm: 0.19
    });

    assert.equal(value, 0.19);
});

test('evaluateHumanoidJogTaskVFIManipulability prefers generic current-task w_task_vfi over task and absolute-task VFI fields', () => {
    const value = evaluateHumanoidJogTaskVFIManipulability({
        w_task: 0.33,
        w_abs_vfi: 0.21,
        w_task_vfi: 0.15
    });

    assert.equal(value, 0.15);
});

test('evaluateHumanoidJogTaskArmVFIManipulability prefers generic current-task w_task_arm_vfi over task-vfi and absolute-arm-vfi fields', () => {
    const value = evaluateHumanoidJogTaskArmVFIManipulability({
        w_task_vfi: 0.16,
        w_abs_arm_vfi: 0.12,
        w_task_arm_vfi: 0.09
    });

    assert.equal(value, 0.09);
});

test('evaluateHumanoidJogTaskHierarchicalManipulability prefers generic current-task w_task_hier over task and absolute hierarchical fields', () => {
    const value = evaluateHumanoidJogTaskHierarchicalManipulability({
        w_task: 0.27,
        w_abs_hier: 0.24,
        w_task_hier: 0.18
    });

    assert.equal(value, 0.18);
});

test('evaluateHumanoidJogTaskHierarchicalVFIManipulability prefers explicit generic current-task w_task_hier_vfi and clamps it below the task hierarchy baseline', () => {
    const value = evaluateHumanoidJogTaskHierarchicalVFIManipulability({
        w_task_hier: 0.31,
        w_task_vfi: 0.12,
        w_abs_hier_vfi: 0.2,
        w_task_hier_vfi: 0.16
    }, {
        hierarchicalVfiBlend: 1
    });

    assert.equal(value, 0.16);
});
