import test from 'node:test';
import assert from 'node:assert/strict';

import {
    normalizeHumanoidJogSolverMode,
    getHumanoidJogSolverDisplayName,
    getHumanoidJogSolverReadyMessage
} from './humanoid-jog-solver-presentation.js';

test('normalizeHumanoidJogSolverMode only accepts the remaining supported solver modes', () => {
    assert.equal(normalizeHumanoidJogSolverMode('cns'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('mj_restore_tr'), 'mj_restore_tr');
    assert.equal(normalizeHumanoidJogSolverMode('mj_restore_pre_w'), 'mj_restore_pre_w');
    assert.equal(normalizeHumanoidJogSolverMode('mj_restore'), 'mj_restore');
    assert.equal(normalizeHumanoidJogSolverMode('torso_seed_upright'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('torso_seed_upright_vfi'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('sqp'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('nlopt'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('manifold'), 'vfi');
});

test('getHumanoidJogSolverDisplayName maps internal solver ids to the public names', () => {
    assert.equal(getHumanoidJogSolverDisplayName('priority'), '零空间投影');
    assert.equal(getHumanoidJogSolverDisplayName('vfi'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('cns'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('mj_restore'), 'MuJoCo 风格恢复');
    assert.equal(getHumanoidJogSolverDisplayName('mj_restore_tr'), '恢复优化(TR)');
    assert.equal(getHumanoidJogSolverDisplayName('mj_restore_pre_w'), 'MuJoCo 风格恢复 + w预优化');
    assert.equal(getHumanoidJogSolverDisplayName('torso_seed_upright'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('torso_seed_upright_vfi'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('nlopt'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('sqp'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('manifold'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('anything-else'), 'new qpJ');
});

test('getHumanoidJogSolverReadyMessage uses the public solver names', () => {
    assert.equal(getHumanoidJogSolverReadyMessage('priority'), '零空间投影 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('vfi'), 'new qpJ 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('cns'), 'new qpJ 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('mj_restore'), 'MuJoCo 风格恢复 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('mj_restore_tr'), '恢复优化(TR) 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('mj_restore_pre_w'), 'MuJoCo 风格恢复 + w预优化 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('torso_seed_upright'), 'new qpJ 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('torso_seed_upright_vfi'), 'new qpJ 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('nlopt'), 'new qpJ 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('sqp'), 'new qpJ 已启用，等待点动输入。');
    assert.equal(getHumanoidJogSolverReadyMessage('manifold'), 'new qpJ 已启用，等待点动输入。');
});
