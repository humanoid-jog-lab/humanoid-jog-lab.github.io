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
    assert.equal(normalizeHumanoidJogSolverMode('mj_restore_tr_safe'), 'mj_restore_tr_safe');
    assert.equal(normalizeHumanoidJogSolverMode('mj_restore_pre_w'), 'mj_restore_pre_w');
    assert.equal(normalizeHumanoidJogSolverMode('mj_restore'), 'mj_restore');
    assert.equal(normalizeHumanoidJogSolverMode('torso_seed_upright'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('torso_seed_upright_vfi'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('sqp'), 'vfi');
    assert.equal(normalizeHumanoidJogSolverMode('manifold'), 'vfi');
});

test('getHumanoidJogSolverDisplayName maps internal solver ids to the public names', () => {
    assert.equal(getHumanoidJogSolverDisplayName('priority'), 'Null-space Projection');
    assert.equal(getHumanoidJogSolverDisplayName('vfi'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('cns'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('mj_restore'), 'Restore (LS)');
    assert.equal(getHumanoidJogSolverDisplayName('mj_restore_tr'), 'Recovery Optimization (TR)');
    assert.equal(getHumanoidJogSolverDisplayName('mj_restore_tr_safe'), 'Recovery Optimization (TR+Safety Bias)');
    assert.equal(getHumanoidJogSolverDisplayName('mj_restore_pre_w'), 'MuJoCo-style Recovery + w Pre-optimization');
    assert.equal(getHumanoidJogSolverDisplayName('torso_seed_upright'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('torso_seed_upright_vfi'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('sqp'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('manifold'), 'new qpJ');
    assert.equal(getHumanoidJogSolverDisplayName('anything-else'), 'new qpJ');
});

test('getHumanoidJogSolverReadyMessage uses the public solver names', () => {
    assert.equal(getHumanoidJogSolverReadyMessage('priority'), 'Null-space Projection is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('vfi'), 'new qpJ is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('cns'), 'new qpJ is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('mj_restore'), 'Restore (LS) is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('mj_restore_tr'), 'Recovery Optimization (TR) is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('mj_restore_tr_safe'), 'Recovery Optimization (TR+Safety Bias) is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('mj_restore_pre_w'), 'MuJoCo-style Recovery + w Pre-optimization is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('torso_seed_upright'), 'new qpJ is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('torso_seed_upright_vfi'), 'new qpJ is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('sqp'), 'new qpJ is enabled and waiting for jog input.');
    assert.equal(getHumanoidJogSolverReadyMessage('manifold'), 'new qpJ is enabled and waiting for jog input.');
});
