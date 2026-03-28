import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('index.html only exposes the supported humanoid jog solvers', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /<option value="vfi">new qpJ<\/option>/);
    assert.match(html, /<option value="priority">零空间投影<\/option>/);
    assert.match(html, /<option value="mj_restore">MuJoCo 风格恢复<\/option>/);
    assert.match(html, /<option value="mj_restore_pre_w">MuJoCo 风格恢复 \+ w预优化<\/option>/);
    assert.doesNotMatch(html, /<option value="cns">/);
    assert.doesNotMatch(html, /<option value="sqp">/);
    assert.doesNotMatch(html, /<option value="nlopt">/);
    assert.doesNotMatch(html, /<option value="manifold">/);
});

test('index.html exposes the humanoid w selector surface with only hierarchical metrics and shared buttons', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-manifold-jog-metric-select"/);
    assert.match(html, /w 指标：/);
    assert.match(html, /<option value="w_abs_hier">w_abs_hier<\/option>/);
    assert.match(html, /<option value="w_abs_hier_vfi">w_abs_hier_vfi<\/option>/);
    assert.match(html, /id="humanoid-w-metric-increase-btn"/);
    assert.doesNotMatch(html, /id="humanoid-w-metric-decrease-btn"/);
    assert.doesNotMatch(html, />w-</);
    assert.doesNotMatch(html, /<option value="w_abs">w_abs<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_arm">w_abs_arm<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_vfi">w_abs_vfi<\/option>/);
    assert.doesNotMatch(html, /w_abs_hier_vfi_iter/);
});

test('index.html hides the humanoid VFI step backend selector and task-equivalent branching toggle', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.doesNotMatch(html, /id="humanoid-vfi-step-backend-select"/);
    assert.doesNotMatch(html, /VFI 步进后端：/);
    assert.doesNotMatch(html, /Projection（原实现）/);
    assert.doesNotMatch(html, /QP（新）/);
    assert.doesNotMatch(html, /id="humanoid-task-equivalent-branching-toggle"/);
    assert.doesNotMatch(html, /任务等价多分支/);
});

test('index.html exposes the humanoid dual-arm test lock toggle', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-test-dual-arm-lock-toggle"/);
    assert.match(html, /双臂测试锁/);
    assert.match(html, /ljoint2∈\[-0\.1,0\.2\], rjoint2∈\[-0\.2,0\.1\]/);
});

test('index.html exposes the humanoid single-arm-plus-waist control modes', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /<option value="robot1-waist">左臂\+腰部<\/option>/);
    assert.match(html, /<option value="robot2-waist">右臂\+腰部<\/option>/);
});
