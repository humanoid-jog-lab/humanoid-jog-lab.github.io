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

test('index.html does not add a new humanoid w selector surface', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.doesNotMatch(html, /id="humanoid-manifold-jog-metric-select"/);
    assert.doesNotMatch(html, /w 指标：/);
    assert.doesNotMatch(html, /<option value="w_abs_hier">w_abs_hier<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_hier_vfi">w_abs_hier_vfi<\/option>/);
});

test('index.html keeps the pre-existing humanoid VFI controls intact', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-vfi-step-backend-select"/);
    assert.match(html, /<option value="projection">Projection（原实现）<\/option>/);
    assert.match(html, /<option value="qp">QP（新）<\/option>/);
    assert.match(html, /id="humanoid-task-equivalent-branching-toggle"/);
    assert.match(html, /任务等价多分支/);
});
