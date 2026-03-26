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

test('index.html keeps the existing humanoid w selector unchanged', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-manifold-jog-metric-select"/);
    assert.match(html, /<option value="w_abs_hier">w_abs_hier<\/option>/);
    assert.match(html, /<option value="w_abs_hier_vfi">w_abs_hier_vfi<\/option>/);
    assert.doesNotMatch(html, /<option value="none">none<\/option>/);
    assert.doesNotMatch(html, /<option value="w_vfi">w_vfi<\/option>/);
    assert.doesNotMatch(html, /<option value="w_cmd">w_cmd<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs">w_abs<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_arm">w_abs_arm<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_vfi">w_abs_vfi<\/option>/);
    assert.doesNotMatch(html, /<option value="sigma">sigma<\/option>/);
});
