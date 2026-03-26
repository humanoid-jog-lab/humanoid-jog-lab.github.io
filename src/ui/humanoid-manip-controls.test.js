import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('index.html exposes a single generic humanoid w control pair', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-w-metric-decrease-btn"/);
    assert.match(html, /id="humanoid-w-metric-increase-btn"/);
    assert.match(html, />w-</);
    assert.match(html, />w\+</);
    assert.doesNotMatch(html, /humanoid-wvfi-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-w-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-w-abs-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-w-abs-arm-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-w-abs-hier-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-w-abs-hier-vfi-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-w-abs-hier-vfi-iter-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-w-abs-vfi-decrease-btn/);
    assert.doesNotMatch(html, /humanoid-sigma-decrease-btn/);
});

test('index.html exposes only hierarchical humanoid w metrics in the selector', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-manifold-jog-metric-select"/);
    assert.match(html, /w 指标/);
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

test('index.html includes a temporary dual-arm test lock toggle', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-test-dual-arm-lock-toggle"/);
    assert.match(html, /启用双臂测试锁/);
    assert.match(html, /ljoint2 ∈ \[-0.1, 0.2\]/);
    assert.match(html, /rjoint2 ∈ \[-0.2, 0.1\]/);
});

test('predictive certification controller toggle is not exposed in humanoid controls', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');
    const handlers = readFileSync(resolve(ROOT, 'src/ui/event-handlers.js'), 'utf8');

    assert.doesNotMatch(html, /humanoid-predictive-cert-toggle/);
    assert.doesNotMatch(html, /预测证书控制器（论文 §V）/);
    assert.doesNotMatch(html, /启用预测证书/);
    assert.doesNotMatch(handlers, /humanoid-predictive-cert-toggle/);
    assert.doesNotMatch(handlers, /setHumanoidPredictiveCertEnabled/);
    assert.doesNotMatch(handlers, /humanoid-predictive-cert-enabled/);
});
