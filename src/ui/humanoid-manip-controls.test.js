import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('index.html only exposes the supported humanoid jog solvers', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /<option value="vfi">new qpJ<\/option>/);
    assert.match(html, /<option value="priority">零空间投影<\/option>/);
    assert.match(html, /<option value="mj_restore">Restore \(LS\)<\/option>/);
    assert.match(html, /<option value="mj_restore_tr">恢复优化\(TR\)<\/option>/);
    assert.doesNotMatch(html, /<option value="mj_restore_pre_w">/);
    assert.doesNotMatch(html, /<option value="torso_seed_upright">Torso-Seeded Upright<\/option>/);
    assert.doesNotMatch(html, /<option value="torso_seed_upright_vfi">Torso-Seeded Upright \+ VFI-w<\/option>/);
    assert.doesNotMatch(html, /<option value="cns">/);
    assert.doesNotMatch(html, /<option value="sqp">/);
    assert.doesNotMatch(html, /<option value="manifold">/);
});

test('index.html exposes the humanoid w selector surface with only hierarchical metrics and shared buttons', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-manifold-jog-metric-select"/);
    assert.match(html, /w 指标：/);
    assert.match(html, /<option value="w_abs_hier">w_abs<\/option>/);
    assert.match(html, /<option value="w_abs_hier_vfi">w_abs_vfi<\/option>/);
    assert.match(html, /id="humanoid-w-metric-increase-btn"/);
    assert.doesNotMatch(html, /id="humanoid-w-metric-decrease-btn"/);
    assert.doesNotMatch(html, />w-</);
    assert.doesNotMatch(html, /<option value="w_abs">w_abs<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_arm">w_abs_arm<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_vfi">w_abs_vfi<\/option>/);
    assert.doesNotMatch(html, /w_abs_hier_vfi_iter/);
    assert.match(html, /仅保留普通可操作度与 VFI 可操作度。/);
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

    assert.match(html, /id="humanoid-waist-x-corridor-toggle"/);
    assert.match(html, /id="humanoid-waist-x-corridor-min-input"/);
    assert.match(html, /id="humanoid-waist-x-corridor-max-input"/);
    assert.match(html, /腰部 X 约束/);
    assert.match(html, /x_min \(m\)：/);
    assert.match(html, /x_max \(m\)：/);
    assert.match(html, /约束腰部 X 位置落在可调走廊区间内/);
    assert.match(html, /id="humanoid-test-dual-arm-lock-toggle"/);
    assert.match(html, /双臂测试锁/);
    assert.match(html, /ljoint2∈\[-0\.1,0\.2\], rjoint2∈\[-0\.2,0\.1\]/);
});

test('index.html exposes the humanoid parameter panel with joint-limit eta and ROS2 publish default inputs', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="parameter-panel"/);
    assert.match(html, /data-panel="parameter-panel"/);
    assert.match(html, /参数面板/);
    assert.match(html, /id="humanoid-joint-limit-eta-input"/);
    assert.match(html, /id="humanoid-joint-limit-eta-status"/);
    assert.match(html, /关节极限 VFI η_θ/);
    assert.match(html, /ROS 2 发布默认值/);
    assert.match(html, /id="humanoid-ros2-arm-vel-input"/);
    assert.match(html, /id="humanoid-ros2-arm-acc-input"/);
    assert.match(html, /id="humanoid-ros2-waist-vel-input"/);
    assert.match(html, /id="humanoid-ros2-waist-acc-input"/);
    assert.match(html, /id="humanoid-ros2-publish-defaults-status"/);
});

test('index.html exposes the humanoid single-arm-plus-waist control modes', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /<option value="robot1-waist">左臂\+腰部<\/option>/);
    assert.match(html, /<option value="robot2-waist">右臂\+腰部<\/option>/);
});

test('index.html removes the old humanoid surface-slide control modes while exposing the new ruled-surface polishing modes', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.doesNotMatch(html, /<option value="surface-slide">/);
    assert.doesNotMatch(html, /<option value="surface-slide-absolute">/);
    assert.doesNotMatch(html, /<option value="dual-independent-surface-slide">/);
    assert.match(html, /<option value="left-ruled-surface-polishing">左臂擦面<\/option>/);
    assert.match(html, /<option value="cooperative-ruled-surface-polishing">双臂协同擦面<\/option>/);
    assert.match(html, /<option value="cooperative-ruled-surface-polishing-dq-setpoint">双臂协同擦面（DQ目标，腰部维持）<\/option>/);
    assert.match(html, /<option value="cooperative-ruled-surface-polishing-dq-setpoint-arm-only">双臂协同擦面（DQ目标，无腰部）<\/option>/);
});

test('index.html exposes humanoid default joint-angle profile buttons', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-default-q-profile-1-btn"/);
    assert.match(html, /id="humanoid-default-q-profile-2-btn"/);
    assert.match(html, /默认关节角/);
    assert.match(html, />默认1</);
    assert.match(html, />默认2</);
});

test('index.html exposes cooperative polishing surface source buttons', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-cooperative-polishing-surface-source-analytic-btn"/);
    assert.match(html, /id="humanoid-cooperative-polishing-surface-source-cylinder-btn"/);
    assert.match(html, /协同擦面示例曲面/);
    assert.match(html, />解析曲面</);
    assert.match(html, />圆筒 Mesh</);
});

test('index.html exposes cooperative polishing object pose controls', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /id="humanoid-cooperative-polishing-object-x-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-y-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-z-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-yaw-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-flip-normal-toggle"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-apply-btn"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-capture-btn"/);
    assert.match(html, /id="humanoid-cooperative-polishing-export-obj-btn"/);
    assert.match(html, /物体位姿/);
    assert.match(html, /法向反转/);
    assert.match(html, />应用位姿</);
    assert.match(html, />捕捉当前双手</);
    assert.match(html, />导出打印 OBJ</);
});
