import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('app.html only exposes the supported humanoid jog solvers', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.doesNotMatch(html, /<option value="vfi">/);
    assert.doesNotMatch(html, /<option value="priority">/);
    assert.doesNotMatch(html, /<option value="hqp">/);
    assert.doesNotMatch(html, /<option value="sns">/);
    assert.doesNotMatch(html, /<option value="predictive_ik">/);
    assert.match(html, /<option value="proxqp">ProxQP<\/option>/);
    assert.match(html, /<option value="proxqp_w">ProxQP\+w<\/option>/);
    assert.match(html, /<option value="proxqp_w_vfi" selected>ProxQP\+w_vfi<\/option>/);
    assert.doesNotMatch(html, /<option value="wbc_qp">/);
    assert.doesNotMatch(html, /<option value="mj_restore">/);
    assert.doesNotMatch(html, /<option value="mj_restore_tr">/);
    assert.doesNotMatch(html, /<option value="mj_restore_tr_safe">/);
    assert.doesNotMatch(html, /<option value="mj_restore_tr_w">/);
    assert.doesNotMatch(html, /<option value="mj_restore_tr_safe_wvfi">/);
    assert.doesNotMatch(html, /<option value="mj_restore_pre_w">/);
    assert.doesNotMatch(html, /<option value="torso_seed_upright">Torso-Seeded Upright<\/option>/);
    assert.doesNotMatch(html, /<option value="torso_seed_upright_vfi">Torso-Seeded Upright \+ VFI-w<\/option>/);
    assert.doesNotMatch(html, /<option value="cns">/);
    assert.doesNotMatch(html, /<option value="sqp">/);
    assert.doesNotMatch(html, /<option value="manifold">/);
});

test('app.html starts with only a lightweight jog option bar visible', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');
    const css = readFileSync(resolve(ROOT, 'src/styles.css'), 'utf8');

    assert.match(html, /<div class="jog-option-bar" id="jog-option-bar"/);
    assert.match(html, /<div class="floating-panel control-panel hidden" id="control-panel">/);
    assert.match(html, /<div class="floating-panel info-panel hidden" id="info-panel">/);
    assert.match(html, /id="quick-control-mode-select"/);
    assert.match(html, /Control Mode/);
    assert.match(html, /id="quick-jog-solver-select"/);
    assert.match(html, /Jog Solver/);
    assert.match(html, /id="quick-distance-controls"/);
    assert.match(html, /class="teach-distance-btn teach-btn teach-btn-negative quick-jog-btn"/);
    ['x', 'y', 'z', 'rx', 'ry', 'rz'].forEach((axis) => {
        assert.match(html, new RegExp(`data-axis="${axis}"`));
    });
    assert.match(html, /id="teach-speed-percent"/);
    assert.match(html, /id="teach-speed-value"/);
    assert.match(html, /Speed Percentage/);
    assert.match(css, /\.jog-option-bar\s*\{/);
    assert.match(css, /\.quick-jog-controls/);
    assert.match(css, /\.quick-distance-controls\s*\{/);
});

test('app.html starts the teach pendant with English labels and requested default safety settings', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /<h2>Joint Controls<\/h2>/);
    assert.match(html, /Import from Clipboard/);
    assert.match(html, /Copy Joint Angles/);
    assert.match(html, /Switch to Radians/);
    assert.match(html, /Top View/);
    assert.match(html, /<h3>Inverse Kinematics<\/h3>/);
    assert.match(html, /<span class="toggle-label">Show URDF Frames<\/span>/);
    assert.match(html, />Speed Percentage:</);
    assert.match(html, /id="teach-speed-value">25<\/span>/);
    assert.match(html, /id="teach-speed-percent"[\s\S]*value="25"/);
    assert.match(html, /id="humanoid-waist-x-corridor-toggle" checked/);
    assert.match(html, /id="humanoid-waist-x-corridor-min-input"[\s\S]*value="-0\.4"/);
    assert.match(html, /id="humanoid-waist-x-corridor-max-input"[\s\S]*value="-0\.1"/);

    [
        '关节控制',
        '从剪贴板导入',
        '复制关节角度',
        '逆运动学求解',
        '速度百分比',
        '腰部 X 约束',
        '双臂测试锁'
    ].forEach((text) => {
        assert.doesNotMatch(html, new RegExp(`>${text}<|>${text}\\s|\\s${text}<`), text);
    });
});

test('app.html uses English default labels in parameter, DQ calculator, and ROS2 panels', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    [
        'Parameters',
        'Joint-Limit VFI η_θ',
        'Corridor VFI η_x',
        'Value applied',
        'ROS 2 Publish Defaults',
        'Arm vel',
        'Waist acc',
        'DQ Calculator',
        'Dual Quaternion Input',
        'Load Current Quaternion',
        'Load Current Position',
        'Translation Operation',
        'Left Multiply',
        'Right Multiply',
        'Result Output',
        'ROS 2 Communication & Control',
        'Service Address',
        'Save',
        'Operation Mode',
        'Real Joint State',
        'Joint Position Publishing',
        'Topic Name:',
        'Continuous Joint Position Publishing',
        'Message Log'
    ].forEach((text) => {
        assert.match(html, new RegExp(text.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')), text);
    });

    [
        '参数面板',
        '关节极限',
        '当前值已应用',
        '加载当前四元数',
        '加载当前位置',
        '平移操作',
        '左乘',
        '右乘',
        '结果输出',
        'ROS 2 通讯和控制',
        '服务地址设置',
        '关节位置发布',
        '持续发布关节位置',
        '消息日志'
    ].forEach((text) => {
        assert.doesNotMatch(html, new RegExp(`>${text}<|>${text}\\s|\\s${text}<`), text);
    });
});

test('app.html exposes the humanoid w selector surface with only hierarchical metrics and shared buttons', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /id="humanoid-manifold-jog-metric-select"/);
    assert.match(html, /w Metric:/);
    assert.match(html, /<option value="w_abs_hier">w_abs<\/option>/);
    assert.match(html, /<option value="w_abs_hier_ext">w_abs_ext<\/option>/);
    assert.match(html, /<option value="w_abs_hier_vfi" selected>w_abs_vfi<\/option>/);
    assert.match(html, /id="humanoid-w-metric-increase-btn"/);
    assert.doesNotMatch(html, /id="humanoid-w-metric-decrease-btn"/);
    assert.doesNotMatch(html, />w-</);
    assert.doesNotMatch(html, /<option value="w_abs">w_abs<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_arm">w_abs_arm<\/option>/);
    assert.doesNotMatch(html, /<option value="w_abs_vfi">w_abs_vfi<\/option>/);
    assert.doesNotMatch(html, /w_abs_hier_vfi_iter/);
    assert.match(html, /Only standard, extended, and VFI manipulability are kept\./);
});

test('humanoid joint controls no longer expose the differential base control section', () => {
    const source = readFileSync(resolve(ROOT, 'src/ui/joint-controls.js'), 'utf8');

    assert.doesNotMatch(source, /Differential Base/);
    assert.doesNotMatch(source, /Point Stabilization/);
    assert.doesNotMatch(source, /Path Tracking \(RPP\)/);
    assert.doesNotMatch(source, /humanoid-base-point-controls/);
    assert.doesNotMatch(source, /humanoid-base-path-controls/);
    assert.doesNotMatch(source, /rpp-run-btn/);
});

test('app.html hides the humanoid VFI step backend selector and task-equivalent branching toggle', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.doesNotMatch(html, /id="humanoid-vfi-step-backend-select"/);
    assert.doesNotMatch(html, /VFI 步进后端：/);
    assert.doesNotMatch(html, /Projection（原实现）/);
    assert.doesNotMatch(html, /QP（新）/);
    assert.doesNotMatch(html, /id="humanoid-task-equivalent-branching-toggle"/);
    assert.doesNotMatch(html, /任务等价多分支/);
});

test('app.html hides the environment management panel and dock entry', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.doesNotMatch(html, /id="environment-panel"/);
    assert.doesNotMatch(html, /id="environment-toggle"/);
    assert.doesNotMatch(html, /data-panel="environment-panel"/);
    assert.doesNotMatch(html, /id="environment-object-list"/);
    assert.doesNotMatch(html, /id="add-environment-box-btn"/);
    assert.doesNotMatch(html, />环境管理</);
});

test('app.html exposes the humanoid dual-arm test lock toggle', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /id="humanoid-waist-x-corridor-toggle"/);
    assert.match(html, /id="humanoid-waist-x-corridor-min-input"/);
    assert.match(html, /id="humanoid-waist-x-corridor-max-input"/);
    assert.match(html, /Waist X Constraint/);
    assert.match(html, /x_min \(m\):/);
    assert.match(html, /x_max \(m\):/);
    assert.match(html, /Constrain the waist X position to stay inside the adjustable corridor/);
    assert.match(html, /id="humanoid-test-dual-arm-lock-toggle"/);
    assert.match(html, /Dual-Arm Test Lock/);
    assert.match(html, /ljoint2∈\[-0\.1,0\.2\], rjoint2∈\[-0\.2,0\.1\]/);
});

test('app.html exposes the humanoid parameter panel with joint-limit eta and ROS2 publish default inputs', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /id="parameter-panel"/);
    assert.match(html, /data-panel="parameter-panel"/);
    assert.match(html, /Parameters/);
    assert.match(html, /id="humanoid-joint-limit-eta-input"/);
    assert.match(html, /id="humanoid-joint-limit-eta-status"/);
    assert.match(html, /Joint-Limit VFI η_θ/);
    assert.match(html, /id="humanoid-waist-x-eta-input"/);
    assert.match(html, /id="humanoid-waist-x-vfi-status"/);
    assert.match(html, /Corridor VFI η_x/);
    assert.match(html, /id="humanoid-safety-refine-joint-activation-input"/);
    assert.match(html, /Refine Joint Activation Distance \(rad\)/);
    assert.match(html, /id="humanoid-safety-refine-waist-activation-input"/);
    assert.match(html, /Refine Waist Activation Distance \(m\)/);
    assert.doesNotMatch(html, /id="humanoid-waist-x-activation-threshold-input"/);
    assert.doesNotMatch(html, /走廊距离尺度 \(m\)/);
    assert.match(html, /ROS 2 Publish Defaults/);
    assert.match(html, /id="humanoid-ros2-arm-vel-input"/);
    assert.match(html, /id="humanoid-ros2-arm-acc-input"/);
    assert.match(html, /id="humanoid-ros2-waist-vel-input"/);
    assert.match(html, /id="humanoid-ros2-waist-acc-input"/);
    assert.match(html, /id="humanoid-ros2-publish-defaults-status"/);
});

test('app.html exposes the humanoid single-arm-plus-waist control modes', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /<option value="robot1-waist">Left Arm \+ Waist<\/option>/);
    assert.match(html, /<option value="robot2-waist">Right Arm \+ Waist<\/option>/);
});

test('app.html removes the old humanoid surface-slide and cooperative polishing control modes while exposing the DQ polishing modes', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.doesNotMatch(html, /<option value="surface-slide">/);
    assert.doesNotMatch(html, /<option value="surface-slide-absolute">/);
    assert.doesNotMatch(html, /<option value="dual-independent-surface-slide">/);
    assert.match(html, /<option value="left-ruled-surface-polishing">Left Ruled-Surface Polishing<\/option>/);
    assert.doesNotMatch(html, /<option value="cooperative-ruled-surface-polishing">/);
    assert.match(html, /<option value="cooperative-ruled-surface-polishing-dq-setpoint">Cooperative Surface Polishing \(DQ Set-point, Waist Maintained\)<\/option>/);
    assert.match(html, /<option value="cooperative-ruled-surface-polishing-dq-setpoint-arm-only">Cooperative Surface Polishing \(DQ Set-point, Arms Only\)<\/option>/);
});

test('app.html exposes humanoid default joint-angle profile buttons', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /id="humanoid-default-q-profile-1-btn"/);
    assert.match(html, /id="humanoid-default-q-profile-2-btn"/);
    assert.match(html, /Default Joint Angles/);
    assert.match(html, />Default 1</);
    assert.match(html, />Default 2</);
});

test('app.html exposes cooperative polishing surface source buttons', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /id="humanoid-cooperative-polishing-surface-source-analytic-btn"/);
    assert.match(html, /id="humanoid-cooperative-polishing-surface-source-cylinder-btn"/);
    assert.match(html, /Cooperative Polishing Surface Sample/);
    assert.match(html, />Analytic Surface</);
    assert.match(html, />Cylinder Mesh</);
});

test('app.html exposes cooperative polishing object pose controls', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /id="humanoid-cooperative-polishing-object-x-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-y-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-z-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-yaw-input"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-flip-normal-toggle"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-apply-btn"/);
    assert.match(html, /id="humanoid-cooperative-polishing-object-capture-btn"/);
    assert.match(html, /id="humanoid-cooperative-polishing-export-obj-btn"/);
    assert.match(html, /Object Pose/);
    assert.match(html, /Flip Normal/);
    assert.match(html, />Apply Pose</);
    assert.match(html, />Capture Current Hands</);
    assert.match(html, />Export Printable OBJ</);
});
