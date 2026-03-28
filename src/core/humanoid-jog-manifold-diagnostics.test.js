import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidJogManifoldDiagnostics,
    buildHumanoidJogManifoldOptimizationLabel,
    buildHumanoidJogManifoldWvfiOptimizationLabel
} from './humanoid-jog-manifold-diagnostics.js';

test('buildHumanoidJogManifoldDiagnostics exposes restoration and nullspace status', () => {
    const payload = buildHumanoidJogManifoldDiagnostics({
        modeTag: 'absolute-target',
        includeWaist: true,
        keepBodyZVertical: true,
        jointLimitAvoidance: true,
        restorationResult: {
            converged: true,
            iterations: 7,
            acceptedIterations: 4,
            currentResidualNorm: 3e-6,
            lastAcceptedScale: 0.5,
            reason: 'best_effort'
        },
        nullspaceResult: {
            accepted: true,
            reason: 'accepted',
            stepNorm: 1.2e-3
        },
        restoredWVFI: 0.14,
        refinedWVFI: 0.19,
        wvfiOptimizationApplied: true,
        jogMetric: 'w_abs_hier',
        jogMetricState: 'active',
        jogMetricBefore: 0.08,
        jogMetricAfter: 0.11,
        jogMetricApplied: true,
        jogMetricAxis: 'x',
        fallbackReason: 'body_already_satisfied',
        relativeResidualNorm: 4e-12,
        bodyResidualNorm: 3e-6,
        wvfiFloorEnabled: true,
        wvfiFloorValue: 0.014,
        wvfiFloorLowerBound: 0.02,
        wvfiFloorSlack: -0.006,
        wvfiFloorViolation: 0.006,
        waistXEnabled: true,
        waistXPx: 0.042,
        waistXDupper: 0.008,
        waistXDlower: 0.142,
        waistXViolation: 0,
        cctsmResult: {
            w: 0.18,
            w_cmd: 0.11,
            w_abs: 0.09,
            w_abs_hier: 0.09,
            w_abs_hier_vfi: 0.085,
            w_abs_arm: 0.075,
            w_abs_vfi: 0.08,
            w_vfi: 0.14
        }
    });

    assert.equal(payload.solverMode, 'manifold');
    assert.equal(payload.modeTag, 'absolute-target');
    assert.equal(payload.message, '约束流形 诊断已更新。');
    assert.equal(payload.restoreIterations, 7);
    assert.equal(payload.restoreAcceptedIterations, 4);
    assert.equal(payload.restoreResidualNorm, 3e-6);
    assert.equal(payload.restoreScale, 0.5);
    assert.equal(payload.stopReason, 'best_effort');
    assert.equal(payload.nullspaceAccepted, true);
    assert.equal(payload.nullspaceReason, 'accepted');
    assert.equal(payload.nullspaceStepNorm, 1.2e-3);
    assert.equal(payload.wvfiOptimizationState, 'active');
    assert.equal(payload.restoredWVFI, 0.14);
    assert.equal(payload.refinedWVFI, 0.19);
    assert.equal(payload.wvfiOptimizationApplied, true);
    assert.equal(payload.jogMetric, 'w_abs_hier');
    assert.equal(payload.jogMetricState, 'active');
    assert.equal(payload.jogMetricBefore, 0.08);
    assert.equal(payload.jogMetricAfter, 0.11);
    assert.equal(payload.jogMetricApplied, true);
    assert.equal(payload.jogMetricAxis, 'x');
    assert.equal(payload.fallbackReason, 'body_already_satisfied');
    assert.equal(payload.relativeResidualNorm, 4e-12);
    assert.equal(payload.bodyResidualNorm, 3e-6);
    assert.equal(payload.wvfiFloorEnabled, true);
    assert.equal(payload.wvfiFloorValue, 0.014);
    assert.equal(payload.wvfiFloorLowerBound, 0.02);
    assert.equal(payload.wvfiFloorSlack, -0.006);
    assert.equal(payload.wvfiFloorViolation, 0.006);
    assert.equal(payload.waistXEnabled, true);
    assert.equal(payload.waistXPx, 0.042);
    assert.equal(payload.waistXDupper, 0.008);
    assert.equal(payload.waistXDlower, 0.142);
    assert.equal(payload.waistXViolation, 0);
    assert.equal(payload.cctsmW, 0.18);
    assert.equal(payload.cctsmWCmd, 0.11);
    assert.equal(payload.cctsmWAbs, 0.09);
    assert.equal(payload.cctsmWAbsHier, 0.09);
    assert.equal(payload.cctsmWAbsHierVfi, 0.085);
    assert.equal(payload.cctsmWAbsArm, 0.075);
    assert.equal(payload.cctsmWAbsVfi, 0.08);
    assert.equal(payload.cctsmWvfi, 0.14);
});

test('buildHumanoidJogManifoldWvfiOptimizationLabel formats a standalone high-precision line', () => {
    const label = buildHumanoidJogManifoldWvfiOptimizationLabel({
        state: 'active',
        before: 0.007,
        after: 0.0071234,
        applied: true
    });

    assert.equal(label, 'wvfi_opt: active  before=0.007000  after=0.007123  applied=yes');
});

test('buildHumanoidJogManifoldOptimizationLabel formats a standalone plain manipulability line', () => {
    const label = buildHumanoidJogManifoldOptimizationLabel({
        metric: 'w_cmd',
        state: 'active',
        before: 0.005,
        after: 0.0054,
        applied: true
    });

    assert.equal(label, 'w_cmd_opt: active  before=0.005000  after=0.005400  applied=yes');
});

test('buildHumanoidJogManifoldOptimizationLabel formats a standalone absolute manipulability line', () => {
    const label = buildHumanoidJogManifoldOptimizationLabel({
        metric: 'w_abs',
        state: 'active',
        before: 0.002,
        after: 0.0025,
        applied: true
    });

    assert.equal(label, 'w_abs_opt: active  before=0.002000  after=0.002500  applied=yes');
});

test('buildHumanoidJogManifoldOptimizationLabel formats a standalone arm-only absolute manipulability line', () => {
    const label = buildHumanoidJogManifoldOptimizationLabel({
        metric: 'w_abs_arm',
        state: 'active',
        before: 0.0012,
        after: 0.0017,
        applied: true
    });

    assert.equal(label, 'w_abs_arm_opt: active  before=0.001200  after=0.001700  applied=yes');
});

test('buildHumanoidJogManifoldOptimizationLabel formats a standalone hierarchical absolute manipulability line', () => {
    const label = buildHumanoidJogManifoldOptimizationLabel({
        metric: 'w_abs_hier',
        state: 'active',
        before: 0.0018,
        after: 0.0021,
        applied: true
    });

    assert.equal(label, 'w_abs_hier_opt: active  before=0.001800  after=0.002100  applied=yes');
});

test('buildHumanoidJogManifoldOptimizationLabel formats a standalone absolute VFI manipulability line', () => {
    const label = buildHumanoidJogManifoldOptimizationLabel({
        metric: 'w_abs_vfi',
        state: 'active',
        before: 0.0015,
        after: 0.0019,
        applied: true
    });

    assert.equal(label, 'w_abs_vfi_opt: active  before=0.001500  after=0.001900  applied=yes');
});

test('buildHumanoidJogManifoldOptimizationLabel formats a standalone hierarchical absolute VFI manipulability line', () => {
    const label = buildHumanoidJogManifoldOptimizationLabel({
        metric: 'w_abs_hier_vfi',
        state: 'active',
        before: 0.0016,
        after: 0.0022,
        applied: true
    });

    assert.equal(label, 'w_abs_hier_vfi_opt: active  before=0.001600  after=0.002200  applied=yes');
});

test('buildHumanoidJogManifoldOptimizationLabel formats a standalone directional manipulability line with axis', () => {
    const label = buildHumanoidJogManifoldOptimizationLabel({
        metric: 'sigma',
        state: 'active',
        axisLabel: 'x',
        before: 0.003,
        after: 0.00325,
        applied: true
    });

    assert.equal(label, 'sigma_opt: active  axis=x  before=0.003000  after=0.003250  applied=yes');
});

test('buildHumanoidJogManifoldDiagnostics tolerates missing optional values', () => {
    const payload = buildHumanoidJogManifoldDiagnostics({
        modeTag: 'absolute-target',
        restorationResult: {
            reason: 'line_search_failed'
        }
    });

    assert.equal(payload.solverMode, 'manifold');
    assert.equal(payload.stopReason, 'line_search_failed');
    assert.equal(payload.nullspaceAccepted, false);
    assert.equal(payload.nullspaceReason, null);
    assert.equal(payload.wvfiOptimizationState, 'skipped(waiting_for_restore)');
    assert.equal(payload.wvfiOptimizationApplied, false);
    assert.equal(payload.fallbackReason, null);
    assert.ok(Number.isNaN(payload.restoreResidualNorm));
    assert.ok(Number.isNaN(payload.restoredWVFI));
    assert.ok(Number.isNaN(payload.refinedWVFI));
    assert.ok(Number.isNaN(payload.relativeResidualNorm));
    assert.ok(Number.isNaN(payload.bodyResidualNorm));
    assert.ok(Number.isNaN(payload.cctsmW));
    assert.ok(Number.isNaN(payload.cctsmWCmd));
    assert.ok(Number.isNaN(payload.cctsmWAbs));
    assert.ok(Number.isNaN(payload.cctsmWAbsHier));
    assert.ok(Number.isNaN(payload.cctsmWAbsHierVfi));
    assert.ok(Number.isNaN(payload.cctsmWAbsArm));
    assert.ok(Number.isNaN(payload.cctsmWAbsVfi));
    assert.ok(Number.isNaN(payload.cctsmWvfi));
});
