import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogMJDiagnostics } from './humanoid-jog-mj-diagnostics.js';

test('buildHumanoidJogMJDiagnostics exposes reference restoration fields', () => {
    const payload = buildHumanoidJogMJDiagnostics({
        modeTag: 'absolute-target',
        includeWaist: true,
        keepBodyZVertical: true,
        restorationResult: {
            converged: true,
            iterations: 7,
            acceptedIterations: 5,
            currentResidualNorm: 3e-6,
            lastAcceptedScale: 0.5,
            reason: 'converged',
            lastReferenceResidual: [0.1, 0.05],
            lastConstraintMotion: [0.2, 0.1]
        },
        nullspaceResult: {
            accepted: true,
            reason: 'accepted',
            stepNorm: 1e-3
        },
        referenceModel: {
            absolute: { stiffness: 1.0, damping: 0.25 },
            relative: { stiffness: 1.0, damping: 0.35 },
            bodyZ: { stiffness: 0.6, damping: 0.45 }
        },
        jogMetric: 'w_abs_arm',
        jogMetricState: 'active',
        jogMetricBefore: 0.1,
        jogMetricAfter: 0.11,
        jogMetricApplied: true,
        jogMetricAxis: 'x',
        hierarchicalVfiBlend: 1,
        relativeResidualNorm: 1e-12,
        bodyResidualNorm: 2e-6,
        polishState: 'converged',
        polishReason: 'converged',
        polishResidualNorm: 9e-8,
        timingPreMs: 0.4,
        timingWorkerInitMs: 3.2,
        timingSolveWaitMs: 1.1,
        timingSolveMs: 6.5,
        timingPostMs: 0.7,
        timingCCTSMMs: 0.9,
        timingDiagnosticMs: 0.3,
        timingTotalMs: 8.8,
        vfiJointLimitCount: 2,
        vfiExtraCount: 1,
        vfiTotalCount: 3,
        vfiMinDistance: 0.012,
        wvfiFloorEnabled: true,
        wvfiFloorValue: 0.018,
        wvfiFloorLowerBound: 0.03,
        wvfiFloorSlack: -0.012,
        wvfiFloorViolation: 0.012,
        waistXEnabled: true,
        waistXPx: 0.041,
        waistXDupper: 0.009,
        waistXDlower: 0.141,
        waistXViolation: 0
    });

    assert.equal(payload.solverMode, 'mj_restore');
    assert.equal(payload.stopReason, 'converged');
    assert.equal(payload.referenceAbsoluteStiffness, 1.0);
    assert.equal(payload.referenceBodyDamping, 0.45);
    assert.equal(payload.nullspaceAccepted, true);
    assert.equal(payload.jogMetric, 'w_abs_arm');
    assert.equal(payload.hierarchicalVfiBlend, 1);
    assert.equal(payload.polishState, 'converged');
    assert.equal(payload.polishReason, 'converged');
    assert.equal(payload.timingWorkerInitMs, 3.2);
    assert.equal(payload.timingSolveWaitMs, 1.1);
    assert.equal(payload.timingSolveMs, 6.5);
    assert.equal(payload.timingTotalMs, 8.8);
    assert.equal(payload.vfiJointLimitCount, 2);
    assert.equal(payload.vfiExtraCount, 1);
    assert.equal(payload.vfiTotalCount, 3);
    assert.equal(payload.vfiMinDistance, 0.012);
    assert.equal(payload.wvfiFloorEnabled, true);
    assert.equal(payload.wvfiFloorValue, 0.018);
    assert.equal(payload.wvfiFloorLowerBound, 0.03);
    assert.equal(payload.wvfiFloorSlack, -0.012);
    assert.equal(payload.wvfiFloorViolation, 0.012);
    assert.equal(payload.waistXEnabled, true);
    assert.equal(payload.waistXPx, 0.041);
    assert.equal(payload.waistXDupper, 0.009);
    assert.equal(payload.waistXDlower, 0.141);
    assert.equal(payload.waistXViolation, 0);
    assert.ok(payload.referenceSummary.includes('abs(k=1.00,b=0.25)'));
});

test('buildHumanoidJogMJDiagnostics exposes mj_restore_pre_w diagnostics for applied, skipped, and disabled cases', () => {
    const applied = buildHumanoidJogMJDiagnostics({
        solverMode: 'mj_restore_pre_w',
        modeTag: 'absolute-target',
        preWState: 'applied',
        preWMetric: 'w_abs_hier_vfi',
        preWStart: 'q_pre'
    });

    assert.equal(applied.solverMode, 'mj_restore_pre_w');
    assert.equal(applied.preWState, 'applied');
    assert.equal(applied.preWMetric, 'w_abs_hier_vfi');
    assert.equal(applied.preWStart, 'q_pre');

    const skipped = buildHumanoidJogMJDiagnostics({
        solverMode: 'mj_restore_pre_w',
        modeTag: 'absolute-target',
        preWState: 'skipped(no_progress)',
        preWMetric: 'w_abs_hier_vfi',
        preWStart: 'q0'
    });

    assert.equal(skipped.solverMode, 'mj_restore_pre_w');
    assert.equal(skipped.preWState, 'skipped(no_progress)');
    assert.equal(skipped.preWMetric, 'w_abs_hier_vfi');
    assert.equal(skipped.preWStart, 'q0');

    const disabled = buildHumanoidJogMJDiagnostics({
        solverMode: 'mj_restore_pre_w',
        modeTag: 'relative-target',
        preWState: 'disabled_for_mode',
        preWMetric: 'w_abs_hier_vfi',
        preWStart: 'q0'
    });

    assert.equal(disabled.solverMode, 'mj_restore_pre_w');
    assert.equal(disabled.preWState, 'disabled_for_mode');
    assert.equal(disabled.preWMetric, 'w_abs_hier_vfi');
    assert.equal(disabled.preWStart, 'q0');
});

test('buildHumanoidJogMJDiagnostics derives task-aware metric labels for jog and pre-w diagnostics', () => {
    const payload = buildHumanoidJogMJDiagnostics({
        solverMode: 'mj_restore_pre_w',
        modeTag: 'relative-target',
        jogMetric: 'w_abs_hier_vfi',
        preWMetric: 'w_abs_hier'
    });

    assert.equal(payload.taskMetricLabel, 'w_rel_vfi');
    assert.equal(payload.taskPreWMetricLabel, 'w_rel');
});

test('buildHumanoidJogMJDiagnostics preserves the trust-region restoration solver id and diagnostics', () => {
    const payload = buildHumanoidJogMJDiagnostics({
        solverMode: 'mj_restore_tr',
        modeTag: 'absolute-target',
        restorationResult: {
            converged: false,
            reason: 'best_effort',
            trustRadius: 0.25,
            lastAcceptedRho: 0.82,
            lastPredictedReduction: 0.6,
            lastActualReduction: 0.49,
            acceptedBy: 'filter',
            lastRejectedRho: -0.15,
            lastRejectedPredictedReduction: 0.08,
            lastRejectedActualReduction: -0.01,
            lastRejectedReason: 'poor_ratio',
            safetyStatus: 'active',
            safetyResidualBefore: 0.012,
            safetyResidualAfter: 0.004,
            safetyAcceptedIterations: 2
        }
    });

    assert.equal(payload.solverMode, 'mj_restore_tr');
    assert.equal(payload.stopReason, 'best_effort');
    assert.equal(payload.restoreAcceptedRho, 0.82);
    assert.equal(payload.restorePredictedReduction, 0.6);
    assert.equal(payload.restoreActualReduction, 0.49);
    assert.equal(payload.restoreAcceptedBy, 'filter');
    assert.equal(payload.restoreRejectedRho, -0.15);
    assert.equal(payload.restoreRejectedPredictedReduction, 0.08);
    assert.equal(payload.restoreRejectedActualReduction, -0.01);
    assert.equal(payload.restoreRejectedReason, 'poor_ratio');
    assert.equal(payload.trSafetyStatus, 'active');
    assert.equal(payload.safetyResidualBefore, 0.012);
    assert.equal(payload.safetyResidualAfter, 0.004);
    assert.equal(payload.safetyAcceptedIterations, 2);
});

test('buildHumanoidJogMJDiagnostics falls back to the supported mj_restore solver id for removed variants', () => {
    const payload = buildHumanoidJogMJDiagnostics({
        solverMode: 'torso_seed_upright_vfi',
        modeTag: 'absolute-target'
    });

    assert.equal(payload.solverMode, 'mj_restore');
});
