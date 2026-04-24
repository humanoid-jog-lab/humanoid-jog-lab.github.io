import test from 'node:test';
import assert from 'node:assert/strict';

import {
    HUMANOID_MJ_RESTORE_WASM_DEFAULTS,
    attemptHumanoidJogMJRestoreWasm,
    buildHumanoidJogMJRestoreWasmRequest,
    buildHumanoidJogMJRestoreWasmEnvelope,
    canUseHumanoidJogMJRestoreWasm,
    normalizeHumanoidJogMJRestoreWasmResult
} from './humanoid-jog-mj-wasm.js';

test('canUseHumanoidJogMJRestoreWasm checks the solver-specific wasm entry point', () => {
    assert.equal(canUseHumanoidJogMJRestoreWasm(null), false);
    assert.equal(canUseHumanoidJogMJRestoreWasm({}), false);
    assert.equal(
        canUseHumanoidJogMJRestoreWasm({ solveAbsoluteTargetMJRestore: () => ({}) }),
        true
    );
    assert.equal(
        canUseHumanoidJogMJRestoreWasm({ solveAbsoluteTargetMJRestore: () => ({}) }, 'mj_restore_tr'),
        false
    );
    assert.equal(
        canUseHumanoidJogMJRestoreWasm({ solveAbsoluteTargetMJRestoreTrustRegion: () => ({}) }, 'mj_restore_tr'),
        true
    );
    assert.equal(
        canUseHumanoidJogMJRestoreWasm(
            { solveCooperativeDQSetpointMJRestoreTrustRegion: () => ({}) },
            'mj_restore_tr',
            'cooperative_dq_setpoint'
        ),
        true
    );
    assert.equal(
        canUseHumanoidJogMJRestoreWasm(
            { solveAbsoluteDQSetpointMJRestoreTrustRegion: () => ({}) },
            'mj_restore_tr',
            'absolute_dq_setpoint'
        ),
        true
    );
});

test('buildHumanoidJogMJRestoreWasmRequest clones arrays and fills defaults', () => {
    const request = buildHumanoidJogMJRestoreWasmRequest({
        qCurrent: [0.1, -0.2, 0.3],
        activeStart: 1,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0).map((_, index) => index),
        relativeTarget: new Array(8).fill(0).map((_, index) => -index),
        leftTarget: new Array(8).fill(0.1),
        rightTarget: new Array(8).fill(-0.1),
        lowerBounds: [-1, -2],
        upperBounds: [1, 2],
        stepBounds: [0.4, 0.5],
        jointLimitVFI: {
            enabled: true,
            rows: [[1, 0], [-1, 0]],
            rhs: [0.1, 0.2],
            count: 2
        },
        waistXCorridor: {
            enabled: true,
            min: -0.01,
            max: 0.05,
            tolerance: 5e-4,
            activationThreshold: 0.12,
            rows: [[1, 0], [0, 1]],
            rhs: [0.01, 0.02]
        },
        safetyPushEnabled: false,
        safetyPushWeight: 0.12,
        safetyPushActivationResidual: 2e-5,
        safetyPushPoseHoldTolerance: 3e-6,
        safetyPushTargetFraction: 0.4,
        safetyPushMaxDistanceStep: 0.03,
        referenceModel: {
            absolute: { stiffness: 1.2 },
            bodyZ: { damping: 0.8 }
        }
    });

    assert.deepEqual(request.qCurrent, [0.1, -0.2, 0.3]);
    assert.equal(request.activeStart, 1);
    assert.equal(request.activeCount, 2);
    assert.equal(request.solveVariant, 'absolute_target');
    assert.deepEqual(request.lowerBounds, [-1, -2]);
    assert.deepEqual(request.upperBounds, [1, 2]);
    assert.deepEqual(request.stepBounds, [0.4, 0.5]);
    assert.deepEqual(request.leftTarget, new Array(8).fill(0.1));
    assert.deepEqual(request.rightTarget, new Array(8).fill(-0.1));
    assert.equal(request.keepBodyZVertical, false);
    assert.equal(request.absoluteWeight, 1);
    assert.equal(request.relativeWeight, 1);
    assert.equal(request.bodyWeight, 1);
    assert.equal(request.wvfiLowerBound, 0);
    assert.equal(request.jointLimitActivationThreshold, HUMANOID_MJ_RESTORE_WASM_DEFAULTS.jointLimitActivationThreshold);
    assert.deepEqual(request.safetyPush, {
        enabled: false,
        weight: 0.12,
        activationResidual: 2e-5,
        poseHoldTolerance: 3e-6,
        targetFraction: 0.4,
        maxDistanceStep: 0.03
    });
    assert.deepEqual(request.jointLimitVFI, {
        enabled: true,
        rows: [[1, 0], [-1, 0]],
        rhs: [0.1, 0.2],
        count: 2
    });
    assert.deepEqual(request.referenceModel, {
        absolute: { stiffness: 1.2, damping: 0.25 },
        relative: { stiffness: 1, damping: 0.35 },
        bodyZ: { stiffness: 0.6, damping: 0.8 }
    });
    assert.deepEqual(request.waistXCorridor, {
        enabled: true,
        min: -0.01,
        max: 0.05,
        tolerance: 5e-4,
        activationThreshold: 0.12,
        rows: [[1, 0], [0, 1]],
        rhs: [0.01, 0.02]
    });
});

test('buildHumanoidJogMJRestoreWasmRequest accepts manifold waist corridor vfiRows shape', () => {
    const request = buildHumanoidJogMJRestoreWasmRequest({
        waistXCorridor: {
            enabled: true,
            min: -0.01,
            max: 0.05,
            tolerance: 5e-4,
            vfiRows: [[1, 0], [0, 1]],
            rhs: [0.01, 0.02]
        }
    });

    assert.deepEqual(request.waistXCorridor.rows, [[1, 0], [0, 1]]);
    assert.deepEqual(request.waistXCorridor.rhs, [0.01, 0.02]);
    assert.equal(request.wvfiLowerBound, 0);
});

test('normalizeHumanoidJogMJRestoreWasmResult safely fills restoration defaults', () => {
    const normalized = normalizeHumanoidJogMJRestoreWasmResult({
        converged: true,
        reason: 'converged',
        iterations: 4,
        acceptedIterations: 3,
        lastAcceptedScale: 0.5,
        lastAcceptedDamping: 1e-4,
        commandResidualNorm: 1e-6,
        relativeResidualNorm: 2e-6,
        bodyResidualNorm: 3e-6,
        inequalityViolation: 0,
        trustRadius: 0.2,
        lastAcceptedRho: 0.8,
        lastPredictedReduction: 0.5,
        lastActualReduction: 0.4,
        acceptedBy: 'filter',
        bestConstraintViolation: 1e-7,
        filterSize: 3,
        referenceResidual: [0.1, 0.2],
        constraintMotion: [0.3, 0.4],
        qApplied: [1, 2],
        secondaryMetric: 'w_abs_hier_vfi',
        secondaryState: 'active',
        secondaryBefore: 0.1,
        secondaryAfter: 0.2,
        secondaryApplied: true,
        safetyStatus: 'active',
        safetyResidualBefore: 0.01,
        safetyResidualAfter: 0.004,
        safetyAcceptedIterations: 2
    }, [9, 9]);

    assert.deepEqual(normalized, {
        converged: true,
        reason: 'converged',
        iterations: 4,
        acceptedIterations: 3,
        lastAcceptedScale: 0.5,
        lastAcceptedDamping: 1e-4,
        commandResidualNorm: 1e-6,
        relativeResidualNorm: 2e-6,
        bodyResidualNorm: 3e-6,
        inequalityViolation: 0,
        trustRadius: 0.2,
        lastAcceptedRho: 0.8,
        lastPredictedReduction: 0.5,
        lastActualReduction: 0.4,
        acceptedBy: 'filter',
        lastRejectedRho: Number.NaN,
        lastRejectedPredictedReduction: Number.NaN,
        lastRejectedActualReduction: Number.NaN,
        lastRejectedReason: '',
        bestConstraintViolation: 1e-7,
        filterSize: 3,
        workerInitMs: Number.NaN,
        solveRoundTripMs: Number.NaN,
        solveMs: Number.NaN,
        referenceResidual: [0.1, 0.2],
        constraintMotion: [0.3, 0.4],
        qApplied: [1, 2],
        secondaryMetric: 'w_abs_hier_vfi',
        secondaryState: 'active',
        secondaryBefore: 0.1,
        secondaryAfter: 0.2,
        secondaryApplied: true,
        safetyStatus: 'active',
        safetyResidualBefore: 0.01,
        safetyResidualAfter: 0.004,
        safetyAcceptedIterations: 2
    });
});

test('normalizeHumanoidJogMJRestoreWasmResult falls back to the provided active joints', () => {
    const normalized = normalizeHumanoidJogMJRestoreWasmResult({}, [0.5, -0.5]);

    assert.deepEqual(normalized, {
        converged: false,
        reason: 'unknown',
        iterations: 0,
        acceptedIterations: 0,
        lastAcceptedScale: 0,
        lastAcceptedDamping: Number.NaN,
        commandResidualNorm: Number.POSITIVE_INFINITY,
        relativeResidualNorm: Number.POSITIVE_INFINITY,
        bodyResidualNorm: 0,
        inequalityViolation: Number.POSITIVE_INFINITY,
        trustRadius: Number.NaN,
        lastAcceptedRho: Number.NaN,
        lastPredictedReduction: Number.NaN,
        lastActualReduction: Number.NaN,
        acceptedBy: '',
        lastRejectedRho: Number.NaN,
        lastRejectedPredictedReduction: Number.NaN,
        lastRejectedActualReduction: Number.NaN,
        lastRejectedReason: '',
        bestConstraintViolation: Number.POSITIVE_INFINITY,
        filterSize: 0,
        workerInitMs: Number.NaN,
        solveRoundTripMs: Number.NaN,
        solveMs: Number.NaN,
        secondaryMetric: '',
        secondaryState: '',
        secondaryBefore: Number.NaN,
        secondaryAfter: Number.NaN,
        secondaryApplied: false,
        safetyStatus: 'skipped',
        safetyResidualBefore: Number.NaN,
        safetyResidualAfter: Number.NaN,
        safetyAcceptedIterations: 0,
        referenceResidual: [],
        constraintMotion: [],
        qApplied: [0.5, -0.5]
    });
});

test('attemptHumanoidJogMJRestoreWasm returns null when the wasm method is unavailable', () => {
    const result = attemptHumanoidJogMJRestoreWasm({
        humanoidModel: {},
        qInitial: [0, 0],
        evaluateProblem: () => ({})
    });

    assert.equal(result, null);
});

test('buildHumanoidJogMJRestoreWasmEnvelope preserves secondary metric attempts', () => {
    const envelope = buildHumanoidJogMJRestoreWasmEnvelope({
        attempt: {
            converged: true,
            reason: 'converged',
            iterations: 4,
            acceptedIterations: 3,
            lastAcceptedScale: 0.5,
            qApplied: [0.2, -0.1],
            currentResidualNorm: 1e-8,
            secondaryMetricAttempt: {
                metric: 'w_abs_hier_vfi',
                state: 'active',
                before: 0.1,
                after: 0.2,
                applied: true
            },
            timings: {
                preMs: 1,
                solveMs: 2,
                postMs: 3
            }
        },
        qInitial: [0.0, 0.0],
        evaluateProblem: () => ({
            equalities: { activeResidualNorm: 1e-8 }
        })
    });

    assert.equal(envelope.converged, true);
    assert.equal(envelope.currentResidualNorm, 1e-8);
    assert.deepEqual(envelope.secondaryMetricAttempt, {
        metric: 'w_abs_hier_vfi',
        state: 'active',
        before: 0.1,
        after: 0.2,
        applied: true
    });
});

test('attemptHumanoidJogMJRestoreWasm prefers wasm and normalizes the restoration result', () => {
    const observed = [];
    const humanoidModel = {
        solveAbsoluteTargetMJRestore(request) {
            observed.push(request);
            return {
                converged: true,
                reason: 'converged',
                iterations: 5,
                acceptedIterations: 4,
                lastAcceptedScale: 0.5,
                commandResidualNorm: 1e-8,
                relativeResidualNorm: 2e-8,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                referenceResidual: [0.1],
                constraintMotion: [0.2],
                qApplied: [0.25, -0.15]
            };
        }
    };
    const evaluateProblem = (q) => ({
        q: q.slice(),
        qCurrent: [0.1, 0.2, 0.3],
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {
            jointLimitVFI: {
                enabled: true,
                rows: [[1, 0], [-1, 0]],
                rhs: [0.1, 0.2],
                count: 2
            },
            waistXCorridor: {
                enabled: true,
                min: -0.01,
                max: 0.05,
                tolerance: 5e-4,
                rows: [[1, 0]],
                rhs: [0.1]
            }
        },
        metadata: {
            qFull: [0.1, q[0] ?? 0, q[1] ?? 0]
        }
    });

    const result = attemptHumanoidJogMJRestoreWasm({
        humanoidModel,
        qInitial: [0.1, -0.2],
        evaluateProblem,
        activeStart: 1,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0),
        relativeTarget: new Array(8).fill(1),
        keepBodyZVertical: true,
        bodyEqualityTolerance: 5e-5,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.4, 0.5],
        wvfiLowerBound: 0.001
    });

    assert.equal(observed.length, 1);
    assert.equal(observed[0].activeStart, 1);
    assert.equal(observed[0].activeCount, 2);
    assert.deepEqual(observed[0].qCurrent, [0.1, 0.1, -0.2]);
    assert.deepEqual(observed[0].jointLimitVFI, {
        enabled: true,
        rows: [[1, 0], [-1, 0]],
        rhs: [0.1, 0.2],
        count: 2
    });
    assert.deepEqual(result.qApplied, [0.25, -0.15]);
    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(result.currentProblem);
    assert.equal(result.currentResidualNorm, 0.4);
    assert.deepEqual(result.lastReferenceResidual, [0.1]);
    assert.deepEqual(result.lastConstraintMotion, [0.2]);
});

test('attemptHumanoidJogMJRestoreWasm dispatches trust-region requests to the dedicated wasm entry point', () => {
    let lineSearchCalled = false;
    let trustRegionCalled = false;
    const humanoidModel = {
        solveAbsoluteTargetMJRestore() {
            lineSearchCalled = true;
            return {};
        },
        solveAbsoluteTargetMJRestoreTrustRegion(request) {
            trustRegionCalled = true;
            assert.equal(request.solverMode, 'mj_restore_tr');
            return {
                converged: false,
                reason: 'best_effort',
                iterations: 6,
                acceptedIterations: 2,
                lastAcceptedScale: 1,
                commandResidualNorm: 1e-3,
                relativeResidualNorm: 2e-3,
                bodyResidualNorm: 0,
                inequalityViolation: 0.0,
                qApplied: [0.1, -0.1]
            };
        }
    };
    const result = attemptHumanoidJogMJRestoreWasm({
        humanoidModel,
        solverMode: 'mj_restore_tr',
        qInitial: [0, 0],
        evaluateProblem: (q) => ({
            q: q.slice(),
            qCurrent: q.slice(),
            equalities: { activeResidualNorm: Math.abs(q[0]) + Math.abs(q[1]) }
        }),
        activeStart: 0,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0),
        relativeTarget: new Array(8).fill(0)
    });

    assert.equal(lineSearchCalled, false);
    assert.equal(trustRegionCalled, true);
    assert.equal(result?.reason, 'best_effort');
});

test('attemptHumanoidJogMJRestoreWasm dispatches cooperative DQ set-point requests to the dedicated wasm entry point', () => {
    let cooperativeCalled = false;
    const humanoidModel = {
        solveCooperativeDQSetpointMJRestoreTrustRegion(request) {
            cooperativeCalled = true;
            assert.equal(request.solveVariant, 'cooperative_dq_setpoint');
            assert.deepEqual(request.leftTarget, new Array(8).fill(0.25));
            assert.deepEqual(request.rightTarget, new Array(8).fill(-0.25));
            return {
                converged: true,
                reason: 'converged',
                iterations: 4,
                acceptedIterations: 3,
                lastAcceptedScale: 1,
                commandResidualNorm: 1e-7,
                relativeResidualNorm: 0,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                qApplied: [0.05, -0.05]
            };
        }
    };

    const result = attemptHumanoidJogMJRestoreWasm({
        humanoidModel,
        solveVariant: 'cooperative_dq_setpoint',
        solverMode: 'mj_restore_tr',
        qInitial: [0, 0],
        evaluateProblem: (q) => ({
            q: q.slice(),
            qCurrent: q.slice(),
            equalities: { activeResidualNorm: Math.abs(q[0]) + Math.abs(q[1]) }
        }),
        activeStart: 0,
        activeCount: 2,
        absoluteTarget: new Array(16).fill(0),
        relativeTarget: [],
        leftTarget: new Array(8).fill(0.25),
        rightTarget: new Array(8).fill(-0.25)
    });

    assert.equal(cooperativeCalled, true);
    assert.equal(result?.reason, 'converged');
});

test('attemptHumanoidJogMJRestoreWasm dispatches absolute DQ set-point requests to the dedicated wasm entry point', () => {
    let absoluteDQSetpointCalled = false;
    const humanoidModel = {
        solveAbsoluteDQSetpointMJRestoreTrustRegion(request) {
            absoluteDQSetpointCalled = true;
            assert.equal(request.solveVariant, 'absolute_dq_setpoint');
            assert.deepEqual(request.absoluteTarget, new Array(8).fill(0.5));
            return {
                converged: true,
                reason: 'converged',
                iterations: 3,
                acceptedIterations: 2,
                lastAcceptedScale: 1,
                commandResidualNorm: 1e-7,
                relativeResidualNorm: 0,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                qApplied: [0.05, -0.05]
            };
        }
    };

    const result = attemptHumanoidJogMJRestoreWasm({
        humanoidModel,
        solveVariant: 'absolute_dq_setpoint',
        solverMode: 'mj_restore_tr',
        qInitial: [0, 0],
        evaluateProblem: (q) => ({
            q: q.slice(),
            qCurrent: q.slice(),
            equalities: { activeResidualNorm: Math.abs(q[0]) + Math.abs(q[1]) }
        }),
        activeStart: 0,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0.5),
        relativeTarget: new Array(8).fill(0)
    });

    assert.equal(absoluteDQSetpointCalled, true);
    assert.equal(result?.reason, 'converged');
});
