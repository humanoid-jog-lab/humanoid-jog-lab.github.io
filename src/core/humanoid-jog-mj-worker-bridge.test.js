import test from 'node:test';
import assert from 'node:assert/strict';

import {
    attemptHumanoidJogMJRestoreWasmWorker,
    buildHumanoidJogMJRestoreWasmRequest
} from './humanoid-jog-mj-wasm.js';

test('attemptHumanoidJogMJRestoreWasmWorker uses the worker result and re-evaluates the applied state', async () => {
    const observed = [];
    const workerClient = {
        async solve({ modelSpec, solverMode, request }) {
            observed.push({ modelSpec, solverMode, request });
            return {
                converged: true,
                reason: 'converged',
                iterations: 3,
                acceptedIterations: 2,
                lastAcceptedScale: 0.5,
                commandResidualNorm: 1e-8,
                relativeResidualNorm: 2e-8,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                solveMs: 4.25,
                workerInitMs: 6.75,
                solveRoundTripMs: 9.5,
                referenceResidual: [0.1],
                constraintMotion: [0.2],
                qApplied: [0.25, -0.15]
            };
        }
    };
    const evaluateProblem = (q) => ({
        qCurrent: [0.1, 0.2, 0.3],
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {
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

    const result = await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient,
        modelSpec: { waistChain: [] },
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
        wvfiLowerBound: 0
    });

    assert.equal(observed.length, 1);
    assert.equal(observed[0].solverMode, 'mj_restore');
    assert.deepEqual(
        observed[0].request,
        buildHumanoidJogMJRestoreWasmRequest({
            qCurrent: [0.1, 0.1, -0.2],
            activeStart: 1,
            activeCount: 2,
            absoluteTarget: new Array(8).fill(0),
            relativeTarget: new Array(8).fill(1),
            keepBodyZVertical: true,
            bodyEqualityTolerance: 5e-5,
            lowerBounds: [-1, -1],
            upperBounds: [1, 1],
            stepBounds: [0.4, 0.5],
            wvfiLowerBound: 0,
            waistXCorridor: {
                enabled: true,
                min: -0.01,
                max: 0.05,
                tolerance: 5e-4,
                rows: [[1, 0]],
                rhs: [0.1]
            }
        })
    );
    assert.deepEqual(result.qApplied, [0.25, -0.15]);
    assert.equal(result.currentResidualNorm, 0.4);
    assert.equal(result.timings.solveMs, 4.25);
    assert.equal(result.timings.workerInitMs, 6.75);
    assert.equal(result.timings.solveWaitMs, 5.25);
    assert.ok(Number.isFinite(result.timings.preMs));
    assert.ok(Number.isFinite(result.timings.postMs));
});

test('attemptHumanoidJogMJRestoreWasmWorker forwards secondary metric requests and normalizes worker secondary state', async () => {
    const observed = [];
    const workerClient = {
        async solve({ modelSpec, solverMode, request, secondaryMetricRequest }) {
            observed.push({ modelSpec, solverMode, request, secondaryMetricRequest });
            return {
                converged: true,
                reason: 'converged',
                iterations: 4,
                acceptedIterations: 3,
                lastAcceptedScale: 0.5,
                commandResidualNorm: 1e-8,
                relativeResidualNorm: 2e-8,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                solveMs: 5.5,
                qApplied: [0.2, -0.1],
                secondaryMetric: 'w_abs_hier_vfi',
                secondaryState: 'active',
                secondaryBefore: 0.1,
                secondaryAfter: 0.2,
                secondaryApplied: true
            };
        }
    };
    const evaluateProblem = (q) => ({
        qCurrent: [0.1, 0.2, 0.3],
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {},
        metadata: {
            qFull: [0.1, q[0] ?? 0, q[1] ?? 0]
        }
    });

    const result = await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient,
        modelSpec: { waistChain: [] },
        qInitial: [0.1, -0.2],
        evaluateProblem,
        activeStart: 1,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0),
        relativeTarget: new Array(8).fill(1),
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.4, 0.5],
        secondaryMetricRequest: {
            metric: 'w_abs_hier_vfi',
            metricMode: 'relative',
            stepSize: 0.01,
            restoreMaxIterations: 12,
            waistActiveCount: 0
        }
    });

    assert.equal(observed.length, 1);
    assert.equal(observed[0].solverMode, 'mj_restore');
    assert.deepEqual(observed[0].secondaryMetricRequest, {
        metric: 'w_abs_hier_vfi',
        metricMode: 'relative',
        stepSize: 0.01,
        restoreMaxIterations: 12,
        waistActiveCount: 0
    });
    assert.deepEqual(result.secondaryMetricAttempt, {
        metric: 'w_abs_hier_vfi',
        state: 'active',
        before: 0.1,
        after: 0.2,
        applied: true
    });
});

test('attemptHumanoidJogMJRestoreWasmWorker forwards trust-region solver mode to the worker bridge', async () => {
    const observed = [];
    const workerClient = {
        async solve(payload) {
            observed.push(payload);
            return {
                converged: false,
                reason: 'best_effort',
                iterations: 3,
                acceptedIterations: 1,
                lastAcceptedScale: 1,
                commandResidualNorm: 1e-3,
                relativeResidualNorm: 2e-3,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                qApplied: [0, 0]
            };
        }
    };
    const evaluateProblem = (q) => ({
        qCurrent: q.slice(),
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {},
        metadata: { qFull: q.slice() }
    });

    await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient,
        modelSpec: { waistChain: [] },
        solverMode: 'mj_restore_tr',
        qInitial: [0, 0],
        evaluateProblem,
        activeStart: 0,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0),
        relativeTarget: new Array(8).fill(0)
    });

    assert.equal(observed.length, 1);
    assert.equal(observed[0].solverMode, 'mj_restore_tr');
    assert.equal(observed[0].solveVariant, 'absolute_target');
    assert.equal(observed[0].request.solverMode, 'mj_restore_tr');
});

test('attemptHumanoidJogMJRestoreWasmWorker forwards cooperative DQ set-point solve variants to the worker bridge', async () => {
    const observed = [];
    const workerClient = {
        async solve(payload) {
            observed.push(payload);
            return {
                converged: true,
                reason: 'converged',
                iterations: 5,
                acceptedIterations: 4,
                lastAcceptedScale: 1,
                commandResidualNorm: 1e-7,
                relativeResidualNorm: 0,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                qApplied: [0.1, -0.1]
            };
        }
    };
    const evaluateProblem = (q) => ({
        qCurrent: q.slice(),
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {},
        metadata: { qFull: q.slice() }
    });

    await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient,
        modelSpec: { waistChain: [] },
        solveVariant: 'cooperative_dq_setpoint',
        solverMode: 'mj_restore_tr',
        qInitial: [0, 0],
        evaluateProblem,
        activeStart: 0,
        activeCount: 2,
        absoluteTarget: new Array(16).fill(0),
        relativeTarget: [],
        leftTarget: new Array(8).fill(0.1),
        rightTarget: new Array(8).fill(-0.1)
    });

    assert.equal(observed.length, 1);
    assert.equal(observed[0].solverMode, 'mj_restore_tr');
    assert.equal(observed[0].solveVariant, 'cooperative_dq_setpoint');
    assert.equal(observed[0].request.solveVariant, 'cooperative_dq_setpoint');
    assert.deepEqual(observed[0].request.leftTarget, new Array(8).fill(0.1));
    assert.deepEqual(observed[0].request.rightTarget, new Array(8).fill(-0.1));
});

test('attemptHumanoidJogMJRestoreWasmWorker forwards absolute DQ set-point solve variants to the worker bridge', async () => {
    const observed = [];
    const workerClient = {
        async solve(payload) {
            observed.push(payload);
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
                qApplied: [0.1, -0.1]
            };
        }
    };
    const evaluateProblem = (q) => ({
        qCurrent: q.slice(),
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {},
        metadata: { qFull: q.slice() }
    });

    await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient,
        modelSpec: { waistChain: [] },
        solveVariant: 'absolute_dq_setpoint',
        solverMode: 'mj_restore_tr',
        qInitial: [0, 0],
        evaluateProblem,
        activeStart: 0,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0.2),
        relativeTarget: new Array(8).fill(0)
    });

    assert.equal(observed.length, 1);
    assert.equal(observed[0].solverMode, 'mj_restore_tr');
    assert.equal(observed[0].solveVariant, 'absolute_dq_setpoint');
    assert.equal(observed[0].request.solveVariant, 'absolute_dq_setpoint');
    assert.deepEqual(observed[0].request.absoluteTarget, new Array(8).fill(0.2));
});

test('attemptHumanoidJogMJRestoreWasmWorker preserves direct w_abs_arm worker metric state', async () => {
    const workerClient = {
        async solve() {
            return {
                converged: true,
                reason: 'converged',
                iterations: 2,
                acceptedIterations: 2,
                lastAcceptedScale: 1,
                commandResidualNorm: 1e-8,
                relativeResidualNorm: 1e-8,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                solveMs: 3,
                qApplied: [0.2, -0.1],
                secondaryMetric: 'w_abs_arm',
                secondaryState: 'active',
                secondaryBefore: 0.05,
                secondaryAfter: 0.08,
                secondaryApplied: true
            };
        }
    };
    const evaluateProblem = (q) => ({
        qCurrent: [0.1, 0.2, 0.3],
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {},
        metadata: {
            qFull: [0.1, q[0] ?? 0, q[1] ?? 0]
        }
    });

    const result = await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient,
        modelSpec: { waistChain: [] },
        qInitial: [0.1, -0.2],
        evaluateProblem,
        activeStart: 1,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0),
        relativeTarget: new Array(8).fill(1),
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.4, 0.5],
        secondaryMetricRequest: {
            metric: 'w_abs_arm',
            stepSize: 0.01,
            restoreMaxIterations: 12,
            waistActiveCount: 0
        }
    });

    assert.deepEqual(result.secondaryMetricAttempt, {
        metric: 'w_abs_arm',
        state: 'active',
        before: 0.05,
        after: 0.08,
        applied: true
    });
});

test('buildHumanoidJogMJRestoreWasmRequest forwards joint-limit activation threshold for worker-side VFI attenuation', () => {
    const request = buildHumanoidJogMJRestoreWasmRequest({
        qCurrent: [0.1, 0.2, 0.3],
        activeStart: 1,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0),
        relativeTarget: new Array(8).fill(1),
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.4, 0.5],
        jointLimitActivationThreshold: 0.15
    });

    assert.equal(request.jointLimitActivationThreshold, 0.15);
});

test('attemptHumanoidJogMJRestoreWasmWorker preserves hierarchical w_abs_hier worker metric state', async () => {
    const workerClient = {
        async solve() {
            return {
                converged: true,
                reason: 'converged',
                iterations: 2,
                acceptedIterations: 2,
                lastAcceptedScale: 1,
                commandResidualNorm: 1e-8,
                relativeResidualNorm: 1e-8,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                solveMs: 3,
                qApplied: [0.2, -0.1],
                secondaryMetric: 'w_abs_hier',
                secondaryState: 'active',
                secondaryBefore: 0.05,
                secondaryAfter: 0.08,
                secondaryApplied: true
            };
        }
    };
    const evaluateProblem = (q) => ({
        qCurrent: [0.1, 0.2, 0.3],
        equalities: { activeResidualNorm: Math.abs(q[0] ?? 0) + Math.abs(q[1] ?? 0) },
        inequalities: {},
        metadata: {
            qFull: [0.1, q[0] ?? 0, q[1] ?? 0]
        }
    });

    const result = await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient,
        modelSpec: { waistChain: [] },
        qInitial: [0.1, -0.2],
        evaluateProblem,
        activeStart: 1,
        activeCount: 2,
        absoluteTarget: new Array(8).fill(0),
        relativeTarget: new Array(8).fill(1),
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [0.4, 0.5],
        secondaryMetricRequest: {
            metric: 'w_abs_hier',
            stepSize: 0.01,
            restoreMaxIterations: 12,
            waistActiveCount: 0
        }
    });

    assert.deepEqual(result.secondaryMetricAttempt, {
        metric: 'w_abs_hier',
        state: 'active',
        before: 0.05,
        after: 0.08,
        applied: true
    });
});

test('attemptHumanoidJogMJRestoreWasmWorker returns null without a worker client or model spec', async () => {
    const evaluateProblem = () => ({});

    await assert.doesNotReject(async () => {
        const noClient = await attemptHumanoidJogMJRestoreWasmWorker({
            workerClient: null,
            modelSpec: { waistChain: [] },
            qInitial: [0],
            evaluateProblem
        });
        assert.equal(noClient, null);
    });

    const noModelSpec = await attemptHumanoidJogMJRestoreWasmWorker({
        workerClient: {},
        modelSpec: null,
        qInitial: [0],
        evaluateProblem
    });
    assert.equal(noModelSpec, null);
});
