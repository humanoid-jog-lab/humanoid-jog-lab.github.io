import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';

function loadWorkerExports() {
    const source = readFileSync(
        new URL('../../public/workers/humanoid-jog-mj-worker.js', import.meta.url),
        'utf8'
    );
    const sandbox = {
        self: {},
        console,
        performance: { now: () => 0 }
    };
    vm.createContext(sandbox);
    vm.runInContext(`${source}\n;globalThis.__workerExports = { computeNullspaceStep, attemptSecondaryMetricRefinement, computeVFIAttenuationMatrix, evaluateSecondaryMetricValue, buildSecondaryProblem };`, sandbox);
    return { sandbox, ...sandbox.__workerExports };
}

test('humanoid mj worker keeps joint-limit attenuation sensitive near exact limits instead of saturating early', () => {
    const { computeVFIAttenuationMatrix } = loadWorkerExports();

    const exactLimit = computeVFIAttenuationMatrix([
        { row: [1], distance: 0, threshold: 0.15, source: 'joint_limit' }
    ], 1);
    const tinyRetreat = computeVFIAttenuationMatrix([
        { row: [1], distance: 1e-5, threshold: 0.15, source: 'joint_limit' }
    ], 1);

    assert.equal(exactLimit[0][0], 0);
    assert.ok(tinyRetreat[0][0] > 0);
    assert.ok(tinyRetreat[0][0] < 1e-3);
});

test('humanoid mj worker evaluates hierarchical VFI metrics against task-relative fields when requested', () => {
    const { evaluateSecondaryMetricValue } = loadWorkerExports();

    const cctsm = {
        w_abs_hier: 0.25,
        w_abs_hier_vfi: 0.2,
        w_abs_arm: 0.22,
        w_abs_arm_vfi: 0.18,
        w_abs: 0.25,
        w_abs_vfi: 0.2,
        w_task_hier: 1.6,
        w_task_hier_vfi: 1.4,
        w_task_arm: 1.5,
        w_task_arm_vfi: 1.3,
        w_task: 1.6,
        w_task_vfi: 1.4
    };

    assert.equal(
        evaluateSecondaryMetricValue('w_abs_hier_vfi', cctsm, 'relative'),
        1.4
    );
    assert.equal(
        evaluateSecondaryMetricValue('w_abs_arm_vfi', cctsm, 'relative'),
        1.3
    );
    assert.equal(
        evaluateSecondaryMetricValue('w_abs_hier_vfi', cctsm, 'absolute'),
        0.2
    );
});

test('humanoid mj worker builds relative-mode secondary problems with a relative-twist command manifold and an absolute-twist protection manifold', () => {
    const { buildSecondaryProblem } = loadWorkerExports();

    const model = {
        absolutePoseDualArm: () => [1, 0, 0, 0, 0, 0, 0, 0],
        relativePoseDualArm: () => [1, 0, 0, 0, 0, 0.2, 0, 0.3],
        absolutePoseJacobianDualArm: () => [
            [0.1, 0],
            [0, 0.2],
            [0.3, 0],
            [0, 0.4],
            [0.5, 0],
            [0, 0.6],
            [0.7, 0],
            [0, 0.8]
        ],
        relativePoseJacobianDualArm: () => [
            [0.08, 0],
            [0, 0.16],
            [0.24, 0],
            [0, 0.32],
            [0.4, 0],
            [0, 0.48],
            [0.56, 0],
            [0, 0.64]
        ]
    };

    const problem = buildSecondaryProblem(
        model,
        {},
        {
            qCurrent: [0, 0],
            activeStart: 0,
            activeCount: 2,
            absoluteTarget: [1, 0, 0, 0, 0, 0, 0, 0],
            relativeTarget: [1, 0, 0, 0, 0, 0.2, 0, 0.3],
            lowerBounds: [-1, -1],
            upperBounds: [1, 1],
            jointLimitActivationThreshold: 0.15,
            keepBodyZVertical: false
        },
        {
            metric: 'w_abs_hier_vfi',
            metricMode: 'relative',
            waistActiveCount: 0,
            hierarchicalVfiBlend: 1
        },
        [0, 0]
    );

    assert.equal(problem.metadata.cctsm.taskMetricKind, 'relative_pose');
    assert.equal(problem.metadata.Jcmd.length, 6);
    assert.equal(problem.metadata.protectedRows.length, 6);
    assert.equal(problem.metadata.Jabs.length, 8);
});

test('humanoid mj worker secondary nullspace step can fall back to the wvfi gradient', () => {
    const { computeNullspaceStep } = loadWorkerExports();

    const result = computeNullspaceStep({
        qCurrent: [0, 0],
        equalityJacobian: [[1, 0]],
        objectiveGradient: [0, 0],
        wvfiGradient: [0, 1],
        stepSize: 0.5,
        lowerBounds: [-10, -10],
        upperBounds: [10, 10],
        stepBounds: [1, 1]
    });

    assert.equal(result.accepted, true);
    assert.equal(result.gradientSource, 'wvfi');
    assert.ok(result.step[1] > 0);
    assert.ok(Math.abs(result.step[0]) <= 1e-9);
});

test('humanoid mj worker can prefer the wvfi gradient when the objective step clips into a joint limit', () => {
    const { computeNullspaceStep } = loadWorkerExports();

    const result = computeNullspaceStep({
        qCurrent: [0.001],
        equalityJacobian: [],
        objectiveGradient: [-1],
        wvfiGradient: [1],
        stepSize: 0.01,
        lowerBounds: [0],
        upperBounds: [1],
        stepBounds: [0.1],
        preferWVFIFallbackOnClip: true
    });

    assert.equal(result.accepted, true);
    assert.equal(result.gradientSource, 'wvfi');
    assert.ok(result.step[0] > 0);
    assert.ok(result.qNext[0] > 0.001);
});

test('humanoid mj worker secondary refinement does not skip when only the wvfi gradient is available', () => {
    const { sandbox, attemptSecondaryMetricRefinement } = loadWorkerExports();

    sandbox.computeProtectedMetricGradient = (_model, _dqModule, _request, _secondaryMetricRequest, _baseProblem, metric) => (
        metric === 'w_vfi' ? [0, 1] : [0, 0]
    );
    sandbox.runRestoreFromActive = () => ({
        converged: true,
        reason: 'converged',
        qApplied: [0, 0.2]
    });
    sandbox.buildSecondaryProblem = (_model, _dqModule, _request, _secondaryMetricRequest, qActive) => ({
        qActive: qActive.slice(),
        equalities: { jacobian: [[1, 0]] },
        metadata: {
            cctsm: {
                w_vfi: 0.2,
                w_abs: 0.15,
                w_abs_arm: 0.15,
                w_abs_vfi: 0.2
            }
        }
    });

    const result = attemptSecondaryMetricRefinement(
        {},
        {},
        {
            lowerBounds: [-1, -1],
            upperBounds: [1, 1],
            stepBounds: [1, 1],
            wvfiLowerBound: 0
        },
        {
            stepSize: 0.2,
            restoreMaxIterations: 8
        },
        {
            qActive: [0, 0],
            equalities: { jacobian: [[1, 0]] },
            metadata: {
                cctsm: {
                    w_vfi: 0.1,
                    w_abs: 0.15,
                    w_abs_arm: 0.15,
                    w_abs_vfi: 0.1
                }
            }
        },
        'w_abs_vfi'
    );

    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.nullspaceResult?.gradientSource, 'wvfi');
});

test('humanoid mj worker can accept a wvfi fallback step when the hierarchical fallback metric improves', () => {
    const { sandbox, attemptSecondaryMetricRefinement } = loadWorkerExports();

    sandbox.computeProtectedMetricGradient = (_model, _dqModule, _request, _secondaryMetricRequest, _baseProblem, metric) => (
        metric === 'w_vfi' ? [0, 1] : [0, 0]
    );
    sandbox.runRestoreFromActive = (_model, _request, qActive) => ({
        converged: true,
        reason: 'converged',
        qApplied: qActive.slice()
    });
    sandbox.buildSecondaryProblem = (_model, _dqModule, _request, _secondaryMetricRequest, qActive) => ({
        qActive: qActive.slice(),
        equalities: { jacobian: [] },
        metadata: {
            cctsm: {
                w_abs_arm_vfi: 1 + qActive[1],
                w_abs_arm: 1 - qActive[1],
                w_abs: 1,
                w_abs_vfi: 1 + 2 * qActive[1],
                hierarchicalVfiBlend: 0.5
            }
        }
    });

    const result = attemptSecondaryMetricRefinement(
        {},
        {},
        {
            lowerBounds: [-1, -1],
            upperBounds: [1, 1],
            stepBounds: [1, 1],
            wvfiLowerBound: 0
        },
        {
            metric: 'w_abs_hier_vfi',
            stepSize: 0.2,
            restoreMaxIterations: 8
        },
        {
            qActive: [0, 0],
            equalities: { jacobian: [] },
            metadata: {
                cctsm: {
                    w_abs_arm_vfi: 1,
                    w_abs_arm: 1,
                    w_abs: 1,
                    w_abs_vfi: 1,
                    hierarchicalVfiBlend: 0.5
                }
            }
        },
        'w_abs_arm_vfi'
    );

    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.nullspaceResult?.gradientSource, 'wvfi');
});

test('humanoid mj worker can accept a safety fallback when w_vfi improves and the requested metric regresses only slightly', () => {
    const { sandbox, attemptSecondaryMetricRefinement } = loadWorkerExports();

    sandbox.computeProtectedMetricGradient = (_model, _dqModule, _request, _secondaryMetricRequest, _baseProblem, metric) => (
        metric === 'w_vfi' ? [0, 1] : [0, 0]
    );
    sandbox.runRestoreFromActive = (_model, _request, qActive) => ({
        converged: true,
        reason: 'converged',
        qApplied: qActive.slice()
    });
    sandbox.buildSecondaryProblem = (_model, _dqModule, _request, _secondaryMetricRequest, qActive) => ({
        qActive: qActive.slice(),
        equalities: { jacobian: [] },
        metadata: {
            cctsm: {
                w_abs_arm_vfi: 1 - 5e-7 * qActive[1],
                w_abs_arm: 1 - 5e-7 * qActive[1],
                w_abs: 1 - 5e-7 * qActive[1],
                w_abs_vfi: 1 - 5e-7 * qActive[1],
                w_vfi: 1 + qActive[1],
                hierarchicalVfiBlend: 0.5
            }
        }
    });

    const result = attemptSecondaryMetricRefinement(
        {},
        {},
        {
            lowerBounds: [-1, -1],
            upperBounds: [1, 1],
            stepBounds: [1, 1],
            wvfiLowerBound: 0
        },
        {
            metric: 'w_abs_hier_vfi',
            stepSize: 0.2,
            restoreMaxIterations: 8
        },
        {
            qActive: [0, 0],
            equalities: { jacobian: [] },
            metadata: {
                cctsm: {
                    w_abs_arm_vfi: 1,
                    w_abs_arm: 1,
                    w_abs: 1,
                    w_abs_vfi: 1,
                    w_vfi: 1,
                    hierarchicalVfiBlend: 0.5
                }
            }
        },
        'w_abs_arm_vfi'
    );

    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.nullspaceResult?.gradientSource, 'wvfi');
});
