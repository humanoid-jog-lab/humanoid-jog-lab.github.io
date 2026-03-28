import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogManifoldProblem } from './humanoid-jog-manifold-problem.js';
import {
    buildHumanoidJogManifoldRestorationStep,
    restoreHumanoidJogManifold
} from './humanoid-jog-manifold-restoration.js';

function buildSimpleProblem(q, options = {}) {
    const target = options.target ?? [0];
    const jacobian = options.jacobian ?? [[1]];
    const residual = q.map((value, index) => (target[index] ?? 0) - value);
    const lower = options.lower ?? new Array(q.length).fill(-Infinity);
    const upper = options.upper ?? new Array(q.length).fill(Infinity);
    const current = options.qCurrent ?? new Array(q.length).fill(0);
    const step = options.step ?? new Array(q.length).fill(Infinity);
    const wvfiValue = typeof options.evaluateWVFI === 'function' ? options.evaluateWVFI(q) : Number.NaN;
    const wvfiLowerBound = options.wvfiLowerBound ?? 0;
    const slack = Number.isFinite(wvfiValue) ? wvfiValue - wvfiLowerBound : Number.NaN;
    const violation = Number.isFinite(slack) ? Math.max(0, -slack) : 0;
    return {
        q: q.slice(),
        equalities: {
            residual,
            jacobian,
            residualNorm: Math.hypot(...residual),
            blocks: {
                absolute: { residual },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: {
                enabled: wvfiLowerBound > 0,
                value: wvfiValue,
                lowerBound: wvfiLowerBound,
                slack,
                violation
            },
            waistXCorridor: {
                enabled: false,
                value: Number.NaN,
                violation: 0
            }
        },
        bounds: {
            lower: lower.slice(),
            upper: upper.slice(),
            step: step.slice(),
            effectiveLower: current.map((value, index) => Math.max(lower[index], value - step[index])),
            effectiveUpper: current.map((value, index) => Math.min(upper[index], value + step[index]))
        }
    };
}

test('restoreHumanoidJogManifold converges a simple equality problem', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [1, -2],
        evaluateProblem: (q) => buildSimpleProblem(q, {
            target: [0, 0],
            jacobian: [[1, 0], [0, 1]],
            lower: [-10, -10],
            upper: [10, 10],
            qCurrent: [1, -2],
            step: [10, 10]
        }),
        maxIterations: 8,
        equalityTolerance: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(result.currentResidualNorm <= 1e-9);
    assert.ok(result.acceptedIterations >= 1);
    assert.ok(Math.abs(result.qApplied[0]) <= 1e-9);
    assert.ok(Math.abs(result.qApplied[1]) <= 1e-9);
});

test('restoreHumanoidJogManifold follows the manifold problem signed-pose residual convention', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [0],
        evaluateProblem: (q) => buildHumanoidJogManifoldProblem({
            q,
            qCurrent: [0],
            absoluteTarget: [1],
            relativeTarget: [],
            absolutePose: (state) => [state[0]],
            relativePose: () => [],
            absoluteJacobian: () => [[1]],
            relativeJacobian: () => [],
            lowerBounds: [-10],
            upperBounds: [10],
            stepBounds: [10]
        }),
        maxIterations: 4,
        equalityTolerance: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(Math.abs(result.qApplied[0] - 1) <= 1e-9);
});

test('restoreHumanoidJogManifold accepts near-threshold improvements smaller than 1e-12', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [1.25e-12],
        evaluateProblem: (q) => buildSimpleProblem(q, {
            target: [0],
            jacobian: [[1]],
            lower: [-10],
            upper: [10],
            qCurrent: [1.25e-12],
            step: [3e-13]
        }),
        maxIterations: 4,
        equalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(Math.abs(result.qApplied[0]) <= 1e-12);
});

test('restoreHumanoidJogManifold respects w_vfi floor and returns best_effort after accepted progress', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [2],
        evaluateProblem: (q) => buildSimpleProblem(q, {
            target: [0],
            jacobian: [[1]],
            lower: [-10],
            upper: [10],
            qCurrent: [2],
            step: [10],
            evaluateWVFI: (state) => state[0],
            wvfiLowerBound: 1
        }),
        maxIterations: 4,
        equalityTolerance: 1e-12
    });

    assert.equal(result.converged, false);
    assert.equal(result.reason, 'best_effort');
    assert.ok(result.acceptedIterations >= 1);
    assert.ok(result.qApplied[0] >= 1 - 1e-12);
    assert.ok(result.currentProblem.inequalities.wvfiFloor.violation <= 1e-12);
});

test('restoreHumanoidJogManifold reports line_search_failed when no feasible trial exists', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [1],
        evaluateProblem: (q) => buildSimpleProblem(q, {
            target: [0],
            jacobian: [[1]],
            lower: [-10],
            upper: [10],
            qCurrent: [1],
            step: [10],
            evaluateWVFI: (state) => state[0],
            wvfiLowerBound: 1
        }),
        maxIterations: 3,
        equalityTolerance: 1e-12
    });

    assert.equal(result.converged, false);
    assert.equal(result.reason, 'line_search_failed');
    assert.deepEqual(result.qApplied, [1]);
});

test('restoreHumanoidJogManifold respects effective step bounds and can stop at max_iter', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [0],
        evaluateProblem: (q) => buildSimpleProblem(q, {
            target: [2],
            jacobian: [[1]],
            lower: [-10],
            upper: [10],
            qCurrent: [0],
            step: [0.5]
        }),
        maxIterations: 1,
        equalityTolerance: 1e-12
    });

    assert.equal(result.converged, false);
    assert.equal(result.reason, 'max_iter');
    assert.ok(result.qApplied[0] <= 0.5 + 1e-12);
    assert.ok(result.acceptedIterations >= 1);
});

test('restoreHumanoidJogManifold shrinks steps to satisfy waist corridor VFI rows near the boundary', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [0],
        evaluateProblem: (q) => ({
            q: q.slice(),
            equalities: {
                residual: [1 - q[0]],
                jacobian: [[1]],
                residualNorm: Math.abs(1 - q[0]),
                blocks: {
                    absolute: { residual: [1 - q[0]] },
                    relative: { residual: [] },
                    bodyZ: null
                }
            },
            inequalities: {
                wvfiFloor: { violation: 0 },
                waistXCorridor: {
                    enabled: true,
                    value: q[0],
                    violation: 0,
                    vfiRows: [[1]],
                    rhs: [0.3]
                }
            },
            bounds: {
                effectiveLower: [-10],
                effectiveUpper: [10]
            }
        }),
        maxIterations: 1,
        equalityTolerance: 1e-12
    });

    assert.equal(result.reason, 'max_iter');
    assert.equal(result.acceptedIterations, 1);
    assert.ok(result.lastAcceptedScale < 1);
    assert.ok(result.qApplied[0] <= 0.3 + 1e-12);
});

test('buildHumanoidJogManifoldRestorationStep prefers weighted equalities when provided', () => {
    const dq = buildHumanoidJogManifoldRestorationStep({
        q: [0, 0],
        equalities: {
            residual: [1],
            jacobian: [[1, 0]],
            weightedResidual: [1],
            weightedJacobian: [[0, 1]]
        }
    }, { damping: 1e-12 });

    assert.ok(Array.isArray(dq));
    assert.ok(Math.abs(dq[0]) <= 1e-9);
    assert.ok(dq[1] > 0.5);
});

test('buildHumanoidJogManifoldRestorationStep applies reference damping against previous projected motion', () => {
    const undamped = buildHumanoidJogManifoldRestorationStep({
        q: [0],
        equalities: {
            weightedResidual: [1],
            weightedJacobian: [[1]]
        }
    }, {
        damping: 1e-12,
        referenceStiffness: 1,
        referenceDamping: 0,
        previousStep: [0.5]
    });
    const damped = buildHumanoidJogManifoldRestorationStep({
        q: [0],
        equalities: {
            weightedResidual: [1],
            weightedJacobian: [[1]]
        }
    }, {
        damping: 1e-12,
        referenceStiffness: 1,
        referenceDamping: 0.5,
        previousStep: [0.5]
    });

    assert.ok(Array.isArray(undamped));
    assert.ok(Array.isArray(damped));
    assert.ok(damped[0] < undamped[0]);
    assert.ok(Math.abs(undamped[0] - 1) <= 1e-6);
    assert.ok(Math.abs(damped[0] - 0.75) <= 1e-6);
});

test('buildHumanoidJogManifoldRestorationStep does not let reference damping reverse the restoration direction', () => {
    const damped = buildHumanoidJogManifoldRestorationStep({
        q: [0],
        equalities: {
            weightedResidual: [0.1],
            weightedJacobian: [[1]]
        }
    }, {
        damping: 1e-12,
        referenceStiffness: 1,
        referenceDamping: 0.35,
        previousStep: [1]
    });

    assert.ok(Array.isArray(damped));
    assert.ok(damped[0] > 0, `expected positive restoration step, got ${damped[0]}`);
    assert.ok(damped[0] < 0.1, `expected clipped damping to still reduce step size, got ${damped[0]}`);
});

test('restoreHumanoidJogManifold converges from active residuals when a tolerated body row remains in full residuals', () => {
    const result = restoreHumanoidJogManifold({
        qInitial: [0.3, 0.5],
        evaluateProblem: (q) => ({
            q: q.slice(),
            equalities: {
                residual: [-q[0], -q[1], 2e-8],
                jacobian: [[1, 0], [0, 1], [0, 0]],
                residualNorm: Math.hypot(q[0], q[1], 2e-8),
                activeResidual: [-q[0], -q[1]],
                activeJacobian: [[1, 0], [0, 1]],
                activeResidualNorm: Math.hypot(q[0], q[1]),
                weightedResidual: [-q[0], -q[1]],
                weightedJacobian: [[1, 0], [0, 1]]
            },
            inequalities: {
                wvfiFloor: { violation: 0 },
                waistXCorridor: { violation: 0 }
            },
            bounds: {
                effectiveLower: [-10, -10],
                effectiveUpper: [10, 10]
            }
        }),
        maxIterations: 8,
        equalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(result.currentResidualNorm <= 1e-12);
});
