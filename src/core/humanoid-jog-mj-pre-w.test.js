import test from 'node:test';
import assert from 'node:assert/strict';

import {
    getHumanoidJogMJPreWDirection,
    mapHumanoidJogMJPreWOutcome,
    resolveHumanoidJogMJPreWMetric,
    supportsHumanoidJogMJPreWControlMode
} from './humanoid-jog-mj-pre-w.js';

test('supportsHumanoidJogMJPreWControlMode enables supported control modes only', () => {
    assert.equal(supportsHumanoidJogMJPreWControlMode('absolute'), true);
    assert.equal(supportsHumanoidJogMJPreWControlMode('decoupled'), true);
    assert.equal(supportsHumanoidJogMJPreWControlMode('decoupled-upright'), true);

    assert.equal(supportsHumanoidJogMJPreWControlMode('relative'), false);
    assert.equal(supportsHumanoidJogMJPreWControlMode('relative-fullbody'), false);
    assert.equal(supportsHumanoidJogMJPreWControlMode('distance'), false);
    assert.equal(supportsHumanoidJogMJPreWControlMode('distance-fullbody'), false);
    assert.equal(supportsHumanoidJogMJPreWControlMode('unknown'), false);
});

test('resolveHumanoidJogMJPreWMetric accepts only hierarchical pre-w metrics', () => {
    assert.equal(resolveHumanoidJogMJPreWMetric('w_abs_hier'), 'w_abs_hier');
    assert.equal(resolveHumanoidJogMJPreWMetric('w_abs_hier_vfi'), 'w_abs_hier_vfi');

    assert.equal(resolveHumanoidJogMJPreWMetric('w_abs'), 'w_abs_hier');
    assert.equal(resolveHumanoidJogMJPreWMetric('w_abs_vfi'), 'w_abs_hier');
    assert.equal(resolveHumanoidJogMJPreWMetric(''), 'w_abs_hier');
    assert.equal(resolveHumanoidJogMJPreWMetric(null), 'w_abs_hier');
});

test('getHumanoidJogMJPreWDirection always returns increase', () => {
    assert.equal(getHumanoidJogMJPreWDirection(), 'increase');
});

test('mapHumanoidJogMJPreWOutcome maps active, skipped, rejected, and invalid outcomes with explicit q_pre eligibility', () => {
    const cases = [
        {
            name: 'active',
            input: { state: 'active' },
            expected: { state: 'applied', qPreEligible: true }
        },
        {
            name: 'active(primary_only; ...) with accepted candidate',
            input: {
                state: 'active(primary_only; secondary=arm_guard)',
                acceptedCandidate: true
            },
            expected: { state: 'applied', qPreEligible: true }
        },
        {
            name: 'active(primary_only; ...) without accepted candidate',
            input: {
                state: 'active(primary_only; secondary=arm_guard)',
                acceptedCandidate: false
            },
            expected: {
                state: 'skipped(projection_only_no_progress)',
                qPreEligible: false
            }
        },
        {
            name: 'skipped(no_progress)',
            input: { state: 'skipped(no_progress)' },
            expected: { state: 'skipped(no_progress)', qPreEligible: false }
        },
        {
            name: 'skipped(no_gradient)',
            input: { state: 'skipped(no_gradient)' },
            expected: { state: 'skipped(no_gradient)', qPreEligible: false }
        },
        {
            name: 'skipped(projection_only_no_progress)',
            input: { state: 'skipped(projection_only_no_progress)' },
            expected: {
                state: 'skipped(projection_only_no_progress)',
                qPreEligible: false
            }
        },
        {
            name: 'rejected(hard_constraints)',
            input: { state: 'rejected(hard_constraints)' },
            expected: { state: 'rejected(hard_constraints)', qPreEligible: false }
        },
        {
            name: 'thrown',
            input: {
                thrown: true,
                error: new Error('boom')
            },
            expected: { state: 'failed(thrown)', qPreEligible: false }
        },
        {
            name: 'invalid',
            input: null,
            expected: { state: 'failed(invalid_metric_result)', qPreEligible: false }
        }
    ];

    for (const testCase of cases) {
        assert.deepEqual(
            mapHumanoidJogMJPreWOutcome(testCase.input),
            testCase.expected,
            testCase.name
        );
    }
});
