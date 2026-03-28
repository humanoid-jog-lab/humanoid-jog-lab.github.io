import test from 'node:test';
import assert from 'node:assert/strict';

import { enforceWVFILowerBound, selectWVFILowerBound } from './wvfi-guard.js';

test('enforceWVFILowerBound accepts the original step when it already satisfies the lower bound', () => {
    const result = enforceWVFILowerBound({
        dqCandidate: [1, -2],
        lowerBound: 0.1,
        evaluateWVFI: () => 0.2,
        backtrackScales: [1, 0.5, 0.25]
    });

    assert.equal(result.accepted, true);
    assert.equal(result.reason, 'accepted');
    assert.equal(result.scale, 1);
    assert.deepEqual(result.dq, [1, -2]);
    assert.equal(result.wvfi, 0.2);
});

test('enforceWVFILowerBound backtracks the step until the exact w_vfi lower bound is satisfied', () => {
    const result = enforceWVFILowerBound({
        dqCandidate: [1, 0],
        lowerBound: 0.1,
        evaluateWVFI: (dq) => (Math.abs(dq[0]) <= 0.5 ? 0.12 : 0.05),
        backtrackScales: [1, 0.5, 0.25]
    });

    assert.equal(result.accepted, true);
    assert.equal(result.reason, 'scaled');
    assert.equal(result.scale, 0.5);
    assert.deepEqual(result.dq, [0.5, 0]);
    assert.equal(result.wvfi, 0.12);
    assert.equal(result.backtracks, 1);
});

test('enforceWVFILowerBound rejects the step when no backtracked candidate satisfies the lower bound', () => {
    const result = enforceWVFILowerBound({
        dqCandidate: [1, 0],
        lowerBound: 0.1,
        evaluateWVFI: () => 0.01,
        backtrackScales: [1, 0.5, 0.25]
    });

    assert.equal(result.accepted, false);
    assert.equal(result.reason, 'below_lower_bound');
    assert.equal(result.scale, 0);
    assert.equal(result.dq, null);
    assert.equal(result.wvfi, 0.01);
});

test('selectWVFILowerBound uses a stricter floor when the waist participates', () => {
    assert.equal(selectWVFILowerBound({
        includeWaist: true,
        lowerBoundWithWaist: 0.03,
        lowerBoundDefault: 1e-4
    }), 0.03);

    assert.equal(selectWVFILowerBound({
        includeWaist: false,
        lowerBoundWithWaist: 0.03,
        lowerBoundDefault: 1e-4
    }), 1e-4);
});

test('selectWVFILowerBound defaults the non-waist floor to 0.001', () => {
    assert.equal(selectWVFILowerBound({
        includeWaist: false,
        lowerBoundWithWaist: 0.03
    }), 0.001);
});
