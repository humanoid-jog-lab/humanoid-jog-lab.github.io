import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildEqualityToleranceCandidates,
    shouldRetryWithoutSecondaryObjective
} from './vfi-step-secondary-guard.js';

test('shouldRetryWithoutSecondaryObjective stays off for clean equality QPs', () => {
    assert.equal(shouldRetryWithoutSecondaryObjective({
        useInvariantEqualityInQP: true,
        hasSecondaryObjective: true,
        qpConverged: true,
        equalityToleranceRelaxed: false,
        usedSoftInvariantFallback: false
    }), false);
});

test('shouldRetryWithoutSecondaryObjective retries when equality tolerance had to be relaxed', () => {
    assert.equal(shouldRetryWithoutSecondaryObjective({
        useInvariantEqualityInQP: true,
        hasSecondaryObjective: true,
        qpConverged: true,
        equalityToleranceRelaxed: true,
        usedSoftInvariantFallback: false
    }), true);
});

test('shouldRetryWithoutSecondaryObjective retries when equality QP fell back to soft invariants', () => {
    assert.equal(shouldRetryWithoutSecondaryObjective({
        useInvariantEqualityInQP: true,
        hasSecondaryObjective: true,
        qpConverged: true,
        equalityToleranceRelaxed: false,
        usedSoftInvariantFallback: true
    }), true);
});

test('shouldRetryWithoutSecondaryObjective ignores equality fallback signals when there is no secondary objective', () => {
    assert.equal(shouldRetryWithoutSecondaryObjective({
        useInvariantEqualityInQP: true,
        hasSecondaryObjective: false,
        qpConverged: true,
        equalityToleranceRelaxed: true,
        usedSoftInvariantFallback: true
    }), false);
});

test('buildEqualityToleranceCandidates keeps only the base equality tolerance', () => {
    assert.deepEqual(buildEqualityToleranceCandidates(1e-12), [1e-12]);
});

test('buildEqualityToleranceCandidates does not add relaxed equality tolerances', () => {
    assert.deepEqual(buildEqualityToleranceCandidates(1e-13, 1e-12), [1e-13]);
});
