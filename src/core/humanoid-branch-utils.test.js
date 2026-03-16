import test from 'node:test';
import assert from 'node:assert/strict';

import {
    applyTaskEquivalentBranchHysteresis,
    buildTaskEquivalentBranches,
    computeJointLimitPressure,
    computeMinJointMarginRatio,
    rankTaskEquivalentBranchCandidates,
    selectBestTaskEquivalentBranchCandidate,
    summarizeTaskEquivalentBranchCandidates
} from './humanoid-branch-utils.js';

test('computeJointLimitPressure only activates near joint limits', () => {
    const q = [0.0, 0.92, -0.94];
    const limits = [
        { lower: -1, upper: 1 },
        { lower: -1, upper: 1 },
        { lower: -1, upper: 1 }
    ];

    const result = computeJointLimitPressure(q, limits, { activationRatio: 0.5 });

    assert.equal(result.pressure[0], 0);
    assert.ok(result.pressure[1] < 0, 'upper-limit joint should be pushed down');
    assert.ok(result.pressure[2] > 0, 'lower-limit joint should be pushed up');
    assert.ok(result.norm > 0);
});

test('buildTaskEquivalentBranches returns only real branch when pressure is negligible', () => {
    const q = [0.0, 0.1];
    const limits = [
        { lower: -1, upper: 1 },
        { lower: -1, upper: 1 }
    ];

    const branches = buildTaskEquivalentBranches(q, limits, {
        stepSize: 0.05,
        scales: [0, 0.5, 1.0]
    });

    assert.equal(branches.length, 1);
    assert.equal(branches[0].id, 'real');
    assert.deepEqual(branches[0].qSeed, q);
});

test('buildTaskEquivalentBranches creates progressively stronger phantom branches', () => {
    const q = [0.95, -0.9];
    const limits = [
        { lower: -1, upper: 1 },
        { lower: -1, upper: 1 }
    ];

    const branches = buildTaskEquivalentBranches(q, limits, {
        stepSize: 0.1,
        scales: [0, 0.5, 1.0, 1.5],
        activationRatio: 0.4
    });

    assert.equal(branches.length, 4);
    assert.equal(branches[1].label, 'phantom-0.5');
    assert.ok(branches[1].qSeed[0] < q[0]);
    assert.ok(branches[1].qSeed[1] > q[1]);
    assert.ok(branches[3].minMarginRatio >= branches[1].minMarginRatio);
});

test('computeMinJointMarginRatio reports normalized minimum margin', () => {
    const ratio = computeMinJointMarginRatio(
        [0.8, 0.0],
        [
            { lower: -1, upper: 1 },
            { lower: -2, upper: 2 }
        ]
    );

    assert.ok(Math.abs(ratio - 0.1) < 1e-12);
});

test('rankTaskEquivalentBranchCandidates prioritizes convergence then residual then margin', () => {
    const ranked = rankTaskEquivalentBranchCandidates([
        {
            id: 'real',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 5e-4,
            invariantError: 1e-5,
            minJointMarginRatio: 0.02,
            seedDistance: 0
        },
        {
            id: 'phantom',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 2e-4,
            invariantError: 1e-5,
            minJointMarginRatio: 0.06,
            seedDistance: 0.03
        },
        {
            id: 'bad',
            qpConverged: false,
            maxViolationAfter: 0.1,
            commandResidualNorm: 0.1,
            invariantError: 0.1,
            minJointMarginRatio: 0.2,
            seedDistance: 0.01
        }
    ]);

    assert.equal(ranked[0].id, 'phantom');
    assert.equal(ranked[2].id, 'bad');
});

test('selectBestTaskEquivalentBranchCandidate returns null for empty input', () => {
    assert.equal(selectBestTaskEquivalentBranchCandidate([]), null);
});

test('applyTaskEquivalentBranchHysteresis keeps previous branch when scores are close', () => {
    const result = applyTaskEquivalentBranchHysteresis([
        {
            branchId: 'real',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 2.0e-4,
            invariantError: 1.0e-5,
            minJointMarginRatio: 0.02,
            seedDistance: 0
        },
        {
            branchId: 'branch_1_0',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 2.1e-4,
            invariantError: 1.0e-5,
            minJointMarginRatio: 0.06,
            seedDistance: 0.01
        }
    ], {
        branchId: 'branch_1_0',
        holdStepsRemaining: 2
    }, {
        hysteresisAbsMargin: 5e-5,
        hysteresisRelMargin: 0.3,
        minHoldSteps: 2
    });

    assert.equal(result.selected.branchId, 'branch_1_0');
    assert.equal(result.keptByHysteresis, true);
    assert.equal(result.state.holdStepsRemaining, 1);
});

test('applyTaskEquivalentBranchHysteresis switches when previous branch is clearly worse', () => {
    const result = applyTaskEquivalentBranchHysteresis([
        {
            branchId: 'real',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 1.0e-4,
            invariantError: 1.0e-5,
            minJointMarginRatio: 0.02,
            seedDistance: 0
        },
        {
            branchId: 'branch_1_0',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 2.0e-2,
            invariantError: 1.0e-3,
            minJointMarginRatio: 0.06,
            seedDistance: 0.01
        }
    ], {
        branchId: 'branch_1_0',
        holdStepsRemaining: 2
    }, {
        hysteresisAbsMargin: 1e-5,
        hysteresisRelMargin: 0.05,
        minHoldSteps: 3
    });

    assert.equal(result.selected.branchId, 'real');
    assert.equal(result.switched, true);
    assert.equal(result.state.holdStepsRemaining, 3);
});

test('summarizeTaskEquivalentBranchCandidates renders ranked diagnostic lines', () => {
    const lines = summarizeTaskEquivalentBranchCandidates([
        {
            branchId: 'real',
            branchLabel: 'real',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 1.0e-4,
            invariantError: 1.0e-5,
            minJointMarginRatio: 0.02,
            seedDistance: 0,
            continuityCost: 0,
            score: 1.1e-4
        },
        {
            branchId: 'branch_1_0',
            branchLabel: 'phantom-1',
            qpConverged: true,
            maxViolationAfter: 0,
            commandResidualNorm: 2.0e-4,
            invariantError: 1.0e-5,
            minJointMarginRatio: 0.05,
            seedDistance: 0.01,
            continuityCost: 0.02,
            score: 1.7e-4
        }
    ], {
        selectedBranchId: 'real',
        heldBranchId: 'branch_1_0',
        maxLines: 2
    });

    assert.equal(lines.length, 2);
    assert.match(lines[0], /^\*\s?1 real score=/);
    assert.match(lines[1], /^-H2 phantom-1 score=/);
});
