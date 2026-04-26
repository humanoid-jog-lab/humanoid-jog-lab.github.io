import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidJogNullspaceSafetyRefineObjective,
    evaluateHumanoidJogNullspaceSafetyRefineImprovement
} from './humanoid-jog-safety-refine.js';

test('buildHumanoidJogNullspaceSafetyRefineObjective pushes joints away from both lower and upper limits', () => {
    const objective = buildHumanoidJogNullspaceSafetyRefineObjective({
        width: 3,
        candidates: [
            { source: 'joint_lower', margin: 0.02, gradient: [1, 0, 0] },
            { source: 'joint_upper', margin: 0.05, gradient: [0, -1, 0] },
            { source: 'joint_lower', margin: 0.30, gradient: [0, 0, 1] }
        ],
        jointActivationDistance: 0.15,
        waistActivationDistance: 0.08
    });

    assert.equal(objective.active, true);
    assert.equal(objective.rowCount, 2);
    assert.equal(objective.rows[0].source, 'joint_lower');
    assert.equal(objective.rows[1].source, 'joint_upper');
    assert.ok(objective.gradient[0] > 0, 'lower-limit row should request positive dq');
    assert.ok(objective.gradient[1] < 0, 'upper-limit row should request negative dq');
    assert.equal(objective.gradient[2], 0);
});

test('buildHumanoidJogNullspaceSafetyRefineObjective includes all active joint and waist corridor rows', () => {
    const objective = buildHumanoidJogNullspaceSafetyRefineObjective({
        width: 3,
        candidates: [
            { source: 'joint_lower', margin: 0.12, gradient: [1, 0, 0] },
            { source: 'joint_upper', margin: 0.14, gradient: [0, -1, 0] },
            { source: 'waist_x_lower', margin: 0.03, gradient: [0, 0, 2] },
            { source: 'waist_x_upper', margin: 0.12, gradient: [0, 0, -2] }
        ],
        jointActivationDistance: 0.15,
        waistActivationDistance: 0.08
    });

    assert.equal(objective.active, true);
    assert.equal(objective.rowCount, 3);
    assert.deepEqual(objective.rows.map((row) => row.source), [
        'joint_lower',
        'joint_upper',
        'waist_x_lower'
    ]);
    assert.ok(objective.gradient[2] > 0, 'lower corridor row should push x back upward into the corridor');
});

test('evaluateHumanoidJogNullspaceSafetyRefineImprovement accepts objective decrease even when dmin is unchanged', () => {
    const decision = evaluateHumanoidJogNullspaceSafetyRefineImprovement({
        marginBefore: 0.02,
        marginAfter: 0.02,
        objectiveBefore: 0.10,
        objectiveAfter: 0.06
    });

    assert.equal(decision.improved, true);
    assert.equal(decision.reason, 'objective_improved');
});

test('evaluateHumanoidJogNullspaceSafetyRefineImprovement rejects non-improving safety steps', () => {
    const decision = evaluateHumanoidJogNullspaceSafetyRefineImprovement({
        marginBefore: 0.02,
        marginAfter: 0.019,
        objectiveBefore: 0.10,
        objectiveAfter: 0.10
    });

    assert.equal(decision.improved, false);
    assert.equal(decision.reason, 'not_improved');
});
