import test from 'node:test';
import assert from 'node:assert/strict';

import {
    DEFAULT_HUMANOID_IGNORED_JOINTS,
    mapHumanoidJointPositions
} from './joint-state-mapping.js';

test('mapHumanoidJointPositions updates humanoid joints by name and ignores wheel joints', () => {
    const result = mapHumanoidJointPositions({
        messageNames: [
            'left_wheel_joint',
            'ljoint1',
            'right_wheel_joint',
            'rjoint1'
        ],
        positions: [12.3, 0.5, -9.9, -0.2],
        jointNames: ['ljoint1', 'rjoint1'],
        currentQ: [0, 0],
        ignoredJointNames: DEFAULT_HUMANOID_IGNORED_JOINTS
    });

    assert.equal(result.updated, true);
    assert.deepEqual(result.nextQ, [0.5, -0.2]);
    assert.equal(result.updatedCount, 2);
});

test('mapHumanoidJointPositions skips invalid numeric values', () => {
    const result = mapHumanoidJointPositions({
        messageNames: ['ljoint1', 'rjoint1'],
        positions: ['1.2', Number.NaN],
        jointNames: ['ljoint1', 'rjoint1'],
        currentQ: [0, 0]
    });

    assert.equal(result.updated, true);
    assert.deepEqual(result.nextQ, [1.2, 0]);
    assert.equal(result.updatedCount, 1);
});

test('mapHumanoidJointPositions returns unchanged state when no joint matches', () => {
    const result = mapHumanoidJointPositions({
        messageNames: ['left_wheel_joint', 'right_wheel_joint'],
        positions: [1, 2],
        jointNames: ['ljoint1', 'rjoint1'],
        currentQ: [0.1, -0.1],
        ignoredJointNames: DEFAULT_HUMANOID_IGNORED_JOINTS
    });

    assert.equal(result.updated, false);
    assert.deepEqual(result.nextQ, [0.1, -0.1]);
    assert.equal(result.updatedCount, 0);
});
