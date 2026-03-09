import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildRobotArmServoMessageData,
    buildRobotServoMessageData
} from './servo-message-payloads.js';

test('buildRobotArmServoMessageData uses humanoidQ for humanoid mode arms', () => {
    const humanoidQ = [
        -0.3,
        0.9,
        -0.6,
        0.0,
        -0.36757,
        0.2782,
        0.50861,
        -1.39601,
        -0.38305,
        0.43154,
        0.03948,
        0.36758,
        -0.27819,
        -0.50863,
        1.39601,
        0.38303,
        -0.43156,
        -0.03949,
        0.0,
        0.0
    ];

    const result = buildRobotArmServoMessageData({
        humanoidMode: true,
        humanoidQ
    });

    assert.equal(result.source, 'humanoid');
    assert.equal(result.messageData.run_mode, 1);
    assert.equal(result.messageData.robot_power, true);
    assert.equal(result.messageData.ljoint1_position, -0.36757);
    assert.equal(result.messageData.ljoint7_position, 0.03948);
    assert.equal(result.messageData.rjoint1_position, 0.36758);
    assert.equal(result.messageData.rjoint7_position, -0.03949);
});

test('buildRobotArmServoMessageData keeps existing dual-arm mapping', () => {
    const result = buildRobotArmServoMessageData({
        dualArmMode: true,
        jointAngles1: [1, 2, 3, 4, 5, 6, 7],
        jointAngles2: [-1, -2, -3, -4, -5, -6, -7],
        runMode: 2,
        robotPower: false,
        runTime: 3.5
    });

    assert.equal(result.source, 'dual-arm');
    assert.equal(result.messageData.run_mode, 2);
    assert.equal(result.messageData.robot_power, false);
    assert.equal(result.messageData.run_time, 3.5);
    assert.equal(result.messageData.ljoint1_position, -1);
    assert.equal(result.messageData.ljoint7_position, -7);
    assert.equal(result.messageData.rjoint1_position, 1);
    assert.equal(result.messageData.rjoint7_position, 7);
});

test('buildRobotServoMessageData uses humanoid waist joints and defaults run_mode to 0 and vel to 0.1', () => {
    const result = buildRobotServoMessageData({
        humanoidMode: true,
        humanoidQ: [-0.3, 0.9, -0.6, 0.2, 10, 11]
    });

    assert.equal(result.valid, true);
    assert.deepEqual(result.messageData, {
        run_mode: 0,
        ankle_position: -0.3,
        knee_position: 0.9,
        hip_pitch_position: -0.6,
        hip_yaw_position: 0.2,
        x: 0,
        y: 0,
        phi: 0,
        mode_leg_select: 0,
        vel: 0.1
    });
});

test('buildRobotServoMessageData marks non-humanoid state invalid', () => {
    const result = buildRobotServoMessageData({
        humanoidMode: false,
        humanoidQ: [1, 2, 3, 4]
    });

    assert.equal(result.valid, false);
    assert.equal(result.messageData.ankle_position, 1);
    assert.equal(result.messageData.hip_yaw_position, 4);
});
