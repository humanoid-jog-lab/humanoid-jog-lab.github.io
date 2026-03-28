import test from 'node:test';
import assert from 'node:assert/strict';

import {
    resolveHumanoidJogVFIDefaultSecondaryMetric,
    shouldProtectCommandRowsForHumanoidVFISecondary
} from './humanoid-jog-vfi-secondary-metric.js';

test('resolveHumanoidJogVFIDefaultSecondaryMetric disables secondary manipulability optimization for absolute-target new qpJ', () => {
    assert.equal(resolveHumanoidJogVFIDefaultSecondaryMetric('absolute'), null);
});

test('resolveHumanoidJogVFIDefaultSecondaryMetric disables secondary manipulability optimization for other new qpJ modes', () => {
    assert.equal(resolveHumanoidJogVFIDefaultSecondaryMetric('relative'), null);
    assert.equal(resolveHumanoidJogVFIDefaultSecondaryMetric('distance'), null);
});

test('shouldProtectCommandRowsForHumanoidVFISecondary disables command-row projection for absolute-target new qpJ', () => {
    assert.equal(shouldProtectCommandRowsForHumanoidVFISecondary('absolute'), false);
});

test('shouldProtectCommandRowsForHumanoidVFISecondary keeps command-row projection for non-absolute new qpJ modes', () => {
    assert.equal(shouldProtectCommandRowsForHumanoidVFISecondary('relative'), true);
    assert.equal(shouldProtectCommandRowsForHumanoidVFISecondary('distance'), true);
});
