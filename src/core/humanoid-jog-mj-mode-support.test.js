import test from 'node:test';
import assert from 'node:assert/strict';

import { supportsHumanoidJogMJRestoreMode } from './humanoid-jog-mj-mode-support.js';

test('supportsHumanoidJogMJRestoreMode supports all unified mj_restore humanoid mode tags', () => {
    assert.equal(supportsHumanoidJogMJRestoreMode('absolute-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('relative-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('distance-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('robot1-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('robot2-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('robot1-waist-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('robot2-waist-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('waist-body-link-xz-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('dual-independent-surface-slide-target'), true);
    assert.equal(supportsHumanoidJogMJRestoreMode('cooperative-ruled-surface-polishing-target'), true);
});
