import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('info panel removes legacy tabs and keeps only pose content', () => {
    const indexHtml = readFileSync(resolve(ROOT, 'index.html'), 'utf8');
    assert.doesNotMatch(indexHtml, /info-tabs/);
    assert.doesNotMatch(indexHtml, /data-tab="pose"/);
    assert.doesNotMatch(indexHtml, /data-tab="jacobian"/);
    assert.doesNotMatch(indexHtml, /data-tab="dq"/);
    assert.doesNotMatch(indexHtml, /id="jacobian-tab"/);
    assert.doesNotMatch(indexHtml, /id="dq-tab"/);
    assert.doesNotMatch(indexHtml, /双臂机器人位姿/);
    assert.doesNotMatch(indexHtml, /实部:/);
    assert.doesNotMatch(indexHtml, /对偶:/);

    const eventHandlersJs = readFileSync(resolve(ROOT, 'src/ui/event-handlers.js'), 'utf8');
    assert.doesNotMatch(eventHandlersJs, /function switchTab/);
    assert.doesNotMatch(eventHandlersJs, /callbacks\.switchTab/);
    assert.doesNotMatch(eventHandlersJs, /querySelectorAll\('\.tab-btn'\)/);

    const stateJs = readFileSync(resolve(ROOT, 'src/core/state.js'), 'utf8');
    assert.doesNotMatch(stateJs, /updateDQDisplay/);
    assert.doesNotMatch(stateJs, /updateJacobianDisplay/);
    assert.doesNotMatch(stateJs, /switchTab/);
});
