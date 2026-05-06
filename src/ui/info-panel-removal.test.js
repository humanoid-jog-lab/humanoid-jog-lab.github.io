import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('info panel removes legacy tabs and keeps only pose content', () => {
    const appHtml = readFileSync(resolve(ROOT, 'app.html'), 'utf8');
    assert.doesNotMatch(appHtml, /info-tabs/);
    assert.doesNotMatch(appHtml, /data-tab="pose"/);
    assert.doesNotMatch(appHtml, /data-tab="jacobian"/);
    assert.doesNotMatch(appHtml, /data-tab="dq"/);
    assert.doesNotMatch(appHtml, /id="jacobian-tab"/);
    assert.doesNotMatch(appHtml, /id="dq-tab"/);
    assert.doesNotMatch(appHtml, /双臂机器人位姿/);
    assert.doesNotMatch(appHtml, /实部:/);
    assert.doesNotMatch(appHtml, /对偶:/);

    const eventHandlersJs = readFileSync(resolve(ROOT, 'src/ui/event-handlers.js'), 'utf8');
    assert.doesNotMatch(eventHandlersJs, /function switchTab/);
    assert.doesNotMatch(eventHandlersJs, /callbacks\.switchTab/);
    assert.doesNotMatch(eventHandlersJs, /querySelectorAll\('\.tab-btn'\)/);

    const stateJs = readFileSync(resolve(ROOT, 'src/core/state.js'), 'utf8');
    assert.doesNotMatch(stateJs, /updateDQDisplay/);
    assert.doesNotMatch(stateJs, /updateJacobianDisplay/);
    assert.doesNotMatch(stateJs, /switchTab/);
});
