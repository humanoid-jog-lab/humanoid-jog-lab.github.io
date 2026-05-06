import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '..');

test('interactive app uses compounded 81 percent scale only when opened as the top-level page', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');
    const css = readFileSync(resolve(ROOT, 'src/styles.css'), 'utf8');

    assert.match(html, /standalone-app/);
    assert.match(html, /embedded-app/);
    assert.match(css, /--app-visual-scale:\s*0\.81;/);
    assert.match(css, /html\.standalone-app\s+body\s*\{[\s\S]*zoom:\s*var\(--app-visual-scale\);/);
    assert.match(css, /width:\s*calc\(100vw \/ var\(--app-visual-scale\)\);/);
    assert.match(css, /height:\s*calc\(100vh \/ var\(--app-visual-scale\)\);/);
});
