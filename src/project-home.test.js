import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '..');

test('project home hardware videos use deterministic autoplay presentation without native controls', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');
    const videoTags = [...html.matchAll(/<video[\s\S]*?>/g)].map((match) => match[0]);

    assert.equal(videoTags.length, 3);
    for (const tag of videoTags) {
        assert.match(tag, /class="project-video"/);
        assert.match(tag, /autoplay/);
        assert.match(tag, /muted/);
        assert.match(tag, /playsinline/);
        assert.match(tag, /data-smooth-loop/);
        assert.doesNotMatch(tag, /\scontrols(?:\s|>|=)/);
    }
});
