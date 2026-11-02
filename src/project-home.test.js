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

test('project home links to and embeds the full YouTube project video', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');

    assert.match(html, /<a href="https:\/\/youtu\.be\/FtjjyPNJ74Y"[^>]*>Video<\/a>/);
    assert.match(html, /<section class="section youtube-section" id="project-video">/);
    assert.match(html, /src="https:\/\/www\.youtube\.com\/embed\/FtjjyPNJ74Y\?si=MJQLKsYJaQsFRuh9"/);
    assert.match(html, /title="YouTube video player"/);
    assert.match(html, /allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"/);
    assert.match(html, /referrerpolicy="strict-origin-when-cross-origin"/);
    assert.match(html, /\sallowfullscreen/);
});

test('project home renders at the compounded 81 percent visual scale', () => {
    const css = readFileSync(resolve(ROOT, 'src/project-home.css'), 'utf8');

    assert.match(css, /--page-visual-scale:\s*0\.81;/);
    assert.match(css, /body\s*\{[\s\S]*zoom:\s*var\(--page-visual-scale\);/);
});
