import assert from 'node:assert/strict';
import { execFile } from 'node:child_process';
import { mkdtemp, readFile } from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { promisify } from 'node:util';
import { test } from 'node:test';

const execFileAsync = promisify(execFile);
const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');

test('offline CLI scripts do not depend on browser automation APIs', async () => {
    const samplerSource = await readFile(path.join(ROOT, 'tools/run-humanoid-jog-offline-sampler.mjs'), 'utf8');
    const plotSource = await readFile(path.join(ROOT, 'tools/plot-humanoid-jog-data.mjs'), 'utf8');
    const combined = `${samplerSource}\n${plotSource}`;
    assert.doesNotMatch(combined, /playwright|chromium|page\.evaluate|document\.|window\./i);
});

test('offline sampler CLI writes JSON/CSV and plot CLI renders SVG from that file', async () => {
    const tempDir = await mkdtemp(path.join(os.tmpdir(), 'humanoid-offline-'));
    const jsonPath = path.join(tempDir, 'samples.json');
    const csvPath = path.join(tempDir, 'samples.csv');
    const svgPath = path.join(tempDir, 'samples.svg');

    await execFileAsync(process.execPath, [
        path.join(ROOT, 'tools/run-humanoid-jog-offline-sampler.mjs'),
        '--steps', '2',
        '--speed-percent', '20',
        '--out', jsonPath,
        '--csv', csvPath
    ], { cwd: ROOT });

    const result = JSON.parse(await readFile(jsonPath, 'utf8'));
    const csv = await readFile(csvPath, 'utf8');
    assert.equal(result.meta.source, 'offline-core');
    assert.equal(result.meta.usesBrowser, false);
    assert.equal(result.samples.length, 2);
    assert.match(csv, /^index,status,reason,err_cmd/m);

    await execFileAsync(process.execPath, [
        path.join(ROOT, 'tools/plot-humanoid-jog-data.mjs'),
        '--input', jsonPath,
        '--out', svgPath,
        '--title', 'CLI offline plot'
    ], { cwd: ROOT });

    const svg = await readFile(svgPath, 'utf8');
    assert.match(svg, /^<svg/);
    assert.match(svg, /CLI offline plot/);
});
