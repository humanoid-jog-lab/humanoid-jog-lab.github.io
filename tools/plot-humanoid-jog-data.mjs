#!/usr/bin/env node

import { mkdir, readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';

import { buildHumanoidJogOfflinePlotSvg } from '../src/core/humanoid-jog-offline-plot.js';

function usage() {
    return `Usage:
  node tools/plot-humanoid-jog-data.mjs --input <samples.json> [options]

Options:
  --input <path>             Offline sampler JSON file.
  --out <path>               SVG output path. Default: input path with .svg extension.
  --title <text>             Plot title.
  --joint-indices <csv>      Joint indices to plot, e.g. 0,5,12.
  --width <n>                SVG width. Default: 1100.
  --height <n>               SVG height. Default: 720.
  --help                     Show this message.
`;
}

function parseArgs(argv) {
    const args = {};
    for (let index = 0; index < argv.length; index += 1) {
        const raw = argv[index];
        if (!raw.startsWith('--')) continue;
        const equalIndex = raw.indexOf('=');
        const key = raw.slice(2, equalIndex >= 0 ? equalIndex : undefined);
        const inlineValue = equalIndex >= 0 ? raw.slice(equalIndex + 1) : null;
        if (key === 'help') {
            args.help = true;
            continue;
        }
        if (inlineValue != null) {
            args[key] = inlineValue;
        } else if (index + 1 < argv.length && !argv[index + 1].startsWith('--')) {
            index += 1;
            args[key] = argv[index];
        } else {
            args[key] = 'true';
        }
    }
    return args;
}

function parseNumber(value, fallback) {
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric : fallback;
}

function parseJointIndices(value) {
    if (!value) return [0, 5, 12];
    return String(value)
        .split(',')
        .map((part) => Number(part.trim()))
        .filter((value) => Number.isInteger(value) && value >= 0);
}

async function writeTextFile(filePath, content) {
    await mkdir(path.dirname(filePath), { recursive: true });
    await writeFile(filePath, content, 'utf8');
}

const args = parseArgs(process.argv.slice(2));
if (args.help) {
    process.stdout.write(usage());
    process.exit(0);
}

if (!args.input) {
    console.error('Missing required --input <samples.json>');
    process.stdout.write(usage());
    process.exit(1);
}

const inputPath = path.resolve(args.input);
const outputPath = path.resolve(args.out || inputPath.replace(/\.json$/i, '.svg'));
const result = JSON.parse(await readFile(inputPath, 'utf8'));
const svg = buildHumanoidJogOfflinePlotSvg(result, {
    title: args.title,
    jointIndices: parseJointIndices(args['joint-indices']),
    width: parseNumber(args.width, 1100),
    height: parseNumber(args.height, 720)
});

await writeTextFile(outputPath, svg);
console.log(`svg: ${outputPath}`);
