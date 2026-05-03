#!/usr/bin/env node

import { mkdir, readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import {
    runHumanoidJogOfflineSampler,
    serializeHumanoidJogOfflineSamplesToCsv
} from '../src/core/humanoid-jog-offline-sampler.js';

const DEFAULT_Q = [
    -0.20794, 0.90626, -0.69682, 0,
    -0.611, 0, 0.00586, -0.94966, -0.00966, 0.00069, -0.01189,
    0.611, 0, -0.00586, 0.94966, 0.00966, -0.00069, 0.01189,
    0, 0
];

function usage() {
    return `Usage:
  node tools/run-humanoid-jog-offline-sampler.mjs [options]

Options:
  --humanoid '[...]'              Initial 20-element humanoid joint vector.
  --humanoid-file <path>          File containing JSON array or humanoid=[...].
  --mode <name>                   Control mode label. Default: absolute-target.
  --solver <name>                 Solver label/config. Default: mj_restore_tr_safe_wvfi.
  --axis <x|y|z>                  Command axis. Default: x.
  --direction <positive|negative> Command direction. Default: positive.
  --command-index <n>             Override active joint/variable index for the command.
  --steps <n>                     Number of offline samples. Default: 50.
  --speed-percent <n>             Command speed percentage. Default: 50.
  --base-step <n>                 Full-speed command increment. Default: 0.02.
  --include-waist <0|1>           Include waist/body joints. Default: 1.
  --param key=value               Override sampler parameter. Repeatable.
  --out <path>                    JSON output path. Default: artifacts/humanoid-jog-offline/*.json.
  --csv <path>                    CSV output path. Default: alongside JSON.
  --no-csv                        Only write JSON.
  --help                          Show this message.
`;
}

function parsePrimitive(value) {
    if (value === 'true') return true;
    if (value === 'false') return false;
    if (value === 'null') return null;
    const numeric = Number(value);
    return Number.isFinite(numeric) && value.trim() !== '' ? numeric : value;
}

function parseArgs(argv) {
    const args = { params: {} };
    for (let index = 0; index < argv.length; index += 1) {
        const raw = argv[index];
        if (!raw.startsWith('--')) continue;
        const equalIndex = raw.indexOf('=');
        const key = raw.slice(2, equalIndex >= 0 ? equalIndex : undefined);
        const inlineValue = equalIndex >= 0 ? raw.slice(equalIndex + 1) : null;
        const readValue = () => {
            if (inlineValue != null) return inlineValue;
            if (index + 1 >= argv.length || argv[index + 1].startsWith('--')) return 'true';
            index += 1;
            return argv[index];
        };

        if (key === 'help') {
            args.help = true;
        } else if (key === 'param') {
            const value = readValue();
            const splitIndex = value.indexOf('=');
            if (splitIndex > 0) {
                args.params[value.slice(0, splitIndex)] = parsePrimitive(value.slice(splitIndex + 1));
            }
        } else if (key === 'no-csv') {
            args.noCsv = true;
        } else {
            args[key] = readValue();
        }
    }
    return args;
}

function parseBool(value, fallback = false) {
    if (value == null) return fallback;
    const normalized = String(value).trim().toLowerCase();
    if (['1', 'true', 'yes', 'on'].includes(normalized)) return true;
    if (['0', 'false', 'no', 'off'].includes(normalized)) return false;
    return fallback;
}

function parseNumber(value, fallback) {
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric : fallback;
}

function parseHumanoidText(text) {
    const trimmed = String(text || '').trim();
    const bracketMatch = trimmed.match(/\[[\s\S]*\]/);
    const candidate = bracketMatch ? bracketMatch[0] : trimmed;
    const parsed = JSON.parse(candidate);
    if (!Array.isArray(parsed)) {
        throw new Error('humanoid input must be a JSON array');
    }
    return parsed.map((value) => Number(value));
}

async function resolveHumanoid(args) {
    if (args.humanoid) return parseHumanoidText(args.humanoid);
    if (args['humanoid-file']) {
        const text = await readFile(path.resolve(args['humanoid-file']), 'utf8');
        return parseHumanoidText(text);
    }
    return DEFAULT_Q.slice();
}

function timestampForFile(date = new Date()) {
    return date.toISOString().replace(/[:.]/g, '-');
}

function defaultOutputPath({ mode, solverMode, axis, direction, steps, speedPercent }) {
    const safe = (value) => String(value || '')
        .replace(/[^a-zA-Z0-9_.+-]+/g, '-')
        .replace(/^-+|-+$/g, '');
    const filename = [
        safe(mode),
        safe(solverMode),
        `${safe(axis)}${String(direction).startsWith('neg') ? '-' : '+'}`,
        `${steps}steps`,
        `${speedPercent}pct`,
        timestampForFile()
    ].filter(Boolean).join('_');
    return path.join('artifacts', 'humanoid-jog-offline', `${filename}.json`);
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

const mode = args.mode || 'absolute-target';
const solverMode = args.solver || args['solver-mode'] || 'mj_restore_tr_safe_wvfi';
const axis = args.axis || 'x';
const direction = args.direction || 'positive';
const steps = Math.max(1, Math.floor(parseNumber(args.steps, 50)));
const speedPercent = parseNumber(args['speed-percent'], 50);
const includeWaist = parseBool(args['include-waist'], true);
const initialQ = await resolveHumanoid(args);
const commandIndex = parseNumber(args['command-index'], Number.NaN);
const outputPath = path.resolve(args.out || defaultOutputPath({
    mode,
    solverMode,
    axis,
    direction,
    steps,
    speedPercent
}));
const csvPath = args.noCsv
    ? null
    : path.resolve(args.csv || outputPath.replace(/\.json$/i, '.csv'));

const result = runHumanoidJogOfflineSampler({
    initialQ,
    mode,
    solverMode,
    axis,
    direction,
    steps,
    speedPercent,
    baseStep: parseNumber(args['base-step'], 0.02),
    includeWaist,
    ...(Number.isInteger(commandIndex) ? { commandIndex } : {}),
    ...args.params
});

await writeTextFile(outputPath, `${JSON.stringify(result, null, 2)}\n`);
if (csvPath) {
    await writeTextFile(csvPath, serializeHumanoidJogOfflineSamplesToCsv(result));
}

const scriptName = path.basename(fileURLToPath(import.meta.url));
const okCount = result.summary?.okCount ?? 0;
const sampleCount = result.summary?.sampleCount ?? result.samples.length;
const meanSolveMs = result.summary?.meanSolveMs ?? 0;

console.log(`${scriptName}: wrote ${sampleCount} offline samples (${okCount} ok)`);
console.log(`json: ${outputPath}`);
if (csvPath) console.log(`csv:  ${csvPath}`);
console.log(`mean_solve_ms: ${meanSolveMs.toFixed(3)}  total_ms: ${(result.summary?.totalMs || 0).toFixed(3)}`);
