#!/usr/bin/env node

import fs from 'node:fs/promises';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';

import { chromium } from 'playwright';

import {
    formatBenchmarkCsvTable,
    formatBenchmarkLatexTable,
    formatBenchmarkMarkdownTable
} from '../src/core/humanoid-jog-benchmark-metrics.js';
import {
    buildHumanoidControllerComparisonTasks,
    getHumanoidControllerComparisonColumns,
    resolveHumanoidControllerComparisonControllerOptions,
    resolveHumanoidControllerComparisonDefaultMaxTicks,
    resolveHumanoidControllerComparisonSpeedPercent,
    resolveHumanoidControllerComparisonTaskValidTolerance,
    summarizeHumanoidControllerComparisonRows
} from '../src/core/humanoid-jog-controller-comparison.js';

const REPO_ROOT = process.env.COMPARISON_REPO_ROOT || process.cwd();
const OUTPUT_DIR = process.env.COMPARISON_OUTPUT_DIR
    || path.join(REPO_ROOT, 'artifacts', 'humanoid-controller-comparison');
const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173';
const OUTPUT_JSON = process.env.COMPARISON_JSON
    || path.join(OUTPUT_DIR, 'controller_comparison_x30cm.json');
const OUTPUT_MD = process.env.COMPARISON_MD
    || path.join(OUTPUT_DIR, 'summary.md');
const OUTPUT_CSV = process.env.COMPARISON_CSV
    || path.join(OUTPUT_DIR, 'summary.csv');
const OUTPUT_LATEX = process.env.COMPARISON_LATEX
    || path.join(OUTPUT_DIR, 'summary_latex.tex');
const TARGET_X_METERS = Number.isFinite(Number(process.env.COMPARISON_TARGET_X_METERS))
    ? Math.max(0.01, Number(process.env.COMPARISON_TARGET_X_METERS))
    : 0.30;
const TEACH_SPEED_PERCENT = resolveHumanoidControllerComparisonSpeedPercent(
    process.env.COMPARISON_SPEED_PERCENT
);
const MAX_TICKS = Number.isFinite(Number(process.env.COMPARISON_MAX_TICKS))
    ? Math.max(1, Math.floor(Number(process.env.COMPARISON_MAX_TICKS)))
    : resolveHumanoidControllerComparisonDefaultMaxTicks(TEACH_SPEED_PERCENT);
const MIN_EFFECTIVE_PROGRESS = Number.isFinite(Number(process.env.COMPARISON_MIN_PROGRESS))
    ? Math.max(1e-8, Number(process.env.COMPARISON_MIN_PROGRESS))
    : 1e-5;
const MAX_STALL_TICKS = Number.isFinite(Number(process.env.COMPARISON_MAX_STALL_TICKS))
    ? Math.max(1, Math.floor(Number(process.env.COMPARISON_MAX_STALL_TICKS)))
    : 3;
const FIXED_TICKS = String(process.env.COMPARISON_FIXED_TICKS || '').trim() === '1'
    || String(process.env.COMPARISON_FIXED_TICKS || '').trim().toLowerCase() === 'true';
const scenarioTickOverride = (name) => {
    const value = Number(process.env[name]);
    return Number.isFinite(value) ? Math.max(1, Math.floor(value)) : Number.NaN;
};
const ARM_ONLY_ABSOLUTE_TICKS = scenarioTickOverride('COMPARISON_ARM_ONLY_ABSOLUTE_TICKS');
const WAIST_UPRIGHT_ABSOLUTE_TICKS = scenarioTickOverride('COMPARISON_WAIST_UPRIGHT_ABSOLUTE_TICKS');
const WAIST_DISTANCE_TICKS = scenarioTickOverride('COMPARISON_WAIST_DISTANCE_TICKS');
const COMMAND_TOLERANCE = Number.isFinite(Number(process.env.COMPARISON_COMMAND_TOLERANCE))
    ? Math.max(0, Number(process.env.COMPARISON_COMMAND_TOLERANCE))
    : 5e-4;
const BODY_TOLERANCE = Number.isFinite(Number(process.env.COMPARISON_BODY_TOLERANCE))
    ? Math.max(0, Number(process.env.COMPARISON_BODY_TOLERANCE))
    : 5e-5;
const TASK_VALID_TOLERANCE = resolveHumanoidControllerComparisonTaskValidTolerance(
    process.env.COMPARISON_TASK_VALID_TOLERANCE
);
const WAIST_CORRIDOR_BACKWARD_MARGIN = Number.isFinite(Number(process.env.COMPARISON_WAIST_CORRIDOR_BACKWARD_MARGIN))
    ? Math.max(0.01, Number(process.env.COMPARISON_WAIST_CORRIDOR_BACKWARD_MARGIN))
    : 0.40;
const WAIST_CORRIDOR_FORWARD_MARGIN = Number.isFinite(Number(process.env.COMPARISON_WAIST_CORRIDOR_FORWARD_MARGIN))
    ? Math.max(0.01, Number(process.env.COMPARISON_WAIST_CORRIDOR_FORWARD_MARGIN))
    : 0.20;
const NEAR_LIMIT_SEED = Number.isFinite(Number(process.env.COMPARISON_NEAR_LIMIT_SEED))
    ? Math.floor(Number(process.env.COMPARISON_NEAR_LIMIT_SEED))
    : 7;
const MODE_FILTER = new Set(
    String(process.env.COMPARISON_MODE_FILTER || '')
        .split(',')
        .map((value) => value.trim())
        .filter(Boolean)
);
const PROFILE_FILTER = new Set(
    String(process.env.COMPARISON_PROFILE_FILTER || '')
        .split(',')
        .map((value) => value.trim())
        .filter(Boolean)
);
const SOLVER_FILTER = new Set(
    String(process.env.COMPARISON_SOLVER_FILTER || '')
        .split(',')
        .map((value) => value.trim())
        .filter(Boolean)
);
const FORCE_JOINT_LIMIT_AVOIDANCE = String(process.env.COMPARISON_FORCE_JOINT_LIMIT_AVOIDANCE || '').trim() === '1'
    || String(process.env.COMPARISON_FORCE_JOINT_LIMIT_AVOIDANCE || '').trim().toLowerCase() === 'true';

function parseNumberArrayEnv(valueRaw = '') {
    const text = String(valueRaw || '').trim();
    if (!text) return null;
    try {
        const parsed = JSON.parse(text);
        if (Array.isArray(parsed) && parsed.every((value) => Number.isFinite(Number(value)))) {
            return parsed.map(Number);
        }
    } catch (_) {}
    const values = text
        .replace(/^\[/, '')
        .replace(/\]$/, '')
        .split(',')
        .map((value) => Number(value.trim()));
    return values.length > 0 && values.every(Number.isFinite) ? values : null;
}

const WAIST_DISTANCE_INITIAL_Q = parseNumberArrayEnv(process.env.COMPARISON_WAIST_DISTANCE_INITIAL_Q);

async function waitForHttp(url, timeoutMs = 120000) {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
        try {
            const response = await fetch(url);
            if (response.ok) return;
        } catch (_) {}
        await delay(500);
    }
    throw new Error(`Timed out waiting for ${url}`);
}

async function startServerIfNeeded() {
    if (process.env.BASE_URL) {
        await waitForHttp(BASE_URL);
        return null;
    }

    const child = spawn(
        'npm',
        ['run', 'dev', '--', '--host', '127.0.0.1', '--port', '4173', '--strictPort'],
        {
            cwd: REPO_ROOT,
            stdio: ['ignore', 'pipe', 'pipe'],
            detached: true
        }
    );

    child.stdout.on('data', (chunk) => process.stdout.write(chunk));
    child.stderr.on('data', (chunk) => process.stderr.write(chunk));

    await waitForHttp(BASE_URL);
    return child;
}

async function stopServer(child) {
    if (!child) return;
    const signalGroup = (signal) => {
        try {
            process.kill(-child.pid, signal);
            return true;
        } catch (_) {
            return false;
        }
    };
    const exitPromise = new Promise((resolve) => {
        child.once('exit', resolve);
        child.once('close', resolve);
    });
    if (!signalGroup('SIGTERM')) child.kill('SIGTERM');
    const exited = await Promise.race([exitPromise.then(() => true), delay(1500).then(() => false)]);
    if (!exited) {
        if (!signalGroup('SIGKILL')) child.kill('SIGKILL');
        await Promise.race([exitPromise, delay(1500)]);
    }
}

async function ensureHumanoidPageReady(page) {
    await page.goto(BASE_URL, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(
        () => Boolean(
            window.visualizer &&
            window.humanoidMode === true &&
            Array.isArray(window.humanoidQ) &&
            window.humanoidQ.length > 0 &&
            window.DQModule?.DQWrapper
        ),
        { timeout: 180000 }
    );
}

function buildMarkdownRows(summaryRows = []) {
    return formatBenchmarkMarkdownTable(summaryRows, {
        columns: getHumanoidControllerComparisonColumns()
    });
}

function buildCsvRows(summaryRows = []) {
    return formatBenchmarkCsvTable(summaryRows, {
        columns: getHumanoidControllerComparisonColumns()
    });
}

function buildLatexRows(summaryRows = []) {
    return formatBenchmarkLatexTable(summaryRows, {
        columns: getHumanoidControllerComparisonColumns()
    });
}

async function writeBenchmarkOutputs(resultRuns) {
    const summaryRows = summarizeHumanoidControllerComparisonRows(resultRuns, {
        taskValidTolerance: TASK_VALID_TOLERANCE
    });
    const markdown = buildMarkdownRows(summaryRows);
    const csv = buildCsvRows(summaryRows);
    const latex = buildLatexRows(summaryRows);
    await fs.mkdir(path.dirname(OUTPUT_JSON), { recursive: true });
    await fs.writeFile(OUTPUT_JSON, JSON.stringify({
        meta: {
            targetXMeters: TARGET_X_METERS,
            maxTicks: MAX_TICKS,
            fixedTicks: FIXED_TICKS,
            teachSpeedPercent: TEACH_SPEED_PERCENT,
            minEffectiveProgress: MIN_EFFECTIVE_PROGRESS,
            maxStallTicks: MAX_STALL_TICKS,
            commandTolerance: COMMAND_TOLERANCE,
            bodyTolerance: BODY_TOLERANCE,
            taskValidTolerance: TASK_VALID_TOLERANCE,
            waistCorridorBackwardMargin: WAIST_CORRIDOR_BACKWARD_MARGIN,
            waistCorridorForwardMargin: WAIST_CORRIDOR_FORWARD_MARGIN,
            nearLimitSeed: NEAR_LIMIT_SEED,
            forceJointLimitAvoidance: FORCE_JOINT_LIMIT_AVOIDANCE,
            waistDistanceInitialQ: WAIST_DISTANCE_INITIAL_Q,
            scenarioTargets: {
                absolute: 0.075,
                'decoupled-upright': TARGET_X_METERS,
                'waist-distance': 0.075
            },
            scenarioTicks: {
                absolute: Number.isFinite(ARM_ONLY_ABSOLUTE_TICKS) ? ARM_ONLY_ABSOLUTE_TICKS : null,
                'decoupled-upright': Number.isFinite(WAIST_UPRIGHT_ABSOLUTE_TICKS) ? WAIST_UPRIGHT_ABSOLUTE_TICKS : null,
                'waist-distance': Number.isFinite(WAIST_DISTANCE_TICKS) ? WAIST_DISTANCE_TICKS : null
            }
        },
        rows: summaryRows,
        runs: resultRuns
    }, null, 2));
    await fs.mkdir(path.dirname(OUTPUT_MD), { recursive: true });
    await fs.writeFile(OUTPUT_MD, `${markdown}\n`);
    await fs.mkdir(path.dirname(OUTPUT_CSV), { recursive: true });
    await fs.writeFile(OUTPUT_CSV, csv);
    await fs.mkdir(path.dirname(OUTPUT_LATEX), { recursive: true });
    await fs.writeFile(OUTPUT_LATEX, `${latex}\n`);
    return { summaryRows, markdown, csv, latex };
}

const browser = await chromium.launch({
    headless: process.env.PLAYWRIGHT_HEADLESS !== '0',
    args: ['--use-angle=swiftshader', '--ignore-gpu-blocklist']
});

let server = null;
try {
    server = await startServerIfNeeded();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    page.on('console', (message) => {
        if (message.type() === 'log' && message.text().startsWith('[scenario]')) {
            process.stdout.write(`[page] ${message.text()}\n`);
        }
    });
    await ensureHumanoidPageReady(page);

    const tasks = buildHumanoidControllerComparisonTasks().filter((task) => {
        if (MODE_FILTER.size > 0 && !MODE_FILTER.has(task.mode)) return false;
        if (PROFILE_FILTER.size > 0 && !PROFILE_FILTER.has(task.profile)) return false;
        if (SOLVER_FILTER.size > 0 && !SOLVER_FILTER.has(task.solverMode)) return false;
        return true;
    }).map((task) => {
        const comparisonOptions = resolveHumanoidControllerComparisonControllerOptions({
            controllerKey: task.key,
            mode: task.mode,
            includeWaist: task.includeWaist,
            maxTicks: (() => {
                if (task.mode === 'absolute' && task.includeWaist !== true && Number.isFinite(ARM_ONLY_ABSOLUTE_TICKS)) {
                    return ARM_ONLY_ABSOLUTE_TICKS;
                }
                if (task.mode === 'decoupled-upright' && Number.isFinite(WAIST_UPRIGHT_ABSOLUTE_TICKS)) {
                    return WAIST_UPRIGHT_ABSOLUTE_TICKS;
                }
                if (task.mode === 'waist-distance' && Number.isFinite(WAIST_DISTANCE_TICKS)) {
                    return WAIST_DISTANCE_TICKS;
                }
                return MAX_TICKS;
            })(),
            teachSpeedPercent: TEACH_SPEED_PERCENT,
            targetXMeters: TARGET_X_METERS,
            commandTolerance: COMMAND_TOLERANCE,
            maxStallTicks: MAX_STALL_TICKS,
            fixedTicks: FIXED_TICKS
        });
        if (FORCE_JOINT_LIMIT_AVOIDANCE) {
            comparisonOptions.jointLimitAvoidanceEnabled = true;
        }
        return {
            ...task,
            initialQOverride: task.mode === 'waist-distance' && Array.isArray(WAIST_DISTANCE_INITIAL_Q)
                ? WAIST_DISTANCE_INITIAL_Q
                : null,
            comparisonOptions
        };
    });
    if (tasks.length === 0) {
        throw new Error('No controller-comparison tasks matched the provided filters.');
    }
    const result = await page.evaluate(
        async ({
            tasks: tasksInput,
            targetXMeters,
            maxTicks,
            teachSpeedPercent,
            minEffectiveProgress,
            maxStallTicks,
            commandTolerance,
            bodyTolerance,
            fixedTicks,
            waistCorridorBackwardMargin,
            waistCorridorForwardMargin,
            nearLimitSeed
        }) => {
            const viz = window.visualizer;
            const DQ = window.DQModule?.DQWrapper;
            const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
            const originalConsoleLog = console.log.bind(console);
            console.log = (...args) => {
                const text = args.map((item) => String(item)).join(' ');
                if (text.startsWith('[scenario]')) {
                    originalConsoleLog(...args);
                }
            };

            const dispatchSelectValue = (id, value) => {
                const element = document.getElementById(id);
                if (!element) return;
                element.value = value;
                element.dispatchEvent(new Event('change', { bubbles: true }));
            };

            const dispatchInputValue = (id, value) => {
                const element = document.getElementById(id);
                if (!element) return;
                element.value = String(value);
                element.dispatchEvent(new Event('input', { bubbles: true }));
                element.dispatchEvent(new Event('change', { bubbles: true }));
            };

            const setCheckbox = (id, checked) => {
                const element = document.getElementById(id);
                if (!element) return;
                element.checked = checked;
                element.dispatchEvent(new Event('change', { bubbles: true }));
            };

            const applyTeachSpeedPercent = (percent) => {
                const p = Math.max(1, Math.min(100, Math.round(Number(percent) || 100)));
                const slider = document.getElementById('teach-speed-percent');
                const valueLabel = document.getElementById('teach-speed-value');
                if (slider) {
                    slider.value = String(p);
                    slider.dispatchEvent(new Event('input', { bubbles: true }));
                    slider.dispatchEvent(new Event('change', { bubbles: true }));
                }
                if (valueLabel) valueLabel.textContent = String(p);
                viz.setTeachSpeedPercent?.(p);
                if (viz) viz.teachSpeedPercent = p;
                return p;
            };

            const evaluateStep = (options = {}) => {
                const stepProgress = Number(options.stepProgress);
                const minEffectiveProgress = Number.isFinite(Number(options.minEffectiveProgress))
                    ? Math.max(1e-8, Number(options.minEffectiveProgress))
                    : 1e-5;
                const stallTicks = Number.isFinite(Number(options.stallTicks))
                    ? Math.max(0, Math.floor(Number(options.stallTicks)))
                    : 0;
                const maxStallTicks = Number.isFinite(Number(options.maxStallTicks))
                    ? Math.max(1, Math.floor(Number(options.maxStallTicks)))
                    : 3;
                const status = options.status && typeof options.status === 'object' ? options.status : {};
                const reason = typeof status.reason === 'string' && status.reason.length > 0
                    ? status.reason
                    : 'unknown';
                const progressed = Number.isFinite(stepProgress) && stepProgress >= minEffectiveProgress;
                const nextStallTicks = progressed ? 0 : stallTicks + 1;
                const explicitFailure = (
                    reason === 'fk_fail' ||
                    reason === 'jacobian_fail' ||
                    reason === 'body_task_fail' ||
                    reason === 'solver_fail' ||
                    reason === 'command_mismatch'
                );
                const ok = !explicitFailure;
                const shouldStop = explicitFailure || nextStallTicks >= maxStallTicks;
                return { reason, progressed, ok, shouldStop, nextStallTicks };
            };

            const evaluateCompletion = (options = {}) => {
                const xProgress = Number(options.xProgress);
                const target = Number.isFinite(Number(options.targetXMeters))
                    ? Math.max(1e-8, Number(options.targetXMeters))
                    : 0.30;
                const targetReachTolerance = Number.isFinite(Number(options.targetReachTolerance))
                    ? Math.max(0, Number(options.targetReachTolerance))
                    : 5e-4;
                const commandError = Number(options.commandError);
                const bodyError = Number(options.bodyError);
                const waistXViolation = Number(options.waistXViolation);
                const commandTol = Number.isFinite(Number(options.commandTolerance))
                    ? Math.max(0, Number(options.commandTolerance))
                    : 5e-4;
                const bodyTol = Number.isFinite(Number(options.bodyTolerance))
                    ? Math.max(0, Number(options.bodyTolerance))
                    : 5e-5;
                const enforceBody = options.enforceBody === true;
                const enforceWaist = options.enforceWaist === true;
                const reachedTarget = Number.isFinite(xProgress) && xProgress + targetReachTolerance >= target;
                const commandConverged = Number.isFinite(commandError) && commandError <= commandTol;
                const bodyConverged = !enforceBody || (Number.isFinite(bodyError) && bodyError <= bodyTol);
                const waistFeasible = !enforceWaist || (!Number.isFinite(waistXViolation) || waistXViolation <= 1e-12);
                return {
                    reachedTarget,
                    commandConverged,
                    bodyConverged,
                    waistFeasible,
                    completed: reachedTarget && commandConverged && bodyConverged && waistFeasible,
                    progressRatio: Number.isFinite(xProgress) ? Math.max(0, Math.min(1, xProgress / target)) : Number.NaN
                };
            };

                const meanOf = (values = []) => {
                    const finite = values.filter((value) => Number.isFinite(value));
                    if (finite.length === 0) return Number.NaN;
                    return finite.reduce((sum, value) => sum + value, 0) / finite.length;
                };

                const vectorNorm = (values = []) => {
                    if (!Array.isArray(values) || values.length === 0) return Number.NaN;
                    let sumSquares = 0;
                    let hasFiniteValue = false;
                    for (const value of values) {
                        const numericValue = Number(value);
                        if (!Number.isFinite(numericValue)) continue;
                        sumSquares += numericValue * numericValue;
                        hasFiniteValue = true;
                    }
                    return hasFiniteValue ? Math.sqrt(sumSquares) : Number.NaN;
                };

            const percentileOf = (values = [], percentile = 0.95) => {
                const finite = values.filter((value) => Number.isFinite(value)).sort((left, right) => left - right);
                if (finite.length === 0) return Number.NaN;
                const rank = Math.min(finite.length - 1, Math.max(0, Math.ceil(percentile * finite.length) - 1));
                return finite[rank];
            };

            const createRng = (seed) => {
                let state = (seed >>> 0) || 1;
                return () => {
                    state = (1664525 * state + 1013904223) >>> 0;
                    return state / 0x100000000;
                };
            };

            const getLimitBounds = (limit) => {
                if (!limit || typeof limit !== 'object') return [Number.NaN, Number.NaN];
                return [Number(limit.lower), Number(limit.upper)];
            };

            const getJointLimits = () => viz._getHumanoidJointLimits?.() || [];

            const resetHumanoidState = async (profile = 'near-limit', initialQOverride = null) => {
                const initial = Array.isArray(initialQOverride) && initialQOverride.length > 0
                    ? initialQOverride.slice()
                    : (Array.isArray(window.customRobotConfig?.initial_q)
                    ? window.customRobotConfig.initial_q.slice()
                    : window.humanoidQ.slice());
                if (profile === 'near-limit') {
                    const limits = getJointLimits();
                    const rng = createRng(nearLimitSeed + 1);
                    const waistCount = viz.humanoidWaistLength || 0;
                    const leftCount = viz.humanoidLeftArmLength || 0;
                    const rightCount = viz.humanoidRightArmLength || 0;
                    const armEnd = waistCount + leftCount + rightCount;
                    for (let index = waistCount; index < armEnd; index += 1) {
                        const [lower, upper] = getLimitBounds(limits[index]);
                        if (!Number.isFinite(lower) || !Number.isFinite(upper) || upper <= lower) continue;
                        const range = upper - lower;
                        if (rng() < 0.45) {
                            const band = 0.05 + 0.10 * rng();
                            initial[index] = rng() < 0.5
                                ? lower + range * band
                                : upper - range * band;
                        }
                    }
                }
                for (let index = 0; index < Math.min(window.humanoidQ.length, initial.length); index += 1) {
                    window.humanoidQ[index] = initial[index];
                }
                viz.setHumanoidJoints(window.humanoidQ);
                window.updateGlobalVariables?.();
                window.updateRobotPose?.();
                await sleep(120);
            };

            const dqTranslation = (dqArray) => {
                if (!Array.isArray(dqArray) || !DQ) return [Number.NaN, Number.NaN, Number.NaN];
                const dq = DQ.createFromArray(dqArray);
                const translation = DQ.getTranslation(dq);
                return [
                    Number(translation?.[1]) || 0,
                    Number(translation?.[2]) || 0,
                    Number(translation?.[3]) || 0
                ];
            };

            const getAbsoluteTranslation = () => dqTranslation(viz._getHumanoidAbsolutePose(window.humanoidQ));

            const getRelativeDistance = () => {
                const translation = dqTranslation(viz._getHumanoidRelativePose(window.humanoidQ.slice()));
                if (!Array.isArray(translation) || translation.length < 3) return Number.NaN;
                const squared = translation.reduce((sum, value) => {
                    const numericValue = Number(value);
                    return Number.isFinite(numericValue) ? sum + numericValue * numericValue : sum;
                }, 0);
                return Number.isFinite(squared) ? Math.sqrt(squared) : Number.NaN;
            };

            const getWaistXPosition = () => {
                const value = Number(viz._evaluateHumanoidWaistXPosition?.(window.humanoidQ.slice()));
                return Number.isFinite(value) ? value : Number.NaN;
            };

            const taskUsesWaistCorridor = (task = {}) => (
                task.includeWaist === true &&
                (task.keepBodyZVertical === true || task.useWaistCorridor === true)
            );

            const configureBenchmarkWaistCorridor = (task = {}) => {
                const enabled = taskUsesWaistCorridor(task);
                if (!enabled) {
                    setCheckbox('humanoid-waist-x-corridor-toggle', false);
                    viz.setHumanoidWaistXCorridorEnabled?.(false);
                    return { enabled: false };
                }

                const waistX = getWaistXPosition();
                const center = Number.isFinite(waistX) ? waistX : 0;
                const min = center - waistCorridorBackwardMargin;
                const max = center + waistCorridorForwardMargin;
                dispatchInputValue('humanoid-waist-x-corridor-min-input', min.toFixed(4));
                dispatchInputValue('humanoid-waist-x-corridor-max-input', max.toFixed(4));
                viz.setHumanoidWaistXCorridorMin?.(min);
                viz.setHumanoidWaistXCorridorMax?.(max);
                setCheckbox('humanoid-waist-x-corridor-toggle', true);
                viz.setHumanoidWaistXCorridorEnabled?.(true);
                return { enabled: true, min, max, waistX };
            };

            const computeDmin = (includeWaist = false) => {
                const qFull = window.humanoidQ.slice();
                const { armStart, activeCount } = viz._getHumanoidActiveSliceInfo(includeWaist);
                const limits = viz._getHumanoidJointLimits?.()?.slice(armStart, armStart + activeCount) || null;
                const constraints = viz._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, limits);
                if (!Array.isArray(constraints) || constraints.length === 0) return Number.NaN;
                const dmin = constraints.reduce((minValue, item) => {
                    const distance = Number(item?.distance);
                    return Number.isFinite(distance) ? Math.min(minValue, distance) : minValue;
                }, Number.POSITIVE_INFINITY);
                return Number.isFinite(dmin) ? dmin : Number.NaN;
            };

            const computeJointClearanceMin = (includeWaist = false) => {
                const qFull = window.humanoidQ.slice();
                const { armStart, activeCount } = viz._getHumanoidActiveSliceInfo(includeWaist);
                const limits = viz._getHumanoidJointLimits?.()?.slice(armStart, armStart + activeCount) || [];
                let best = Number.POSITIVE_INFINITY;
                for (let index = 0; index < activeCount; index += 1) {
                    const [lower, upper] = getLimitBounds(limits[index]);
                    const q = Number(qFull[armStart + index]);
                    if (!Number.isFinite(q)) continue;
                    if (Number.isFinite(lower)) best = Math.min(best, q - lower);
                    if (Number.isFinite(upper)) best = Math.min(best, upper - q);
                }
                return Number.isFinite(best) ? Math.max(0, best) : Number.NaN;
            };

            const computeWaistClearanceMin = (diagnostics = {}, fallback = {}) => {
                const upper = Number(diagnostics.waistXDupper);
                const lower = Number(diagnostics.waistXDlower);
                const value = Number(fallback.value);
                const min = Number(fallback.min);
                const max = Number(fallback.max);
                const fallbackUpper = Number.isFinite(value) && Number.isFinite(max) ? max - value : Number.NaN;
                const fallbackLower = Number.isFinite(value) && Number.isFinite(min) ? value - min : Number.NaN;
                const values = [upper, lower, fallbackUpper, fallbackLower].filter(Number.isFinite);
                if (values.length === 0) return Number.NaN;
                return Math.max(0, Math.min(...values));
            };

            const computeWaistViolation = (fallback = {}) => {
                const value = Number(fallback.value);
                const min = Number(fallback.min);
                const max = Number(fallback.max);
                if (!Number.isFinite(value)) return Number.NaN;
                return Math.max(
                    0,
                    Number.isFinite(min) ? min - value : 0,
                    Number.isFinite(max) ? value - max : 0
                );
            };

            const extractTaskScale = (status = {}, diagnostics = {}) => {
                const candidates = [
                    status.taskScale,
                    status.task_scale,
                    diagnostics.taskScale,
                    diagnostics.task_scale,
                    diagnostics.predictiveTaskScale,
                    diagnostics.snsTaskScale
                ];
                for (const candidate of candidates) {
                    const value = Number(candidate);
                    if (Number.isFinite(value)) return value;
                }
                return Number.NaN;
            };

            const extractTRSafety = (status = {}, diagnostics = {}) => {
                const value = diagnostics.trSafetyStatus || diagnostics.tr_safety || status.safetyStatus || status.trSafetyStatus;
                return typeof value === 'string' && value.length > 0 ? value : '';
            };

            const finiteDiagnostic = (diagnostics = {}, key = '') => {
                const value = Number(diagnostics?.[key]);
                return Number.isFinite(value) ? value : Number.NaN;
            };

            const maxJointDelta = (beforeQ, afterQ) => {
                if (!Array.isArray(beforeQ) || !Array.isArray(afterQ)) return Number.NaN;
                const length = Math.min(beforeQ.length, afterQ.length);
                let maxDelta = 0;
                for (let index = 0; index < length; index += 1) {
                    const delta = Math.abs((Number(afterQ[index]) || 0) - (Number(beforeQ[index]) || 0));
                    if (delta > maxDelta) maxDelta = delta;
                }
                return maxDelta;
            };

            const hasSettledStatus = (status) => {
                if (!status || typeof status !== 'object') return false;
                return (
                    typeof status.solver === 'string' && status.solver.length > 0
                ) || (
                    typeof status.reason === 'string' && status.reason.length > 0
                ) || (
                    typeof status.message === 'string' && status.message.length > 0
                ) || typeof status.converged === 'boolean';
            };

            const waitForJogSettle = async (solverMode, qBefore, timeoutMs = 15000) => {
                const start = performance.now();
                let observedAsyncBusy = false;
                while (performance.now() - start < timeoutMs) {
                    await sleep(16);
                    const workerBusy = Boolean(viz._humanoidMJWorkerLoopActive);
                    const preWBusy = Boolean(viz._humanoidMJPreWLoopActive);
                    const asyncBusy = workerBusy || preWBusy;
                    observedAsyncBusy = observedAsyncBusy || asyncBusy;
                    const status = viz.lastIKSolveStatus || {};
                    const diagnostics = viz.lastHumanoidJogDiagnostics || {};
                    const qAfter = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
                    const qChanged = Number.isFinite(maxJointDelta(qBefore, qAfter)) && maxJointDelta(qBefore, qAfter) > 1e-9;
                    const diagnosticsReady = diagnostics && diagnostics.modeTag && diagnostics.modeTag !== 'idle';
                    const statusReady = hasSettledStatus(status);
                    const asyncMode = false;
                    if (asyncMode) {
                        const graceElapsed = (performance.now() - start) >= 32;
                        if (!asyncBusy && (statusReady || diagnosticsReady || qChanged) && (observedAsyncBusy || graceElapsed)) {
                            return;
                        }
                    } else if (statusReady || diagnosticsReady || qChanged) {
                        return;
                    }
                }
            };

            const invokeJog = async (task = {}) => {
                const controlMode = task.controlMode;
                if (controlMode === 'decoupled-upright') {
                    return Promise.resolve(viz.moveHumanoidAbsolutePoseAlongAxisDecoupledUpright('x', 'positive'));
                }
                if (controlMode === 'distance' || controlMode === 'distance-fullbody') {
                    const direction = task.jogDirection === 'positive' ? 'positive' : 'negative';
                    const includeWaist = controlMode === 'distance-fullbody' || task.includeWaist === true;
                    return Promise.resolve(viz.moveDualArmRelativeDistance(direction, includeWaist));
                }
                return Promise.resolve(viz.moveDualArmAbsolutePoseAlongAxis('x', 'positive'));
            };

            const runs = [];

            setCheckbox('humanoid-test-dual-arm-lock-toggle', false);
            dispatchSelectValue('humanoid-manifold-jog-metric-select', 'w_abs_hier_vfi');
            viz.setHumanoidTestDualArmLockEnabled?.(false);
            if (viz.vfiConfig) {
                viz.vfiConfig.humanoidTestDualArmLockEnabled = false;
            }

            for (const task of tasksInput) {
                const controllerOptions = task.comparisonOptions || {};
                dispatchSelectValue('humanoid-jog-solver-select', task.solverMode);
                dispatchSelectValue('dual-arm-control-mode-select', task.controlMode);
                const appliedTeachSpeedPercent = applyTeachSpeedPercent(teachSpeedPercent);
                setCheckbox('joint-limit-avoidance-toggle', controllerOptions.jointLimitAvoidanceEnabled);
                if (viz.vfiConfig) {
                    viz.vfiConfig.qpUseInvariantEquality = controllerOptions.qpUseInvariantEquality !== false;
                }
                await resetHumanoidState(task.profile, task.initialQOverride);
                const waistCorridor = configureBenchmarkWaistCorridor(task);
                await sleep(50);
                const initialQ = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
                const initialWaistX = getWaistXPosition();
                const waistCorridorActive = waistCorridor?.enabled === true;

                viz.lastIKSolveStatus = {};
                viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };

                const startTranslation = getAbsoluteTranslation();
                const startX = Number(startTranslation?.[0]) || 0;
                const startRelativePose = viz._getHumanoidRelativePose(window.humanoidQ.slice());
                const startRelativeDistance = getRelativeDistance();
                let previousProgress = 0;
                let sustainableTicks = 0;
                let stallTicks = 0;
                let reachedTarget = false;
                let completed = false;
                let completionTicks = Number.NaN;
                let terminalReason = 'max_iter';
                const steps = [];

                const tickBudget = Number.isFinite(Number(controllerOptions.maxTicks))
                    ? Math.max(1, Math.floor(Number(controllerOptions.maxTicks)))
                    : maxTicks;

                for (let tick = 1; tick <= tickBudget; tick += 1) {
                    viz.lastIKSolveStatus = {};
                    viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };
                    const qBefore = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
                    const distanceBefore = task.progressKind === 'relative-distance'
                        ? getRelativeDistance()
                        : Number.NaN;
                    const distanceDirectionSign = task.jogDirection === 'positive' ? 1 : -1;
                    const distanceCommandStep = Number(viz.getTeachTranslationStep?.());
                    const distanceCommandTarget = Number.isFinite(distanceBefore) && Number.isFinite(distanceCommandStep)
                        ? Math.max(0, distanceBefore + distanceDirectionSign * Math.abs(distanceCommandStep))
                        : Number.NaN;
                    const t0 = performance.now();
                    await invokeJog(task);
                    await waitForJogSettle(task.solverMode, qBefore);
                    const runtimeMs = performance.now() - t0;

                    const currentTranslation = getAbsoluteTranslation();
                    const xProgress = (Number(currentTranslation?.[0]) || 0) - startX;
                    const currentRelativeDistance = task.progressKind === 'relative-distance'
                        ? getRelativeDistance()
                        : Number.NaN;
                    const distanceProgress = Number.isFinite(startRelativeDistance) && Number.isFinite(currentRelativeDistance)
                        ? (task.jogDirection === 'positive'
                            ? currentRelativeDistance - startRelativeDistance
                            : startRelativeDistance - currentRelativeDistance)
                        : Number.NaN;
                    const progressValue = task.progressKind === 'relative-distance' ? distanceProgress : xProgress;
                    const stepProgress = progressValue - previousProgress;
                    previousProgress = progressValue;
                    const status = viz.lastIKSolveStatus || {};
                    const diagnostics = viz.lastHumanoidJogDiagnostics || {};
                    const diagnosticCommandError = Number.isFinite(Number(diagnostics.commandError))
                        ? Number(diagnostics.commandError)
                        : (Number.isFinite(Number(status.finalResidual)) ? Number(status.finalResidual) : Number.NaN);
                    const distanceCommandError = Number.isFinite(distanceCommandTarget) && Number.isFinite(currentRelativeDistance)
                        ? Math.abs(currentRelativeDistance - distanceCommandTarget)
                        : Number.NaN;
                    const commandError = task.progressKind === 'relative-distance' && Number.isFinite(distanceCommandError)
                        ? distanceCommandError
                        : diagnosticCommandError;
                    const diagnosticInvariantError = Number.isFinite(Number(diagnostics.invariantError))
                        ? Number(diagnostics.invariantError)
                        : (Number.isFinite(Number(status.invariantResidual)) ? Number(status.invariantResidual) : Number.NaN);
                    const bodyError = Number.isFinite(Number(diagnostics.bodyResidualNorm))
                        ? Number(diagnostics.bodyResidualNorm)
                        : Number.NaN;
                    const waistXValue = waistCorridorActive
                        ? getWaistXPosition()
                        : Number.NaN;
                    const waistXMin = Number(viz.vfiConfig?.waistXMin);
                    const waistXMax = Number(viz.vfiConfig?.waistXMax);
                    const fallbackWaist = {
                        value: waistXValue,
                        min: waistXMin,
                        max: waistXMax
                    };
                    const fallbackWaistViolation = waistCorridorActive
                        ? computeWaistViolation(fallbackWaist)
                        : Number.NaN;
                    const waistXViolation = Number.isFinite(Number(diagnostics.waistXViolation))
                        ? Number(diagnostics.waistXViolation)
                        : fallbackWaistViolation;
                    const currentRelativePose = viz._getHumanoidRelativePose(window.humanoidQ.slice());
                    const relativePoseChange = Array.isArray(startRelativePose) && Array.isArray(currentRelativePose)
                        ? (() => {
                            const signedError = viz._signedPoseError?.(startRelativePose, currentRelativePose);
                            if (!Array.isArray(signedError)) return Number.NaN;
                            const norm = typeof viz._computePoseErrorNorm === 'function'
                                ? Number(viz._computePoseErrorNorm(signedError))
                                : vectorNorm(signedError);
                            return Number.isFinite(norm) ? norm : Number.NaN;
                        })()
                        : Number.NaN;
                    const invariantError = Number.isFinite(diagnosticInvariantError)
                        ? diagnosticInvariantError
                        : relativePoseChange;
                    const dmin = Number.isFinite(Number(diagnostics.vfiMinDistance))
                        ? Number(diagnostics.vfiMinDistance)
                        : computeDmin(task.includeWaist);
                    const jointClearanceMin = computeJointClearanceMin(task.includeWaist);
                    const waistClearanceMin = waistCorridorActive
                        ? computeWaistClearanceMin(diagnostics, fallbackWaist)
                        : Number.NaN;
                    const maxViolation = Math.max(
                        0,
                        Number.isFinite(waistXViolation) ? waistXViolation : 0,
                        Number.isFinite(Number(diagnostics.maxViolation)) ? Number(diagnostics.maxViolation) : 0,
                        Number.isFinite(Number(status.maxViolation)) ? Number(status.maxViolation) : 0
                    );
                    const taskScale = extractTaskScale(status, diagnostics);
                    const trSafetyStatus = extractTRSafety(status, diagnostics);
                    const tickSolveMs = runtimeMs;
                    const stepEvaluation = evaluateStep({
                        stepProgress,
                        minEffectiveProgress,
                        stallTicks,
                        maxStallTicks: controllerOptions.maxStallTicks,
                        status
                    });
                    const scenarioTargetXMeters = Number.isFinite(Number(controllerOptions.targetXMeters))
                        ? Math.max(1e-8, Number(controllerOptions.targetXMeters))
                        : targetXMeters;
                    const completionEvaluation = evaluateCompletion({
                        xProgress: progressValue,
                        targetXMeters: scenarioTargetXMeters,
                        commandError,
                        commandTolerance: controllerOptions.commandTolerance,
                        bodyError,
                        bodyTolerance,
                        waistXViolation,
                        enforceBody: task.keepBodyZVertical === true,
                        enforceWaist: waistCorridorActive
                    });

                    if (stepEvaluation.progressed) {
                        sustainableTicks += 1;
                    }
                    stallTicks = stepEvaluation.nextStallTicks;

                    steps.push({
                        tick,
                        runtimeMs,
                        tickSolveMs,
                        xProgress,
                        distanceBefore,
                        relativeDistance: currentRelativeDistance,
                        distanceCommandTarget,
                        distanceCommandError,
                        distanceProgress,
                        progressValue,
                        stepProgress,
                        commandError,
                        bodyError,
                        waistXValue,
                        waistXViolation,
                        relativePoseChange,
                        diagnosticInvariantError,
                        invariantError,
                        progressRatio: completionEvaluation.progressRatio,
                        dmin,
                        jointClearanceMin,
                        waistClearanceMin,
                        maxViolation,
                        taskScale,
                        trSafetyStatus,
                        cctsmW: finiteDiagnostic(diagnostics, 'cctsmW'),
                        cctsmWvfi: finiteDiagnostic(diagnostics, 'cctsmWvfi'),
                        cctsmWAbsHier: finiteDiagnostic(diagnostics, 'cctsmWAbsHier'),
                        cctsmWAbsHierVfi: finiteDiagnostic(diagnostics, 'cctsmWAbsHierVfi'),
                        cctsmWAbsArm: finiteDiagnostic(diagnostics, 'cctsmWAbsArm'),
                        cctsmWAbsArmVfi: finiteDiagnostic(diagnostics, 'cctsmWAbsArmVfi'),
                        cctsmWCmd: finiteDiagnostic(diagnostics, 'cctsmWCmd'),
                        ok: stepEvaluation.ok,
                        solver: typeof status.solver === 'string' ? status.solver : task.solverMode,
                        reason: stepEvaluation.reason,
                        message: typeof status.message === 'string' ? status.message : '',
                        diagnosticLabel: typeof diagnostics.diagnosticLabel === 'string' ? diagnostics.diagnosticLabel : '',
                        converged: status.converged !== false
                    });

                    if (completionEvaluation.completed && completed !== true) {
                        reachedTarget = true;
                        completed = true;
                        completionTicks = tick;
                        terminalReason = 'completed';
                        if (!fixedTicks) break;
                    }
                    if (!fixedTicks && completionEvaluation.reachedTarget) {
                        reachedTarget = true;
                        terminalReason = 'target_not_converged';
                        break;
                    }
                    if (!fixedTicks && stepEvaluation.shouldStop) {
                        terminalReason = stepEvaluation.ok ? 'stall' : stepEvaluation.reason;
                        break;
                    }
                }
                if (fixedTicks) {
                    terminalReason = completed ? 'fixed_ticks_completed' : 'fixed_ticks';
                }

                const finalStep = steps[steps.length - 1] || {};
                const meanSolveMs = meanOf(steps.map((step) => Number(step.tickSolveMs)));
                const p95SolveMs = percentileOf(steps.map((step) => Number(step.tickSolveMs)), 0.95);
                runs.push({
                    mode: task.mode,
                    profile: task.profile,
                    controllerLabel: task.label,
                    controllerKey: task.key,
                    solverMode: task.solverMode,
                    controlMode: task.controlMode,
                    progressKind: task.progressKind || 'absolute-x',
                    jogDirection: task.jogDirection || 'positive',
                    initialQ,
                    initialWaistX,
                    initialRelativeDistance: Number.isFinite(startRelativeDistance) ? startRelativeDistance : Number.NaN,
                    sustainableTicks,
                    reachedTarget,
                    completed,
                    completionTicks,
                    finalXProgress: Number.isFinite(finalStep.xProgress) ? finalStep.xProgress : 0,
                    finalDistanceProgress: Number.isFinite(finalStep.distanceProgress) ? finalStep.distanceProgress : Number.NaN,
                    finalCommandError: Number.isFinite(finalStep.commandError) ? finalStep.commandError : Number.NaN,
                    finalBodyError: Number.isFinite(finalStep.bodyError) ? finalStep.bodyError : Number.NaN,
                    finalRelativePoseChange: Number.isFinite(finalStep.relativePoseChange) ? finalStep.relativePoseChange : Number.NaN,
                    finalInvariantError: Number.isFinite(finalStep.invariantError) ? finalStep.invariantError : Number.NaN,
                    finalDmin: Number.isFinite(finalStep.dmin) ? finalStep.dmin : Number.NaN,
                    finalJointClearanceMin: Number.isFinite(finalStep.jointClearanceMin) ? finalStep.jointClearanceMin : Number.NaN,
                    finalWaistClearanceMin: Number.isFinite(finalStep.waistClearanceMin) ? finalStep.waistClearanceMin : Number.NaN,
                    finalCCTSMW: Number.isFinite(finalStep.cctsmW) ? finalStep.cctsmW : Number.NaN,
                    finalCCTSMWvfi: Number.isFinite(finalStep.cctsmWvfi) ? finalStep.cctsmWvfi : Number.NaN,
                    finalCCTSMWAbsHier: Number.isFinite(finalStep.cctsmWAbsHier) ? finalStep.cctsmWAbsHier : Number.NaN,
                    finalCCTSMWAbsHierVfi: Number.isFinite(finalStep.cctsmWAbsHierVfi) ? finalStep.cctsmWAbsHierVfi : Number.NaN,
                    finalCCTSMWAbsArm: Number.isFinite(finalStep.cctsmWAbsArm) ? finalStep.cctsmWAbsArm : Number.NaN,
                    finalCCTSMWAbsArmVfi: Number.isFinite(finalStep.cctsmWAbsArmVfi) ? finalStep.cctsmWAbsArmVfi : Number.NaN,
                    finalCCTSMWCmd: Number.isFinite(finalStep.cctsmWCmd) ? finalStep.cctsmWCmd : Number.NaN,
                    maxViolation: Number.isFinite(finalStep.maxViolation) ? finalStep.maxViolation : Number.NaN,
                    trSafetyStatus: finalStep.trSafetyStatus || '',
                    progressRatio: Number.isFinite(finalStep.progressRatio) ? finalStep.progressRatio : Number.NaN,
                    meanSolveMs,
                    p95SolveMs,
                    maxTicks: tickBudget,
                    fixedTicks,
                    teachSpeedPercent: appliedTeachSpeedPercent,
                    targetXMeters: Number.isFinite(Number(controllerOptions.targetXMeters))
                        ? Number(controllerOptions.targetXMeters)
                        : targetXMeters,
                    waistCorridor,
                    terminalReason: terminalReason || finalStep.reason || 'unknown',
                    terminalLabel: finalStep.diagnosticLabel || 'unknown',
                    steps
                });
                console.log(
                    `[scenario] ${task.label} | ${task.profile} | ${task.label} | ${task.solverMode} `
                    + `completed=${completed ? 'yes' : 'no'} progress=${((Number.isFinite(finalStep.progressValue) ? finalStep.progressValue : 0)).toFixed(4)}`
                );
            }

            return { runs };
        },
        {
            tasks,
            targetXMeters: TARGET_X_METERS,
            maxTicks: MAX_TICKS,
            teachSpeedPercent: TEACH_SPEED_PERCENT,
            minEffectiveProgress: MIN_EFFECTIVE_PROGRESS,
            maxStallTicks: MAX_STALL_TICKS,
            commandTolerance: COMMAND_TOLERANCE,
            bodyTolerance: BODY_TOLERANCE,
            fixedTicks: FIXED_TICKS,
            waistCorridorBackwardMargin: WAIST_CORRIDOR_BACKWARD_MARGIN,
            waistCorridorForwardMargin: WAIST_CORRIDOR_FORWARD_MARGIN,
            nearLimitSeed: NEAR_LIMIT_SEED
        }
    );

    for (const run of result.runs) {
        const progress = Number.isFinite(Number(run.finalDistanceProgress))
            ? Number(run.finalDistanceProgress)
            : Number(run.finalXProgress);
        console.log(
            `[done] ${run.mode} | ${run.profile} | ${run.controllerLabel} `
            + `completed=${run.completed ? 'yes' : 'no'} progress=${(Number.isFinite(progress) ? progress : 0).toFixed(4)}`
        );
    }

    const { summaryRows, markdown } = await writeBenchmarkOutputs(result.runs);

    console.log(markdown);
    console.log(`\nSaved JSON to ${OUTPUT_JSON}`);
    console.log(`Saved Markdown to ${OUTPUT_MD}`);
    console.log(`Saved CSV to ${OUTPUT_CSV}`);
    console.log(`Saved LaTeX to ${OUTPUT_LATEX}`);
} finally {
    await browser.close();
    await stopServer(server);
}
