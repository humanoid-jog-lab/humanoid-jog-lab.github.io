#!/usr/bin/env node

import fs from 'node:fs/promises';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';

import { chromium } from 'playwright';

import { formatBenchmarkMarkdownTable } from '../src/core/humanoid-jog-benchmark-metrics.js';
import {
    buildHumanoidControllerComparisonTasks,
    getHumanoidControllerComparisonColumns,
    resolveHumanoidControllerComparisonControllerOptions,
    summarizeHumanoidControllerComparisonRows
} from '../src/core/humanoid-jog-controller-comparison.js';

const REPO_ROOT = '/home/echoz/dq_vizlab';
const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173';
const OUTPUT_JSON = process.env.COMPARISON_JSON
    || path.join(REPO_ROOT, '2026_humanoid_robot', 'figure', 'sim', 'controller_comparison_x30cm.json');
const OUTPUT_MD = process.env.COMPARISON_MD
    || path.join(REPO_ROOT, '2026_humanoid_robot', 'figure', 'sim', 'controller_comparison_x30cm.md');
const TARGET_X_METERS = Number.isFinite(Number(process.env.COMPARISON_TARGET_X_METERS))
    ? Math.max(0.01, Number(process.env.COMPARISON_TARGET_X_METERS))
    : 0.30;
const MAX_TICKS = Number.isFinite(Number(process.env.COMPARISON_MAX_TICKS))
    ? Math.max(1, Math.floor(Number(process.env.COMPARISON_MAX_TICKS)))
    : 60;
const MIN_EFFECTIVE_PROGRESS = Number.isFinite(Number(process.env.COMPARISON_MIN_PROGRESS))
    ? Math.max(1e-8, Number(process.env.COMPARISON_MIN_PROGRESS))
    : 1e-5;
const MAX_STALL_TICKS = Number.isFinite(Number(process.env.COMPARISON_MAX_STALL_TICKS))
    ? Math.max(1, Math.floor(Number(process.env.COMPARISON_MAX_STALL_TICKS)))
    : 3;
const COMMAND_TOLERANCE = Number.isFinite(Number(process.env.COMPARISON_COMMAND_TOLERANCE))
    ? Math.max(0, Number(process.env.COMPARISON_COMMAND_TOLERANCE))
    : 5e-4;
const BODY_TOLERANCE = Number.isFinite(Number(process.env.COMPARISON_BODY_TOLERANCE))
    ? Math.max(0, Number(process.env.COMPARISON_BODY_TOLERANCE))
    : 5e-5;
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

async function writeBenchmarkOutputs(resultRuns) {
    const summaryRows = summarizeHumanoidControllerComparisonRows(resultRuns);
    const markdown = buildMarkdownRows(summaryRows);
    await fs.mkdir(path.dirname(OUTPUT_JSON), { recursive: true });
    await fs.writeFile(OUTPUT_JSON, JSON.stringify({
        meta: {
            targetXMeters: TARGET_X_METERS,
            maxTicks: MAX_TICKS,
            minEffectiveProgress: MIN_EFFECTIVE_PROGRESS,
            maxStallTicks: MAX_STALL_TICKS,
            commandTolerance: COMMAND_TOLERANCE,
            bodyTolerance: BODY_TOLERANCE,
            nearLimitSeed: NEAR_LIMIT_SEED,
            scenarioTargets: {
                absolute: 0.075,
                'decoupled-upright': TARGET_X_METERS
            }
        },
        rows: summaryRows,
        runs: resultRuns
    }, null, 2));
    await fs.mkdir(path.dirname(OUTPUT_MD), { recursive: true });
    await fs.writeFile(OUTPUT_MD, `${markdown}\n`);
    return { summaryRows, markdown };
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
    }).map((task) => ({
        ...task,
        comparisonOptions: resolveHumanoidControllerComparisonControllerOptions({
            controllerKey: task.key,
            mode: task.mode,
            includeWaist: task.includeWaist,
            maxTicks: MAX_TICKS,
            commandTolerance: COMMAND_TOLERANCE,
            maxStallTicks: MAX_STALL_TICKS
        })
    }));
    if (tasks.length === 0) {
        throw new Error('No controller-comparison tasks matched the provided filters.');
    }
    const result = await page.evaluate(
        async ({
            tasks: tasksInput,
            targetXMeters,
            maxTicks,
            minEffectiveProgress,
            maxStallTicks,
            commandTolerance,
            bodyTolerance,
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

            const setCheckbox = (id, checked) => {
                const element = document.getElementById(id);
                if (!element) return;
                element.checked = checked;
                element.dispatchEvent(new Event('change', { bubbles: true }));
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

            const resetHumanoidState = async (profile = 'near-limit') => {
                const initial = Array.isArray(window.customRobotConfig?.initial_q)
                    ? window.customRobotConfig.initial_q.slice()
                    : window.humanoidQ.slice();
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
                    const asyncMode = solverMode === 'mj_restore' || solverMode === 'mj_restore_pre_w';
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

            const invokeJog = async (controlMode) => {
                if (controlMode === 'decoupled-upright') {
                    return Promise.resolve(viz.moveHumanoidAbsolutePoseAlongAxisDecoupledUpright('x', 'positive'));
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
                setCheckbox('humanoid-waist-x-corridor-toggle', task.includeWaist === true && task.keepBodyZVertical === true);
                setCheckbox('joint-limit-avoidance-toggle', controllerOptions.jointLimitAvoidanceEnabled);
                if (viz.vfiConfig) {
                    viz.vfiConfig.qpUseInvariantEquality = controllerOptions.qpUseInvariantEquality !== false;
                }
                await resetHumanoidState(task.profile);
                await sleep(50);

                viz.lastIKSolveStatus = {};
                viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };

                const startTranslation = getAbsoluteTranslation();
                const startX = Number(startTranslation?.[0]) || 0;
                const startRelativePose = viz._getHumanoidRelativePose(window.humanoidQ.slice());
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
                    const t0 = performance.now();
                    await invokeJog(task.controlMode);
                    await waitForJogSettle(task.solverMode, qBefore);
                    const runtimeMs = performance.now() - t0;

                    const currentTranslation = getAbsoluteTranslation();
                    const xProgress = (Number(currentTranslation?.[0]) || 0) - startX;
                    const stepProgress = xProgress - previousProgress;
                    previousProgress = xProgress;
                    const status = viz.lastIKSolveStatus || {};
                    const diagnostics = viz.lastHumanoidJogDiagnostics || {};
                    const commandError = Number.isFinite(Number(diagnostics.commandError))
                        ? Number(diagnostics.commandError)
                        : (Number.isFinite(Number(status.finalResidual)) ? Number(status.finalResidual) : Number.NaN);
                    const bodyError = Number.isFinite(Number(diagnostics.bodyResidualNorm))
                        ? Number(diagnostics.bodyResidualNorm)
                        : Number.NaN;
                    const waistXViolation = Number.isFinite(Number(diagnostics.waistXViolation))
                        ? Number(diagnostics.waistXViolation)
                        : Number.NaN;
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
                    const dmin = Number.isFinite(Number(diagnostics.vfiMinDistance))
                        ? Number(diagnostics.vfiMinDistance)
                        : computeDmin(task.includeWaist);
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
                        xProgress,
                        targetXMeters: scenarioTargetXMeters,
                        commandError,
                        commandTolerance: controllerOptions.commandTolerance,
                        bodyError,
                        bodyTolerance,
                        waistXViolation,
                        enforceBody: task.keepBodyZVertical === true,
                        enforceWaist: task.includeWaist === true && task.keepBodyZVertical === true
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
                        stepProgress,
                        commandError,
                        bodyError,
                        waistXViolation,
                        relativePoseChange,
                        progressRatio: completionEvaluation.progressRatio,
                        dmin,
                        ok: stepEvaluation.ok,
                        solver: typeof status.solver === 'string' ? status.solver : task.solverMode,
                        reason: stepEvaluation.reason,
                        message: typeof status.message === 'string' ? status.message : '',
                        diagnosticLabel: typeof diagnostics.diagnosticLabel === 'string' ? diagnostics.diagnosticLabel : '',
                        converged: status.converged !== false
                    });

                    if (completionEvaluation.completed) {
                        reachedTarget = true;
                        completed = true;
                        completionTicks = tick;
                        terminalReason = 'completed';
                        break;
                    }
                    if (completionEvaluation.reachedTarget) {
                        reachedTarget = true;
                        terminalReason = 'target_not_converged';
                        break;
                    }
                    if (stepEvaluation.shouldStop) {
                        terminalReason = stepEvaluation.ok ? 'stall' : stepEvaluation.reason;
                        break;
                    }
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
                    sustainableTicks,
                    reachedTarget,
                    completed,
                    completionTicks,
                    finalXProgress: Number.isFinite(finalStep.xProgress) ? finalStep.xProgress : 0,
                    finalCommandError: Number.isFinite(finalStep.commandError) ? finalStep.commandError : Number.NaN,
                    finalBodyError: Number.isFinite(finalStep.bodyError) ? finalStep.bodyError : Number.NaN,
                    finalRelativePoseChange: Number.isFinite(finalStep.relativePoseChange) ? finalStep.relativePoseChange : Number.NaN,
                    finalDmin: Number.isFinite(finalStep.dmin) ? finalStep.dmin : Number.NaN,
                    progressRatio: Number.isFinite(finalStep.progressRatio) ? finalStep.progressRatio : Number.NaN,
                    meanSolveMs,
                    p95SolveMs,
                    maxTicks: tickBudget,
                    targetXMeters: Number.isFinite(Number(controllerOptions.targetXMeters))
                        ? Number(controllerOptions.targetXMeters)
                        : targetXMeters,
                    terminalReason: terminalReason || finalStep.reason || 'unknown',
                    terminalLabel: finalStep.diagnosticLabel || 'unknown',
                    steps
                });
                console.log(
                    `[scenario] ${task.label} | ${task.profile} | ${task.label} | ${task.solverMode} `
                    + `completed=${completed ? 'yes' : 'no'} progress=${((Number.isFinite(finalStep.xProgress) ? finalStep.xProgress : 0)).toFixed(4)}`
                );
            }

            return { runs };
        },
        {
            tasks,
            targetXMeters: TARGET_X_METERS,
            maxTicks: MAX_TICKS,
            minEffectiveProgress: MIN_EFFECTIVE_PROGRESS,
            maxStallTicks: MAX_STALL_TICKS,
            commandTolerance: COMMAND_TOLERANCE,
            bodyTolerance: BODY_TOLERANCE,
            nearLimitSeed: NEAR_LIMIT_SEED
        }
    );

    for (const run of result.runs) {
        console.log(
            `[done] ${run.mode} | ${run.profile} | ${run.controllerLabel} `
            + `completed=${run.completed ? 'yes' : 'no'} progress=${run.finalXProgress.toFixed(4)}`
        );
    }

    const { summaryRows, markdown } = await writeBenchmarkOutputs(result.runs);

    console.log(markdown);
    console.log(`\nSaved JSON to ${OUTPUT_JSON}`);
    console.log(`Saved Markdown to ${OUTPUT_MD}`);
} finally {
    await browser.close();
    await stopServer(server);
}
