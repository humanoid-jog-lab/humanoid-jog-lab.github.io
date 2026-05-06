#!/usr/bin/env node

import fs from 'node:fs/promises';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';

import { chromium } from 'playwright';

const REPO_ROOT = process.env.SURFACE_COMPARISON_REPO_ROOT || process.cwd();
const OUTPUT_DIR = process.env.SURFACE_COMPARISON_OUTPUT_DIR
    || path.join(REPO_ROOT, 'artifacts', 'humanoid-surface-polishing');
const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173/app.html';
const TICKS = Number.isFinite(Number(process.env.SURFACE_TICKS))
    ? Math.max(1, Math.floor(Number(process.env.SURFACE_TICKS)))
    : 50;
const SPEED_PERCENT = Number.isFinite(Number(process.env.SURFACE_SPEED_PERCENT))
    ? Math.max(1, Math.min(100, Math.round(Number(process.env.SURFACE_SPEED_PERCENT))))
    : 50;
const OUTPUT_STEM = process.env.SURFACE_OUTPUT_STEM
    || `cooperative_surface_dq_setpoint_waist_cylinder_capture_${TICKS}ticks_all_comparison_current`;
const OUTPUT_JSON = process.env.SURFACE_JSON || path.join(OUTPUT_DIR, `${OUTPUT_STEM}.json`);
const OUTPUT_CSV = process.env.SURFACE_CSV || path.join(OUTPUT_DIR, `${OUTPUT_STEM}.csv`);
const TASK_VALID_DEVIATION_TOLERANCE = Number.isFinite(Number(process.env.SURFACE_TASK_VALID_TOLERANCE))
    ? Math.max(0, Number(process.env.SURFACE_TASK_VALID_TOLERANCE))
    : 1e-6;
const SOLVER_FILTER = new Set(
    String(process.env.SURFACE_SOLVER_FILTER || '')
        .split(',')
        .map((value) => value.trim())
        .filter(Boolean)
);

const INITIAL_Q = [
    -0.3, 0.9, -0.6, 0,
    -0.51865, 0.66105, 0.90855, -1.5711, -0.93562, 0.67402, 1.40137,
    0.56421, -0.68286, -0.98744, 1.58165, 1.00933, -0.71114, -1.35432,
    0, 0
];

const CONTROLLERS = [
    { label: 'ProxQP', solverMode: 'proxqp' },
    { label: 'ProxQP+w', solverMode: 'proxqp_w' },
    { label: 'ProxQP+w_vfi', solverMode: 'proxqp_w_vfi' }
].filter((item) => SOLVER_FILTER.size === 0 || SOLVER_FILTER.has(item.solverMode) || SOLVER_FILTER.has(item.label));

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

function csvEscape(value) {
    const text = String(value ?? '');
    return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function buildCsv(runs) {
    const header = [
        'Controller',
        'Success / ticks',
        'Task-valid ticks',
        'Progress (u)',
        'Avg. task deviation',
        'Avg. invariant deviation',
        'Avg. joint clearance',
        'Avg. waist clearance',
        'Min joint clearance',
        'Min waist clearance',
        'Max violation',
        'Mean solve ms',
        'P95 solve ms',
        'Terminal reason',
        'Last reason'
    ];
    const rows = runs.map(({ summary }) => [
        summary.label,
        `${summary.completed ? 'yes' : 'no'} / ${summary.successTicks}`,
        summary.taskValidTicks,
        summary.uProgress,
        summary.avgTaskDeviation,
        summary.avgInvariantDeviation,
        summary.avgJointLimitClearance,
        summary.avgWaistCorridorClearance,
        summary.minJointLimitClearance,
        summary.minWaistCorridorClearance,
        summary.maxViolation,
        summary.meanSolveMs,
        summary.p95SolveMs,
        summary.terminalReason,
        summary.lastReason
    ]);
    return `${[header, ...rows].map((row) => row.map(csvEscape).join(',')).join('\n')}\n`;
}

const browser = await chromium.launch({
    headless: process.env.PLAYWRIGHT_HEADLESS !== '0',
    args: ['--use-angle=swiftshader', '--ignore-gpu-blocklist']
});

let server = null;
try {
    server = await startServerIfNeeded();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    const browserErrors = [];
    page.on('console', (message) => {
        const text = message.text();
        if (message.type() === 'log' && text.startsWith('[surface]')) {
            process.stdout.write(`[page] ${text}\n`);
        } else if (message.type() === 'error') {
            browserErrors.push(text);
        }
    });
    page.on('pageerror', (error) => {
        browserErrors.push(error?.message || String(error));
    });

    await ensureHumanoidPageReady(page);

    const result = await page.evaluate(
        async ({ controllers, ticksRequested, speedPercent, initialQ, taskValidDeviationTolerance }) => {
            const viz = window.visualizer;
            const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

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
                viz.teachSpeedPercent = p;
                return p;
            };
            const meanOf = (values = []) => {
                const finite = values.filter((value) => Number.isFinite(value));
                if (finite.length === 0) return Number.NaN;
                return finite.reduce((sum, value) => sum + value, 0) / finite.length;
            };
            const percentileOf = (values = [], percentile = 0.95) => {
                const finite = values.filter((value) => Number.isFinite(value)).sort((left, right) => left - right);
                if (finite.length === 0) return Number.NaN;
                const rank = Math.min(finite.length - 1, Math.max(0, Math.ceil(percentile * finite.length) - 1));
                return finite[rank];
            };
            const getLimitBounds = (limit) => {
                if (!limit || typeof limit !== 'object') return [Number.NaN, Number.NaN];
                return [Number(limit.lower), Number(limit.upper)];
            };
            const computeJointClearanceMin = () => {
                const qFull = window.humanoidQ.slice();
                const activeIndices = viz._humanoidCoordinatedDualArmIndices?.(true) || [];
                const limits = viz._getHumanoidJointLimits?.() || [];
                let best = Number.POSITIVE_INFINITY;
                for (const qIndex of activeIndices) {
                    const [lower, upper] = getLimitBounds(limits[qIndex]);
                    const q = Number(qFull[qIndex]);
                    if (!Number.isFinite(q)) continue;
                    if (Number.isFinite(lower)) best = Math.min(best, q - lower);
                    if (Number.isFinite(upper)) best = Math.min(best, upper - q);
                }
                return Number.isFinite(best) ? Math.max(0, best) : Number.NaN;
            };
            const getWaistXPosition = () => {
                const value = Number(viz._evaluateHumanoidWaistXPosition?.(window.humanoidQ.slice()));
                return Number.isFinite(value) ? value : Number.NaN;
            };
            const computeWaistClearanceMin = (diagnostics = {}) => {
                const value = getWaistXPosition();
                const min = Number(viz.vfiConfig?.waistXMin);
                const max = Number(viz.vfiConfig?.waistXMax);
                const values = [
                    Number(diagnostics.waistXDupper),
                    Number(diagnostics.waistXDlower),
                    Number.isFinite(value) && Number.isFinite(max) ? max - value : Number.NaN,
                    Number.isFinite(value) && Number.isFinite(min) ? value - min : Number.NaN
                ].filter(Number.isFinite);
                if (values.length === 0) return Number.NaN;
                return Math.max(0, Math.min(...values));
            };
            const computeWaistViolation = () => {
                const value = getWaistXPosition();
                const min = Number(viz.vfiConfig?.waistXMin);
                const max = Number(viz.vfiConfig?.waistXMax);
                if (!Number.isFinite(value)) return Number.NaN;
                return Math.max(
                    0,
                    Number.isFinite(min) ? min - value : 0,
                    Number.isFinite(max) ? value - max : 0
                );
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
            const waitForJogSettle = async (solverMode, qBefore, timeoutMs = 20000) => {
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
            const resetForController = async () => {
                dispatchSelectValue('dual-arm-control-mode-select', 'cooperative-ruled-surface-polishing-dq-setpoint');
                dispatchSelectValue('humanoid-manifold-jog-metric-select', 'w_abs_hier_vfi');
                setCheckbox('joint-limit-avoidance-toggle', true);
                setCheckbox('humanoid-waist-x-corridor-toggle', true);
                dispatchInputValue('humanoid-waist-x-corridor-min-input', '-0.5');
                dispatchInputValue('humanoid-waist-x-corridor-max-input', '0.2');
                viz.setHumanoidWaistXCorridorEnabled?.(true);
                viz.setHumanoidWaistXCorridorMin?.(-0.5);
                viz.setHumanoidWaistXCorridorMax?.(0.2);
                await Promise.resolve(viz.setHumanoidCooperativeRuledSurfaceSurfaceSource?.('cylindrical-patch'));
                window.humanoidCooperativeRuledSurfaceSource = 'cylindrical-patch';
                for (let index = 0; index < Math.min(window.humanoidQ.length, initialQ.length); index += 1) {
                    window.humanoidQ[index] = Number(initialQ[index]) || 0;
                }
                viz.setHumanoidJoints(window.humanoidQ);
                window.updateGlobalVariables?.();
                window.updateRobotPose?.();
                viz.humanoidCooperativeRuledSurfacePolishingState = null;
                viz.resetHumanoidSurfaceSlideReference?.();
                await sleep(150);
                const capturedPose = await Promise.resolve(
                    viz.captureHumanoidCooperativeRuledSurfaceObjectPoseFromCurrentHands?.({ flipNormal: false })
                );
                if (capturedPose) {
                    viz.setHumanoidCooperativeRuledSurfaceObjectPose?.(capturedPose);
                }
                const state = viz._ensureHumanoidCooperativeRuledSurfacePolishingState?.();
                viz._updateHumanoidSurfaceSlideVisualization?.();
                await sleep(100);
                return {
                    capturedPose,
                    state: state ? {
                        u: Number(state.u),
                        vCenter: Number(state.vCenter),
                        stripSpacing: Number(state.stripSpacing),
                        normalOffset: Number(state.normalOffset)
                    } : null,
                    waistX: getWaistXPosition()
                };
            };
            const extractCommandError = (status = {}, diagnostics = {}) => {
                const candidates = [diagnostics.commandError, status.finalResidual, status.commandError];
                for (const item of candidates) {
                    const value = Number(item);
                    if (Number.isFinite(value)) return value;
                }
                return Number.NaN;
            };
            const extractInvariantError = (status = {}, diagnostics = {}) => {
                const candidates = [
                    diagnostics.invariantError,
                    diagnostics.bodyResidualNorm,
                    status.invariantResidual,
                    status.bodyResidualNorm
                ];
                for (const item of candidates) {
                    const value = Number(item);
                    if (Number.isFinite(value)) return value;
                }
                return Number.NaN;
            };
            const extractReason = (status = {}) => {
                if (typeof status.reason === 'string' && status.reason.length > 0) return status.reason;
                if (status.converged === true) return 'converged';
                if (status.converged === false) return 'not_converged';
                return 'unknown';
            };
            const formatTaskValidTicks = (steps = []) => {
                if (!Array.isArray(steps) || steps.length === 0) return 'n/a';
                const validCount = steps.filter((step) => (
                    Number.isFinite(Number(step.commandError)) &&
                    Number(step.commandError) <= taskValidDeviationTolerance
                )).length;
                return `${validCount} / ${steps.length}`;
            };

            applyTeachSpeedPercent(speedPercent);
            const runs = [];
            for (const controller of controllers) {
                dispatchSelectValue('humanoid-jog-solver-select', controller.solverMode);
                const start = await resetForController();
                const startState = start.state || {};
                const steps = [];
                let terminalReason = 'max_ticks';
                let successTicks = 0;
                let previousUProgress = 0;

                for (let tick = 1; tick <= ticksRequested; tick += 1) {
                    viz.lastIKSolveStatus = {};
                    viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };
                    const qBefore = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
                    const t0 = performance.now();
                    await Promise.resolve(viz.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive'));
                    await waitForJogSettle(controller.solverMode, qBefore);
                    const runtimeMs = performance.now() - t0;
                    const status = viz.lastIKSolveStatus || {};
                    const diagnostics = viz.lastHumanoidJogDiagnostics || {};
                    const state = viz.humanoidCooperativeRuledSurfacePolishingState || {};
                    const uProgress = Number(state.u) - Number(startState.u);
                    const stepProgress = Number.isFinite(uProgress) ? uProgress - previousUProgress : Number.NaN;
                    previousUProgress = Number.isFinite(uProgress) ? uProgress : previousUProgress;
                    const progressed = Number.isFinite(stepProgress) && stepProgress > 1e-6;
                    if (progressed) successTicks += 1;
                    const waistViolation = computeWaistViolation();
                    const maxViolation = Math.max(
                        0,
                        Number.isFinite(waistViolation) ? waistViolation : 0,
                        Number.isFinite(Number(diagnostics.maxViolation)) ? Number(diagnostics.maxViolation) : 0,
                        Number.isFinite(Number(status.maxViolation)) ? Number(status.maxViolation) : 0
                    );
                    const step = {
                        tick,
                        runtimeMs,
                        u: Number(state.u),
                        vCenter: Number(state.vCenter),
                        stripSpacing: Number(state.stripSpacing),
                        normalOffset: Number(state.normalOffset),
                        uProgress,
                        stepProgress,
                        commandError: extractCommandError(status, diagnostics),
                        invariantError: extractInvariantError(status, diagnostics),
                        jointClearanceMin: computeJointClearanceMin(),
                        waistClearanceMin: computeWaistClearanceMin(diagnostics),
                        waistX: getWaistXPosition(),
                        maxViolation,
                        solver: typeof status.solver === 'string' ? status.solver : controller.solverMode,
                        reason: extractReason(status),
                        message: typeof status.message === 'string' ? status.message : '',
                        qpMessage: typeof status.qpMessage === 'string' && status.qpMessage.length > 0
                            ? status.qpMessage
                            : (typeof diagnostics.qpMessage === 'string' ? diagnostics.qpMessage : ''),
                        converged: status.converged !== false
                    };
                    steps.push(step);
                    if (!progressed) {
                        terminalReason = step.reason === 'unknown' ? 'stall' : step.reason;
                        break;
                    }
                }

                const finalStep = steps[steps.length - 1] || {};
                const completed = successTicks >= ticksRequested;
                if (completed) terminalReason = 'max_ticks';
                const summary = {
                    label: controller.label,
                    completed,
                    successTicks,
                    taskValidTicks: formatTaskValidTicks(steps),
                    uProgress: Number.isFinite(finalStep.uProgress) ? finalStep.uProgress : 0,
                    avgTaskDeviation: meanOf(steps.map((step) => step.commandError)),
                    avgInvariantDeviation: meanOf(steps.map((step) => step.invariantError)),
                    avgJointLimitClearance: meanOf(steps.map((step) => step.jointClearanceMin)),
                    avgWaistCorridorClearance: meanOf(steps.map((step) => step.waistClearanceMin)),
                    minJointLimitClearance: Math.min(...steps.map((step) => step.jointClearanceMin).filter(Number.isFinite)),
                    minWaistCorridorClearance: Math.min(...steps.map((step) => step.waistClearanceMin).filter(Number.isFinite)),
                    maxViolation: Math.max(0, ...steps.map((step) => step.maxViolation).filter(Number.isFinite)),
                    meanSolveMs: meanOf(steps.map((step) => step.runtimeMs)),
                    p95SolveMs: percentileOf(steps.map((step) => step.runtimeMs), 0.95),
                    terminalReason,
                    lastReason: finalStep.reason || 'unknown',
                    lastMessage: finalStep.message || '',
                    lastQpMessage: finalStep.qpMessage || null
                };
                runs.push({
                    summary,
                    run: {
                        solver: controller.solverMode,
                        label: controller.label,
                        source: 'cylindrical-patch',
                        ticksRequested,
                        capturedPose: start.capturedPose,
                        start: startState,
                        final: {
                            u: finalStep.u,
                            vCenter: finalStep.vCenter,
                            stripSpacing: finalStep.stripSpacing,
                            normalOffset: finalStep.normalOffset
                        },
                        startWaistX: start.waistX,
                        finalWaistX: finalStep.waistX,
                        finalQ: Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [],
                        appliedTicks: successTicks,
                        completed,
                        terminalReason,
                        steps
                    }
                });
                console.log(
                    `[surface] ${controller.label} completed=${completed ? 'yes' : 'no'} `
                    + `ticks=${successTicks}/${ticksRequested} u=${summary.uProgress.toFixed(4)} reason=${terminalReason}`
                );
            }
            return { runs };
        },
        {
            controllers: CONTROLLERS,
            ticksRequested: TICKS,
            speedPercent: SPEED_PERCENT,
            initialQ: INITIAL_Q,
            taskValidDeviationTolerance: TASK_VALID_DEVIATION_TOLERANCE
        }
    );

    const output = {
        meta: {
            generatedAt: new Date().toISOString(),
            scenario: 'Cooperative Surface Polishing (DQ Set-point, Waist Maintained)',
            ticksRequested: TICKS,
            speedPercent: SPEED_PERCENT,
            axis: 'x',
            direction: 'positive',
            surfaceSource: 'cylindrical-patch',
            capturedHands: true,
            waistXCorridor: { enabled: true, min: -0.5, max: 0.2 },
            initialQ: INITIAL_Q
        },
        runs: result.runs,
        browserErrors
    };
    await fs.mkdir(path.dirname(OUTPUT_JSON), { recursive: true });
    await fs.writeFile(OUTPUT_JSON, JSON.stringify(output, null, 2));
    await fs.mkdir(path.dirname(OUTPUT_CSV), { recursive: true });
    await fs.writeFile(OUTPUT_CSV, buildCsv(result.runs));

    for (const { summary } of result.runs) {
        console.log(
            `[done] ${summary.label} completed=${summary.completed ? 'yes' : 'no'} `
            + `ticks=${summary.successTicks}/${TICKS} u=${summary.uProgress.toFixed(4)}`
        );
    }
    console.log(`Saved JSON to ${OUTPUT_JSON}`);
    console.log(`Saved CSV to ${OUTPUT_CSV}`);
} finally {
    await browser.close();
    await stopServer(server);
}
