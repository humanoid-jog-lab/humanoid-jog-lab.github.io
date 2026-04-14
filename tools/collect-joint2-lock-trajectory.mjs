#!/usr/bin/env node

import fs from 'node:fs/promises';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';

import { chromium } from 'playwright';

import {
    JOINT2_LOCK_TRAJECTORY_CASE,
    buildJoint2LockTrajectoryPlotPayload
} from '../src/core/humanoid-joint2-lock-trajectory.js';

const REPO_ROOT = '/home/echoz/dq_vizlab';
const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173';
const OUTPUT_JSON = process.env.JOINT2_LOCK_TRAJECTORY_JSON
    || path.join(REPO_ROOT, '2026_humanoid_robot', 'figure', 'sim', 'joint2_lock_metric_trajectory.json');
const REPEAT_COUNT = Number.isFinite(Number(process.env.METRIC_REPEAT))
    ? Math.max(1, Math.floor(Number(process.env.METRIC_REPEAT)))
    : JOINT2_LOCK_TRAJECTORY_CASE.repeatCountDefault;
const METRIC_KEYS = String(process.env.METRIC_KEYS || '')
    .split(',')
    .map((entry) => entry.trim())
    .filter(Boolean);
const METRIC_ACTIVITY_TIMEOUT_MS = Number.isFinite(Number(process.env.METRIC_ACTIVITY_TIMEOUT_MS))
    ? Math.max(100, Math.floor(Number(process.env.METRIC_ACTIVITY_TIMEOUT_MS)))
    : 160;

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
    if (!signalGroup('SIGTERM')) {
        child.kill('SIGTERM');
    }
    const exited = await Promise.race([
        exitPromise.then(() => true),
        delay(1500).then(() => false)
    ]);
    if (!exited) {
        if (!signalGroup('SIGKILL')) {
            child.kill('SIGKILL');
        }
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

let server = null;
const browser = await chromium.launch({
    headless: process.env.PLAYWRIGHT_HEADLESS !== '0',
    args: ['--use-angle=swiftshader', '--ignore-gpu-blocklist']
});

try {
    server = await startServerIfNeeded();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    page.on('console', (msg) => {
        if (msg.type() === 'error') {
            console.error('[page-error]', msg.text());
            return;
        }
        if (msg.text().includes('[joint2-lock]')) {
            console.log(`[page-${msg.type()}] ${msg.text()}`);
        }
    });

    await ensureHumanoidPageReady(page);

    const result = await page.evaluate(
        async ({ config, repeatCount, metricKeys, metricActivityTimeoutMs }) => {
            const viz = window.visualizer;
            const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

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

            const settleHumanoidState = async (qFull) => {
                for (let i = 0; i < Math.min(window.humanoidQ.length, qFull.length); i += 1) {
                    window.humanoidQ[i] = qFull[i];
                }
                viz.setHumanoidJoints(window.humanoidQ);
                window.updateRobotPose?.();
                await sleep(120);
            };

            const waitForSolveActivity = async (qBefore, settleMs = 160) => {
                await sleep(settleMs);
                const currentQ = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
                const qChanged = currentQ.some((value, index) => Math.abs((value || 0) - (qBefore?.[index] || 0)) > 1e-12);
                const statusActive = Boolean(viz.lastIKSolveStatus && Object.keys(viz.lastIKSolveStatus).length > 0);
                const diagMode = viz.lastHumanoidJogDiagnostics?.modeTag || '';
                return {
                    timedOut: false,
                    qChanged,
                    statusActive,
                    diagMode
                };
            };

            const captureSample = (step, metricKey) => {
                const q = window.humanoidQ.slice();
                const { armStart, activeCount } = viz._getHumanoidActiveSliceInfo(config.includeWaist);
                const hardLimits = viz._getHumanoidJointLimits()?.slice(armStart, armStart + activeCount) || null;
                const vfiConstraints = viz._buildManipulabilityVFIConstraints(q, armStart, activeCount, hardLimits);
                const cctsm = viz._computeModeCCTSM(
                    q,
                    armStart,
                    activeCount,
                    'absolute',
                    vfiConstraints,
                    { force: true }
                );
                const nearestLimits = (viz._getHumanoidJointLimits?.() || []).map((limit, index) => {
                    const value = Number(q[index]);
                    const lower = Number(limit?.lower);
                    const upper = Number(limit?.upper);
                    return {
                        index,
                        q: value,
                        lower,
                        upper,
                        dMin: Math.min(
                            Number.isFinite(lower) ? value - lower : Number.POSITIVE_INFINITY,
                            Number.isFinite(upper) ? upper - value : Number.POSITIVE_INFINITY
                        )
                    };
                });
                return {
                    step,
                    metricKey,
                    q,
                    cctsm: {
                        w_abs_hier: Number(cctsm?.w_abs_hier ?? Number.NaN),
                        w_abs_hier_vfi: Number(cctsm?.w_abs_hier_vfi ?? Number.NaN),
                        w_abs_arm: Number(cctsm?.w_abs_arm ?? Number.NaN),
                        w_abs_arm_vfi: Number(cctsm?.w_abs_arm_vfi ?? Number.NaN)
                    },
                    dmin: nearestLimits.reduce((best, entry) => Math.min(best, Number(entry?.dMin ?? Number.POSITIVE_INFINITY)), Number.POSITIVE_INFINITY),
                    diagnostics: { ...(viz.lastHumanoidJogDiagnostics || {}) },
                    status: { ...(viz.lastIKSolveStatus || {}) }
                };
            };

            dispatchSelectValue('dual-arm-control-mode-select', config.controlMode);
            dispatchSelectValue('humanoid-jog-solver-select', config.solverMode);
            dispatchSelectValue('humanoid-manifold-jog-metric-select', 'none');
            setCheckbox('joint-limit-avoidance-toggle', true);
            setCheckbox('humanoid-waist-x-corridor-toggle', false);
            setCheckbox('humanoid-test-dual-arm-lock-toggle', true);
            viz.setHumanoidJogSolverMode?.(config.solverMode);
            viz.setHumanoidPredictiveJogEnabled?.(false);
            viz.setHumanoidVFITaskEquivalentBranching?.(false);
            viz.setHumanoidWaistXCorridorEnabled?.(false);
            viz.setHumanoidTestDualArmLockEnabled?.(true);
            if (viz.vfiConfig) {
                viz.vfiConfig.waistXCorridorEnabled = false;
                viz.vfiConfig.humanoidTestDualArmLockEnabled = true;
            }

            const selectedMetrics = Array.isArray(config.metrics)
                ? config.metrics.filter((metric) => !metricKeys.length || metricKeys.includes(metric.key))
                : [];
            const samplesByMetric = {};
            for (const metric of selectedMetrics) {
                await settleHumanoidState(config.baseQ);
                viz.lastIKSolveStatus = {};
                viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };
                const samples = [captureSample(0, metric.key)];
                console.log(`[joint2-lock] start ${metric.key} repeat=${repeatCount}`);
                for (let step = 1; step <= repeatCount; step += 1) {
                    const qBefore = window.humanoidQ.slice();
                    const triggered = viz[metric.method]('increase');
                    const activity = await waitForSolveActivity(qBefore, metricActivityTimeoutMs);
                    const sample = captureSample(step, metric.key);
                    sample.triggered = Boolean(triggered);
                    sample.activity = activity;
                    samples.push(sample);
                    if (step % 50 === 0 || step === repeatCount) {
                        console.log(`[joint2-lock] ${metric.key} step=${step}/${repeatCount}`);
                    }
                }
                samplesByMetric[metric.key] = samples;
                console.log(`[joint2-lock] done ${metric.key}`);
            }
            return samplesByMetric;
        },
        {
            config: JOINT2_LOCK_TRAJECTORY_CASE,
            repeatCount: REPEAT_COUNT,
            metricKeys: METRIC_KEYS,
            metricActivityTimeoutMs: METRIC_ACTIVITY_TIMEOUT_MS
        }
    );

    const payload = {
        generatedAt: new Date().toISOString(),
        baseUrl: BASE_URL,
        repeatCount: REPEAT_COUNT,
        case: JOINT2_LOCK_TRAJECTORY_CASE,
        rawSamples: result,
        plot: buildJoint2LockTrajectoryPlotPayload({
            repeatCount: REPEAT_COUNT,
            samplesByMetric: result
        })
    };

    await fs.mkdir(path.dirname(OUTPUT_JSON), { recursive: true });
    await fs.writeFile(OUTPUT_JSON, `${JSON.stringify(payload, null, 2)}\n`, 'utf8');
    console.log(`Saved JSON to ${OUTPUT_JSON}`);
} finally {
    await browser.close();
    await stopServer(server);
}
