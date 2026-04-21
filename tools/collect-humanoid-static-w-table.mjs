#!/usr/bin/env node

import fs from 'node:fs/promises';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';

import { chromium } from 'playwright';

import { formatBenchmarkMarkdownTable } from '../src/core/humanoid-jog-benchmark-metrics.js';
import {
    buildHumanoidStaticManipTasks,
    summarizeHumanoidStaticManipRow
} from '../src/core/humanoid-jog-static-w-table.js';

const REPO_ROOT = '/home/echoz/dq_vizlab';
const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173';
const REPEAT_COUNT = Number.isFinite(Number(process.env.METRIC_REPEAT))
    ? Math.max(1, Math.floor(Number(process.env.METRIC_REPEAT)))
    : 200;
const METRIC_ACTIVITY_TIMEOUT_MS = Number.isFinite(Number(process.env.METRIC_ACTIVITY_TIMEOUT_MS))
    ? Math.max(100, Math.floor(Number(process.env.METRIC_ACTIVITY_TIMEOUT_MS)))
    : 160;
const FULL_BODY_ABSOLUTE_MODE = process.env.FULL_BODY_ABSOLUTE_MODE || 'decoupled';
const TASK_FILTER = process.env.TASK_FILTER || '';
const STRUCTURE_FILTER = process.env.STRUCTURE_FILTER || '';
const METRIC_FILTER = process.env.METRIC_FILTER || '';
const LOCK_STATE_FILTER = process.env.LOCK_STATE_FILTER || '';
const SOLVER_MODE = process.env.STATIC_SOLVER_MODE || 'mj_restore';
const LOCKED_ONLY = ['1', 'true', 'yes', 'on'].includes(String(process.env.STATIC_LOCKED_ONLY || '').toLowerCase());
const OUTPUT_JSON = process.env.STATIC_TABLE_JSON || path.join(REPO_ROOT, '2026_humanoid_robot', 'figure', 'sim', 'static_manip_table_rows_joint2lock.json');
const OUTPUT_MD = process.env.STATIC_TABLE_MD || path.join(REPO_ROOT, '2026_humanoid_robot', 'figure', 'sim', 'static_manip_table_rows_joint2lock.md');
const INPUT_Q_JSON = process.env.STATIC_HUMANOID_Q_JSON || '';
const INPUT_DUAL_ARM_Q_JSON = process.env.STATIC_DUAL_ARM_Q_JSON || '';
const INPUT_FULL_BODY_Q_JSON = process.env.STATIC_FULL_BODY_Q_JSON || '';

function parseHumanoidQEnv(raw) {
    if (typeof raw !== 'string' || raw.trim().length === 0) return null;
    try {
        const parsed = JSON.parse(raw);
        if (!Array.isArray(parsed) || parsed.length === 0) return null;
        const numeric = parsed.map((value) => Number(value));
        return numeric.every((value) => Number.isFinite(value)) ? numeric : null;
    } catch (_) {
        return null;
    }
}

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

const browser = await chromium.launch({
    headless: process.env.PLAYWRIGHT_HEADLESS !== '0',
    args: ['--use-angle=swiftshader', '--ignore-gpu-blocklist']
});

let server = null;
try {
    server = await startServerIfNeeded();
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    page.on('console', (msg) => {
        if (msg.type() === 'error') {
            console.error('[page-error]', msg.text());
            return;
        }
        if (msg.text().includes('[static-w-table]')) {
            console.log(`[page-${msg.type()}] ${msg.text()}`);
        }
    });
    await ensureHumanoidPageReady(page);

    const tasks = buildHumanoidStaticManipTasks({
        fullBodyAbsoluteMode: FULL_BODY_ABSOLUTE_MODE,
        lockedOnly: LOCKED_ONLY
    }).filter((task) => {
        if (TASK_FILTER && !(task.controlMode.includes(TASK_FILTER) || task.taskKind.includes(TASK_FILTER))) {
            return false;
        }
        if (STRUCTURE_FILTER && task.structure !== STRUCTURE_FILTER) {
            return false;
        }
        if (METRIC_FILTER && task.metricKey !== METRIC_FILTER) {
            return false;
        }
        if (LOCK_STATE_FILTER) {
            const normalized = String(LOCK_STATE_FILTER).trim().toLowerCase();
            const wantLocked = ['1', 'true', 'yes', 'on', 'locked'].includes(normalized);
            const wantUnlocked = ['0', 'false', 'no', 'off', 'unlocked'].includes(normalized);
            if (wantLocked && task.locked !== true) return false;
            if (wantUnlocked && task.locked !== false) return false;
        }
        return true;
    });

    const baseQEnv = parseHumanoidQEnv(INPUT_Q_JSON);
    const dualArmQEnv = parseHumanoidQEnv(INPUT_DUAL_ARM_Q_JSON);
    const fullBodyQEnv = parseHumanoidQEnv(INPUT_FULL_BODY_Q_JSON);
    const result = await page.evaluate(
        async ({ tasks: tasksInput, repeatCount, solverMode, baseQOverride, dualArmQOverride, fullBodyQOverride, metricActivityTimeoutMs }) => {
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

            const getBaseQ = (task) => {
                if (task?.structure === 'dual-arm' && Array.isArray(dualArmQOverride) && dualArmQOverride.length > 0) {
                    return dualArmQOverride.slice();
                }
                if (task?.structure === 'full-body' && Array.isArray(fullBodyQOverride) && fullBodyQOverride.length > 0) {
                    return fullBodyQOverride.slice();
                }
                if (Array.isArray(task?.defaultBaseQ) && task.defaultBaseQ.length > 0) {
                    return task.defaultBaseQ.slice();
                }
                if (Array.isArray(baseQOverride) && baseQOverride.length > 0) {
                    return baseQOverride.slice();
                }
                if (Array.isArray(window.customRobotConfig?.initial_q) && window.customRobotConfig.initial_q.length > 0) {
                    return window.customRobotConfig.initial_q.map((value) => Number(value));
                }
                return Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
            };

            const settleHumanoidState = async (qFull) => {
                for (let i = 0; i < Math.min(window.humanoidQ.length, qFull.length); i += 1) {
                    window.humanoidQ[i] = qFull[i];
                }
                viz.setHumanoidJoints(window.humanoidQ);
                window.updateGlobalVariables?.();
                window.updateRobotPose?.();
                await sleep(120);
            };

            const snapshot = (controlMode, includeWaist) => {
                const qFull = window.humanoidQ.slice();
                const { armStart, activeCount } = viz._getHumanoidActiveSliceInfo(includeWaist);
                const hardLimits = viz._getHumanoidJointLimits?.()?.slice(armStart, armStart + activeCount) || null;
                const constraints = viz._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits);
                const cctsm = viz._computeModeCCTSM(qFull, armStart, activeCount, controlMode, constraints, { force: true });
                const dmin = Array.isArray(constraints) && constraints.length > 0
                    ? constraints.reduce((min, item) => {
                        const distance = Number(item?.distance);
                        return Number.isFinite(distance) ? Math.min(min, distance) : min;
                    }, Number.POSITIVE_INFINITY)
                    : Number.NaN;
                const finiteDmin = Number.isFinite(dmin) ? dmin : Number.NaN;
                return {
                    q: qFull,
                    wHier: viz._evaluateHumanoidJogManifoldMetricValue('w_abs_hier', cctsm, ''),
                    wHierVfi: viz._evaluateHumanoidJogManifoldMetricValue('w_abs_hier_vfi', cctsm, ''),
                    dmin: finiteDmin,
                    cctsm
                };
            };

            dispatchSelectValue('humanoid-jog-solver-select', solverMode);
            setCheckbox('joint-limit-avoidance-toggle', true);

            const rows = [];
            console.log(`[static-w-table] base_q_dual_length=${getBaseQ({ structure: 'dual-arm' }).length}`);
            console.log(`[static-w-table] base_q_full_length=${getBaseQ({ structure: 'full-body' }).length}`);

            for (const task of tasksInput) {
                dispatchSelectValue('dual-arm-control-mode-select', task.controlMode);
                setCheckbox('humanoid-test-dual-arm-lock-toggle', Boolean(task.locked));
                dispatchSelectValue('humanoid-manifold-jog-metric-select', task.metricKey);
                const taskBaseQ = getBaseQ(task);
                await settleHumanoidState(taskBaseQ);
                await sleep(80);

                const before = snapshot(task.controlMode, task.includeWaist);

                for (let repeatIndex = 0; repeatIndex < repeatCount; repeatIndex += 1) {
                    viz.lastIKSolveStatus = {};
                    viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };
                    const qBefore = window.humanoidQ.slice();
                    if (task.metricKey === 'w_abs_hier_vfi') {
                        viz.optimizeHumanoidHierarchicalAbsoluteVFIManipulability('increase');
                    } else {
                        viz.optimizeHumanoidHierarchicalAbsoluteManipulability('increase');
                    }
                    await waitForSolveActivity(qBefore, metricActivityTimeoutMs);
                    await sleep(40);
                    if ((repeatIndex + 1) % 50 === 0 || repeatIndex + 1 === repeatCount) {
                        console.log(`[static-w-table] ${task.taskKind} | ${task.structure} | ${task.locked ? 'locked | ' : ''}${task.metricKey} step=${repeatIndex + 1}/${repeatCount}`);
                    }
                }

                const after = snapshot(task.controlMode, task.includeWaist);
                rows.push({
                    task,
                    baseQ: taskBaseQ,
                    before,
                    after
                });
                console.log(`[static-w-table] done ${task.taskKind} | ${task.structure} | ${task.locked ? 'locked | ' : ''}${task.metricKey}`);
            }

            return {
                baseQ: Array.isArray(baseQOverride) ? baseQOverride : null,
                dualArmBaseQ: Array.isArray(dualArmQOverride) ? dualArmQOverride : null,
                fullBodyBaseQ: Array.isArray(fullBodyQOverride) ? fullBodyQOverride : null,
                repeatCount,
                solverMode,
                rows
            };
        },
        {
            tasks,
            repeatCount: REPEAT_COUNT,
            solverMode: SOLVER_MODE,
            baseQOverride: baseQEnv,
            dualArmQOverride: dualArmQEnv,
            fullBodyQOverride: fullBodyQEnv,
            metricActivityTimeoutMs: METRIC_ACTIVITY_TIMEOUT_MS
        }
    );

    const summarizedRows = result.rows.map((entry) => summarizeHumanoidStaticManipRow(entry));
    const markdown = formatBenchmarkMarkdownTable(summarizedRows, {
        columns: [
            { key: 'taskDescriptor', label: '具体任务' },
            { key: 'w_hier_before', label: 'w_hier_before', digits: 6 },
            { key: 'w_hier_after', label: 'w_hier_after', digits: 6 },
            { key: 'delta_w_hier', label: 'Δw_hier', digits: 6 },
            { key: 'w_hier_vfi_before', label: 'w_hier_vfi_before', digits: 6 },
            { key: 'w_hier_vfi_after', label: 'w_hier_vfi_after', digits: 6 },
            { key: 'delta_w_hier_vfi', label: 'Δw_hier_vfi', digits: 6 },
            { key: 'dmin_before', label: 'dmin_before', digits: 6 },
            { key: 'dmin_after', label: 'dmin_after', digits: 6 }
        ]
    });

    await fs.mkdir(path.dirname(OUTPUT_JSON), { recursive: true });
    await fs.mkdir(path.dirname(OUTPUT_MD), { recursive: true });
    await fs.writeFile(
        OUTPUT_JSON,
        JSON.stringify({
            generatedAt: new Date().toISOString(),
            baseUrl: BASE_URL,
            solverMode: SOLVER_MODE,
            fullBodyAbsoluteMode: FULL_BODY_ABSOLUTE_MODE,
            repeatCount: REPEAT_COUNT,
            lockedOnly: LOCKED_ONLY,
            baseQ: result.baseQ,
            dualArmBaseQ: result.dualArmBaseQ,
            fullBodyBaseQ: result.fullBodyBaseQ,
            rows: summarizedRows,
            rawRows: result.rows
        }, null, 2),
        'utf8'
    );
    await fs.writeFile(OUTPUT_MD, markdown + '\n', 'utf8');

    console.log(`\nSaved JSON: ${OUTPUT_JSON}`);
    console.log(`Saved Markdown: ${OUTPUT_MD}\n`);
    console.log(markdown);
} finally {
    await browser.close();
    await stopServer(server);
}
