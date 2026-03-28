#!/usr/bin/env node

import { chromium } from 'playwright';

const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173';
const CASE_FILTER = process.env.CASE_FILTER || '';
const METRIC_REPEAT = Number.isFinite(Number(process.env.METRIC_REPEAT))
    ? Math.max(1, Math.floor(Number(process.env.METRIC_REPEAT)))
    : 1;
const SHOW_Q = process.env.SHOW_Q === '1';

const CASES = [
    {
        name: 'dual_arm_exact_limits',
        controlMode: 'absolute',
        q: [
            0.07183, 0.60355, -0.67552, 0,
            -0.5236, -0.1, 0, -1.0471, 0, -1.4, 1.65,
            0.5236, 0.1, 0, 1.0471, 0, 1.4, -1.65,
            0, 0
        ]
    },
    {
        name: 'dual_arm_near_limits',
        controlMode: 'absolute',
        q: [
            0.0678, 0.60862, -0.67656, 0,
            -0.52738, -0.09697, 0.00677, -1.04265, -0.00243, -1.39552, 1.64587,
            0.52738, 0.09697, -0.00677, 1.04265, 0.00243, 1.39552, -1.64587,
            0, 0
        ]
    },
    {
        name: 'full_body_upright_near_limits',
        controlMode: 'decoupled-upright',
        q: [
            0.10722, 0.86849, -0.96968, 0.00004,
            -0.65409, 0.19766, 0.41952, -1.18219, -0.36643, 0.29524, -0.64981,
            0.65415, -0.19763, -0.41951, 1.18209, 0.36642, -0.29513, 0.64981,
            0, 0
        ]
    }
];

const METRICS = [
    { key: 'w_abs_hier', method: 'optimizeHumanoidHierarchicalAbsoluteManipulability' },
    { key: 'w_abs_hier_vfi', method: 'optimizeHumanoidHierarchicalAbsoluteVFIManipulability' }
];

function fmt(value) {
    if (!Number.isFinite(value)) return 'n/a';
    const abs = Math.abs(value);
    if (abs === 0) return '0';
    if (abs >= 1000 || abs < 1e-4) return value.toExponential(3);
    return value.toFixed(6);
}

function formatLimitEntry(entry = null) {
    if (!entry) return 'n/a';
    return `${entry.name || 'n/a'} dmin=${fmt(entry.dMin)} q=${fmt(entry.q)}`;
}

const browser = await chromium.launch({
    headless: process.env.PLAYWRIGHT_HEADLESS !== '0',
    args: ['--use-angle=swiftshader', '--ignore-gpu-blocklist']
});

try {
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    page.on('console', (msg) => {
        if (msg.type() === 'error') {
            console.error('[page-error]', msg.text());
            return;
        }
        if (msg.text().includes('[compare]')) {
            console.log(`[page-${msg.type()}] ${msg.text()}`);
        }
    });
    page.on('dialog', async (dialog) => {
        console.log(`[dialog] ${dialog.message()}`);
        await dialog.dismiss();
    });

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

    const results = await page.evaluate(async ({ cases, metrics, caseFilter, metricRepeat }) => {
        const viz = window.visualizer;
        const DQ = window.DQModule?.DQWrapper;
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
            window.updateGlobalVariables?.();
            window.updateRobotPose?.();
            await sleep(120);
        };

        const waitForSolveActivity = async (qBefore, timeoutMs = 12000) => {
            const start = performance.now();
            while (performance.now() - start < timeoutMs) {
                const currentQ = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
                const qChanged = currentQ.some((value, index) => Math.abs((value || 0) - (qBefore?.[index] || 0)) > 1e-12);
                const statusActive = Boolean(viz.lastIKSolveStatus && Object.keys(viz.lastIKSolveStatus).length > 0);
                const diagMode = viz.lastHumanoidJogDiagnostics?.modeTag || '';
                if (qChanged || statusActive || diagMode !== 'idle') {
                    await sleep(80);
                    return { timedOut: false, qChanged, statusActive, diagMode };
                }
                await sleep(40);
            }
            return {
                timedOut: true,
                qChanged: false,
                statusActive: Boolean(viz.lastIKSolveStatus && Object.keys(viz.lastIKSolveStatus).length > 0),
                diagMode: viz.lastHumanoidJogDiagnostics?.modeTag || ''
            };
        };

        const summarizeNearestLimits = (qFull, includeWaist = false) => {
            const { armStart, activeCount } = viz._getHumanoidActiveSliceInfo(includeWaist);
            const names = viz.humanoidJointNames || [];
            const limits = viz._getHumanoidJointLimits?.() || [];
            const entries = [];
            for (let i = armStart; i < Math.min(armStart + activeCount, names.length, qFull.length, limits.length); i += 1) {
                const lim = limits[i];
                const q = Number(qFull[i]);
                const lower = Number(lim?.lower);
                const upper = Number(lim?.upper);
                const dLo = Number.isFinite(lower) ? q - lower : Number.NaN;
                const dUp = Number.isFinite(upper) ? upper - q : Number.NaN;
                const dMin = Math.min(
                    Number.isFinite(dLo) ? dLo : Number.POSITIVE_INFINITY,
                    Number.isFinite(dUp) ? dUp : Number.POSITIVE_INFINITY
                );
                entries.push({ index: i, name: names[i], q, lower, upper, dLo, dUp, dMin });
            }
            entries.sort((a, b) => a.dMin - b.dMin);
            return entries.slice(0, 6);
        };

        const absoluteX = (qFull) => {
            const absPose = viz._getHumanoidAbsolutePose(qFull);
            if (!absPose || !DQ) return Number.NaN;
            const translation = DQ.getTranslation(DQ.createFromArray(absPose));
            return Number(translation?.[1] || 0);
        };

        const snapshot = (qFull, refs, includeWaist, keepBodyZVertical) => {
            const { armStart, activeCount } = viz._getHumanoidActiveSliceInfo(includeWaist);
            const hardLimits = viz._getHumanoidJointLimits()?.slice(armStart, armStart + activeCount) || null;
            const cctsm = viz._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                viz._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits),
                { force: true }
            );
            const absPose = viz._getHumanoidAbsolutePose(qFull);
            const relPose = viz._getHumanoidRelativePose(qFull);
            const absErr = refs.abs && absPose
                ? viz._computePoseErrorNorm(viz._signedPoseError(absPose, refs.abs))
                : Number.NaN;
            const relErr = refs.rel && relPose
                ? viz._computePoseErrorNorm(viz._signedPoseError(relPose, refs.rel))
                : Number.NaN;
            const bodyState = keepBodyZVertical
                ? viz._getHumanoidBodyVerticalState(qFull, armStart, activeCount)
                : null;
            const bodyErr = keepBodyZVertical ? Number(bodyState?.errorNorm ?? Number.NaN) : Number.NaN;
            return {
                absX: absoluteX(qFull),
                nearestLimits: summarizeNearestLimits(qFull, includeWaist),
                cctsm,
                absErr,
                relErr,
                bodyErr,
                q: qFull.slice(),
                diag: { ...(viz.lastHumanoidJogDiagnostics || {}) },
                status: { ...(viz.lastIKSolveStatus || {}) }
            };
        };

        const runMetricCase = async (cfg, metricCfg) => {
            console.log(`[compare] start ${cfg.name} | ${metricCfg.key} | repeat=${metricRepeat}`);
            const includeWaist = cfg.controlMode === 'decoupled' || cfg.controlMode === 'decoupled-upright';
            const keepBodyZVertical = cfg.controlMode === 'decoupled-upright';
            dispatchSelectValue('dual-arm-control-mode-select', cfg.controlMode);
            dispatchSelectValue('humanoid-jog-solver-select', 'mj_restore');
            dispatchSelectValue('humanoid-manifold-jog-metric-select', 'none');
            setCheckbox('joint-limit-avoidance-toggle', true);
            setCheckbox('humanoid-waist-x-corridor-toggle', false);
            viz.setHumanoidJogSolverMode?.('mj_restore');
            viz.setHumanoidPredictiveJogEnabled?.(false);
            viz.setHumanoidVFITaskEquivalentBranching?.(false);
            if (viz.vfiConfig) {
                viz.vfiConfig.waistXCorridorEnabled = false;
                viz.vfiConfig.manifoldHierarchicalVfiBlend = 1;
            }

            await settleHumanoidState(cfg.q);

            const refs = {
                abs: viz._getHumanoidAbsolutePose(window.humanoidQ.slice()),
                rel: viz._getHumanoidRelativePose(window.humanoidQ.slice())
            };
            const before = snapshot(window.humanoidQ.slice(), refs, includeWaist, keepBodyZVertical);

            let metricTriggered = false;
            let metricActivity = null;
            for (let repeatIndex = 0; repeatIndex < metricRepeat; repeatIndex += 1) {
                viz.lastIKSolveStatus = {};
                viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };
                const qBeforeMetric = window.humanoidQ.slice();
                metricTriggered = viz[metricCfg.method]('increase') || metricTriggered;
                metricActivity = await waitForSolveActivity(qBeforeMetric);
                await sleep(40);
            }
            console.log(`[compare] metric done ${cfg.name} | ${metricCfg.key}`);
            const afterMetric = snapshot(window.humanoidQ.slice(), refs, includeWaist, keepBodyZVertical);
            afterMetric.triggered = metricTriggered;
            afterMetric.activity = metricActivity;
            afterMetric.repeatCount = metricRepeat;

            const metricStateAbsX = afterMetric.absX;
            viz.lastIKSolveStatus = {};
            viz.lastHumanoidJogDiagnostics = { modeTag: 'idle' };
            const qBeforeMove = window.humanoidQ.slice();
            const moveTriggered = viz.moveDualArmAbsolutePoseAlongAxis('x', 'negative');
            const moveActivity = await waitForSolveActivity(qBeforeMove);
            console.log(`[compare] move done ${cfg.name} | ${metricCfg.key}`);
            const afterMove = snapshot(window.humanoidQ.slice(), refs, includeWaist, keepBodyZVertical);
            afterMove.triggered = moveTriggered;
            afterMove.activity = moveActivity;
            afterMove.absXDeltaFromMetricState = afterMove.absX - metricStateAbsX;
            afterMove.absXDeltaFromInitial = afterMove.absX - before.absX;

            return {
                caseName: cfg.name,
                controlMode: cfg.controlMode,
                metric: metricCfg.key,
                includeWaist,
                keepBodyZVertical,
                before,
                afterMetric,
                afterMove
            };
        };

        const filteredCases = cases.filter((cfg) => !caseFilter || cfg.name.includes(caseFilter));
        const out = [];
        for (const cfg of filteredCases) {
            for (const metricCfg of metrics) {
                out.push(await runMetricCase(cfg, metricCfg));
            }
        }
        return out;
    }, { cases: CASES, metrics: METRICS, caseFilter: CASE_FILTER, metricRepeat: METRIC_REPEAT });

    for (const entry of results) {
        const topBefore = entry.before.nearestLimits[0] || null;
        const topAfter = entry.afterMetric.nearestLimits[0] || null;
        console.log(`\n=== ${entry.caseName} | ${entry.metric} ===`);
        console.log(`mode=${entry.controlMode} waist=${entry.includeWaist} body_z=${entry.keepBodyZVertical}`);
        console.log(`metric repeats      : ${entry.afterMetric.repeatCount || 1}`);
        console.log(
            `metric button: state=${entry.afterMetric.diag.jogMetricState || 'n/a'} applied=${entry.afterMetric.diag.jogMetricApplied} triggered=${entry.afterMetric.triggered}`
        );
        console.log(`closest limit before: ${formatLimitEntry(topBefore)}`);
        console.log(`closest limit after : ${formatLimitEntry(topAfter)}`);
        console.log(`dmin delta         : ${fmt((topAfter?.dMin ?? Number.NaN) - (topBefore?.dMin ?? Number.NaN))}`);
        console.log(`w_abs_hier         : ${fmt(entry.before.cctsm?.w_abs_hier)} -> ${fmt(entry.afterMetric.cctsm?.w_abs_hier)}`);
        console.log(`w_abs_hier_vfi     : ${fmt(entry.before.cctsm?.w_abs_hier_vfi)} -> ${fmt(entry.afterMetric.cctsm?.w_abs_hier_vfi)}`);
        console.log(`w_abs_arm_vfi      : ${fmt(entry.before.cctsm?.w_abs_arm_vfi)} -> ${fmt(entry.afterMetric.cctsm?.w_abs_arm_vfi)}`);
        console.log(`w_vfi              : ${fmt(entry.before.cctsm?.w_vfi)} -> ${fmt(entry.afterMetric.cctsm?.w_vfi)}`);
        console.log(`pose drift abs/rel/body: ${fmt(entry.afterMetric.absErr)} / ${fmt(entry.afterMetric.relErr)} / ${fmt(entry.afterMetric.bodyErr)}`);
        console.log(`x- delta from metric state: ${fmt(entry.afterMove.absXDeltaFromMetricState)}`);
        console.log(`x- delta from initial     : ${fmt(entry.afterMove.absXDeltaFromInitial)}`);
        console.log(`x- residual rel/body      : ${fmt(entry.afterMove.diag?.relativeResidualNorm)} / ${fmt(entry.afterMove.diag?.bodyResidualNorm)}`);
        console.log(`x- status                 : ${entry.afterMove.status?.message || entry.afterMove.diag?.message || 'n/a'}`);
        if (SHOW_Q) {
            console.log(`q_after_metric           : ${JSON.stringify(entry.afterMetric.q)}`);
        }
    }
} finally {
    await browser.close();
}
