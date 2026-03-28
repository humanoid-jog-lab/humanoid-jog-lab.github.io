#!/usr/bin/env node

import { chromium } from 'playwright';

const BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173';
const TARGET_Q = [
    -0.3, 0.9, -0.6, 0,
    -1.09922, 0.39517, 1.70768, -1.14947, -1.75016, 0.69436, -0.64843,
    0.79659, -0.37967, -1.13817, 1.12953, 1.07757, -0.62949, 0.28969,
    0, 0
];

function printSection(title, value) {
    console.log(`\n[${title}]`);
    console.log(JSON.stringify(value, null, 2));
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
        }
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

    const result = await page.evaluate(async (targetQ) => {
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

        const sliceRows = (rows, start, count, rowCount = rows.length) => {
            const out = [];
            for (let i = 0; i < rowCount; i += 1) {
                const row = Array.isArray(rows?.[i]) ? rows[i].slice(start, start + count) : null;
                if (!row) return null;
                out.push(row);
            }
            return out;
        };

        const summarizeNearestLimits = (qFull) => {
            const names = viz.humanoidJointNames || [];
            const limits = viz._getHumanoidJointLimits?.() || [];
            const entries = [];
            for (let i = 0; i < Math.min(names.length, qFull.length, limits.length); i += 1) {
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
            return entries.slice(0, 8);
        };

        const buildMetricDebug = (qFull) => {
            const { armStart, activeCount } = viz._getHumanoidActiveSliceInfo(false);
            const jointLimits = viz._getHumanoidJointLimits();
            const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
            const JabsFull = viz._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = viz._getHumanoidRelativePoseJacobian(qFull);
            const Jabs = sliceRows(JabsFull, armStart, activeCount, 8);
            const Jrel = sliceRows(JrelFull, armStart, activeCount, 8);
            const protectedRows = Array.isArray(Jrel) ? Jrel.map((row) => row.slice()) : [];
            const vfiConstraints = viz._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits);
            const cctsm = viz._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                vfiConstraints,
                { force: true }
            );
            const metricGradient = viz._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'w_abs_hier_vfi'
            });
            const wvfiGradient = viz._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'w_vfi'
            });
            return {
                cctsm,
                vfiConstraintCount: Array.isArray(vfiConstraints) ? vfiConstraints.length : 0,
                vfiConstraints,
                metricGradient,
                metricGradientNorm: Array.isArray(metricGradient)
                    ? Math.sqrt(metricGradient.reduce((sum, value) => sum + (value || 0) * (value || 0), 0))
                    : Number.NaN,
                wvfiGradient,
                wvfiGradientNorm: Array.isArray(wvfiGradient)
                    ? Math.sqrt(wvfiGradient.reduce((sum, value) => sum + (value || 0) * (value || 0), 0))
                    : Number.NaN
            };
        };

        const settleHumanoidState = async (qFull) => {
            for (let i = 0; i < Math.min(window.humanoidQ.length, qFull.length); i += 1) {
                window.humanoidQ[i] = qFull[i];
            }
            viz.setHumanoidJoints(window.humanoidQ);
            window.updateGlobalVariables?.();
            window.updateRobotPose?.();
            await sleep(100);
        };

        const waitForSolveActivity = async (qBefore, timeoutMs = 5000) => {
            const start = performance.now();
            while (performance.now() - start < timeoutMs) {
                const currentQ = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
                const qChanged = currentQ.some((value, index) => Math.abs((value || 0) - (qBefore?.[index] || 0)) > 1e-12);
                const statusActive = Boolean(viz.lastIKSolveStatus && Object.keys(viz.lastIKSolveStatus).length > 0);
                const diagMode = viz.lastHumanoidJogDiagnostics?.modeTag || '';
                if (qChanged || statusActive || diagMode !== 'idle') {
                    return {
                        timedOut: false,
                        qChanged,
                        statusActive,
                        diagMode
                    };
                }
                await sleep(50);
            }
            return {
                timedOut: true,
                qChanged: false,
                statusActive: Boolean(viz.lastIKSolveStatus && Object.keys(viz.lastIKSolveStatus).length > 0),
                diagMode: viz.lastHumanoidJogDiagnostics?.modeTag || ''
            };
        };

        dispatchSelectValue('dual-arm-control-mode-select', 'absolute');
        dispatchSelectValue('humanoid-jog-solver-select', 'mj_restore');
        dispatchSelectValue('humanoid-manifold-jog-metric-select', 'none');
        setCheckbox('joint-limit-avoidance-toggle', true);
        viz.setHumanoidJogSolverMode?.('mj_restore');
        viz.setHumanoidPredictiveJogEnabled?.(false);
        viz.setHumanoidVFITaskEquivalentBranching?.(false);

        await settleHumanoidState(targetQ);

        const beforeQ = window.humanoidQ.slice();
        const beforeAbs = viz._getHumanoidAbsolutePose(beforeQ);
        const beforeRel = viz._getHumanoidRelativePose(beforeQ);
        const beforeDebug = buildMetricDebug(beforeQ);
        viz.lastIKSolveStatus = {};

        const metricTriggered = viz.optimizeHumanoidHierarchicalAbsoluteVFIManipulability('increase');
        const metricActivity = await waitForSolveActivity(beforeQ);

        const afterMetricQ = window.humanoidQ.slice();
        const metricDelta = afterMetricQ.map((value, index) => value - beforeQ[index]);
        const afterMetricDebug = buildMetricDebug(afterMetricQ);
        const afterMetricStatus = { ...viz.lastIKSolveStatus };
        const afterMetricDiagnostics = { ...viz.lastHumanoidJogDiagnostics };

        const moveStartQ = window.humanoidQ.slice();
        viz.lastIKSolveStatus = {};
        const moveTriggered = viz.moveDualArmAbsolutePoseAlongAxis('x', 'negative');
        const moveActivity = await waitForSolveActivity(moveStartQ);

        const afterMoveQ = window.humanoidQ.slice();
        const moveDelta = afterMoveQ.map((value, index) => value - moveStartQ[index]);
        const afterMoveDebug = buildMetricDebug(afterMoveQ);
        const afterMoveStatus = { ...viz.lastIKSolveStatus };
        const afterMoveDiagnostics = { ...viz.lastHumanoidJogDiagnostics };

        return {
            before: {
                nearestLimits: summarizeNearestLimits(beforeQ),
                vizHumanoidMode: viz.humanoidMode,
                globalHumanoidMode: window.humanoidMode,
                hasHumanoidModel: Boolean(window.humanoidModel),
                hasDualArmSystem: Boolean(window.dualArmSystem),
                cctsm: beforeDebug.cctsm,
                metricGradientNorm: beforeDebug.metricGradientNorm,
                wvfiGradientNorm: beforeDebug.wvfiGradientNorm,
                metricGradient: beforeDebug.metricGradient,
                wvfiGradient: beforeDebug.wvfiGradient,
                vfiConstraintCount: beforeDebug.vfiConstraintCount,
                vfiConstraints: beforeDebug.vfiConstraints
            },
            afterMetric: {
                nearestLimits: summarizeNearestLimits(afterMetricQ),
                triggered: metricTriggered,
                activity: metricActivity,
                delta: metricDelta,
                cctsm: afterMetricDebug.cctsm,
                metricGradientNorm: afterMetricDebug.metricGradientNorm,
                wvfiGradientNorm: afterMetricDebug.wvfiGradientNorm,
                metricGradient: afterMetricDebug.metricGradient,
                wvfiGradient: afterMetricDebug.wvfiGradient,
                vfiConstraintCount: afterMetricDebug.vfiConstraintCount,
                status: afterMetricStatus,
                diagnostics: afterMetricDiagnostics
            },
            afterMove: {
                nearestLimits: summarizeNearestLimits(afterMoveQ),
                triggered: moveTriggered,
                activity: moveActivity,
                delta: moveDelta,
                cctsm: afterMoveDebug.cctsm,
                metricGradientNorm: afterMoveDebug.metricGradientNorm,
                wvfiGradientNorm: afterMoveDebug.wvfiGradientNorm,
                vfiConstraintCount: afterMoveDebug.vfiConstraintCount,
                status: afterMoveStatus,
                diagnostics: afterMoveDiagnostics
            }
        };
    }, TARGET_Q);

    printSection('before', result.before);
    printSection('afterMetric', result.afterMetric);
    printSection('afterMove', result.afterMove);
} finally {
    await browser.close();
}
