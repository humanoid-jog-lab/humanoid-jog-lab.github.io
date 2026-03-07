const JOINT_MONITOR_CHANNEL_NAME = 'dq-vizlab-joint-monitor-v1';
const JOINT_MONITOR_HISTORY_LIMIT = 260;
const JOINT_MONITOR_STALE_WARN_MS = 1500;
const JOINT_MONITOR_STALE_RESET_MS = 10000;
const JOINT_MONITOR_COLORS = [
    '#22d3ee',
    '#60a5fa',
    '#34d399',
    '#f59e0b',
    '#f472b6',
    '#a78bfa',
    '#f87171',
    '#facc15',
];
const JOINT_MONITOR_GROUP_KEYS = ['waist', 'left', 'right', 'head'];
const JOINT_MONITOR_GROUP_LABELS = {
    waist: '腰部',
    left: '左臂',
    right: '右臂',
    head: '头部',
};

const monitorEls = {
    state: document.getElementById('joint-monitor-window-state'),
    updated: document.getElementById('joint-monitor-window-updated'),
    groups: {
        waist: {
            canvas: document.getElementById('joint-monitor-window-waist-canvas'),
            meta: document.getElementById('joint-monitor-window-waist-meta'),
            values: document.getElementById('joint-monitor-window-waist-values'),
        },
        left: {
            canvas: document.getElementById('joint-monitor-window-left-canvas'),
            meta: document.getElementById('joint-monitor-window-left-meta'),
            values: document.getElementById('joint-monitor-window-left-values'),
        },
        right: {
            canvas: document.getElementById('joint-monitor-window-right-canvas'),
            meta: document.getElementById('joint-monitor-window-right-meta'),
            values: document.getElementById('joint-monitor-window-right-values'),
        },
        head: {
            canvas: document.getElementById('joint-monitor-window-head-canvas'),
            meta: document.getElementById('joint-monitor-window-head-meta'),
            values: document.getElementById('joint-monitor-window-head-values'),
        },
    },
};

let monitorChannel = null;
let monitorHistory = [];
let monitorQDim = 0;
let monitorLastSnapshotPerfTs = 0;
let monitorLastSnapshotRealTs = 0;
let monitorLastStateText = '';

function setMonitorState(text, stale = false) {
    if (!monitorEls.state) return;
    const value = String(text || '等待主页面数据');
    if (value === monitorLastStateText && monitorEls.state.classList.contains('stale') === stale) {
        return;
    }
    monitorEls.state.textContent = value;
    monitorEls.state.classList.toggle('stale', stale);
    monitorLastStateText = value;
}

function setMonitorUpdated(tsMs = 0) {
    if (!monitorEls.updated) return;
    if (!Number.isFinite(tsMs) || tsMs <= 0) {
        monitorEls.updated.textContent = '最后更新: -';
        return;
    }
    const dt = new Date(tsMs);
    monitorEls.updated.textContent = `最后更新: ${dt.toLocaleTimeString()}`;
}

function prepareCanvas(canvas) {
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const cssWidth = Math.max(1, Math.floor(rect.width || canvas.clientWidth || 1));
    const cssHeight = Math.max(1, Math.floor(rect.height || canvas.clientHeight || 1));
    const dpr = Math.max(1, Math.min(2, Number(window.devicePixelRatio) || 1));
    const widthPx = Math.max(1, Math.floor(cssWidth * dpr));
    const heightPx = Math.max(1, Math.floor(cssHeight * dpr));
    if (canvas.width !== widthPx || canvas.height !== heightPx) {
        canvas.width = widthPx;
        canvas.height = heightPx;
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.scale(dpr, dpr);
    return { ctx, width: cssWidth, height: cssHeight };
}

function drawJointChart(canvas, history, indices) {
    const view = prepareCanvas(canvas);
    if (!view) return { minDeg: null, maxDeg: null };
    const { ctx, width, height } = view;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = 'rgba(2, 6, 23, 0.92)';
    ctx.fillRect(0, 0, width, height);

    if (!Array.isArray(indices) || indices.length === 0) {
        ctx.fillStyle = 'rgba(148, 163, 184, 0.9)';
        ctx.font = '12px monospace';
        ctx.fillText('无关节', 10, Math.max(18, Math.round(height / 2)));
        return { minDeg: null, maxDeg: null };
    }
    if (!Array.isArray(history) || history.length === 0) {
        ctx.fillStyle = 'rgba(148, 163, 184, 0.9)';
        ctx.font = '12px monospace';
        ctx.fillText('等待数据', 10, Math.max(18, Math.round(height / 2)));
        return { minDeg: null, maxDeg: null };
    }

    const sampleCount = history.length;
    const series = indices.map(() => new Array(sampleCount));
    let minDeg = Number.POSITIVE_INFINITY;
    let maxDeg = Number.NEGATIVE_INFINITY;

    for (let s = 0; s < sampleCount; s++) {
        const qSample = Array.isArray(history[s]?.q) ? history[s].q : [];
        for (let j = 0; j < indices.length; j++) {
            const idx = indices[j];
            const deg = ((Number(qSample[idx]) || 0) * 180) / Math.PI;
            series[j][s] = deg;
            if (deg < minDeg) minDeg = deg;
            if (deg > maxDeg) maxDeg = deg;
        }
    }

    if (!Number.isFinite(minDeg) || !Number.isFinite(maxDeg)) {
        return { minDeg: null, maxDeg: null };
    }
    if (maxDeg - minDeg < 0.5) {
        const center = 0.5 * (maxDeg + minDeg);
        minDeg = center - 0.25;
        maxDeg = center + 0.25;
    }

    const padX = 10;
    const padY = 10;
    const plotW = Math.max(1, width - 2 * padX);
    const plotH = Math.max(1, height - 2 * padY);
    const xDen = Math.max(1, sampleCount - 1);
    const yDen = Math.max(1e-9, maxDeg - minDeg);
    const toX = (i) => padX + (i / xDen) * plotW;
    const toY = (deg) => padY + (1 - ((deg - minDeg) / yDen)) * plotH;

    ctx.strokeStyle = 'rgba(100, 116, 139, 0.34)';
    ctx.lineWidth = 1;
    for (let g = 1; g <= 3; g++) {
        const y = padY + (g / 4) * plotH;
        ctx.beginPath();
        ctx.moveTo(padX, y);
        ctx.lineTo(padX + plotW, y);
        ctx.stroke();
    }
    if (minDeg < 0 && maxDeg > 0) {
        const y0 = toY(0);
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.52)';
        ctx.beginPath();
        ctx.moveTo(padX, y0);
        ctx.lineTo(padX + plotW, y0);
        ctx.stroke();
    }

    for (let j = 0; j < series.length; j++) {
        const line = series[j];
        if (!Array.isArray(line) || line.length === 0) continue;
        ctx.strokeStyle = JOINT_MONITOR_COLORS[j % JOINT_MONITOR_COLORS.length];
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(toX(0), toY(line[0]));
        for (let i = 1; i < line.length; i++) {
            ctx.lineTo(toX(i), toY(line[i]));
        }
        ctx.stroke();
    }

    return { minDeg, maxDeg };
}

function normalizeJointIndices(indices, qLen) {
    if (!Array.isArray(indices)) return [];
    const seen = new Set();
    const out = [];
    for (const raw of indices) {
        const idx = Number(raw);
        if (!Number.isInteger(idx) || idx < 0 || idx >= qLen || seen.has(idx)) continue;
        seen.add(idx);
        out.push(idx);
    }
    return out;
}

function normalizeJointGroups(rawGroups, qLen) {
    const groups = rawGroups && typeof rawGroups === 'object' ? rawGroups : {};
    const normalized = {};
    for (const key of JOINT_MONITOR_GROUP_KEYS) {
        const group = groups[key] || {};
        normalized[key] = {
            label: group.label || JOINT_MONITOR_GROUP_LABELS[key] || key,
            indices: normalizeJointIndices(group.indices, qLen),
        };
    }
    return normalized;
}

function formatJointValues(indices, qNow, jointNames) {
    if (!Array.isArray(indices) || indices.length === 0) return '-';
    const names = Array.isArray(jointNames) ? jointNames : [];
    const maxItems = 14;
    const n = Math.min(indices.length, maxItems);
    const chunks = [];
    for (let i = 0; i < n; i++) {
        const idx = indices[i];
        const name = names[idx] || `q${idx}`;
        const deg = ((Number(qNow[idx]) || 0) * 180 / Math.PI).toFixed(2);
        chunks.push(`${name}: ${deg}°`);
    }
    if (indices.length > n) chunks.push(`+${indices.length - n} joints`);
    return chunks.join(' | ');
}

function resetMonitorCards() {
    for (const key of JOINT_MONITOR_GROUP_KEYS) {
        const panel = monitorEls.groups[key];
        if (!panel) continue;
        if (panel.meta) panel.meta.textContent = '0 joints';
        if (panel.values) panel.values.textContent = '-';
        drawJointChart(panel.canvas, [], []);
    }
}

function applyMonitorSnapshot(payload) {
    const qNow = Array.isArray(payload?.qNow) ? payload.qNow.slice() : null;
    if (!qNow || qNow.length === 0) return;
    const qLen = qNow.length;
    if (monitorQDim !== qLen) {
        monitorHistory = [];
        monitorQDim = qLen;
    }

    monitorHistory.push({
        t: Number(payload.ts) || performance.now(),
        q: qNow,
    });
    if (monitorHistory.length > JOINT_MONITOR_HISTORY_LIMIT) {
        monitorHistory.splice(0, monitorHistory.length - JOINT_MONITOR_HISTORY_LIMIT);
    }

    const groups = normalizeJointGroups(payload.groups, qLen);
    const jointNames = Array.isArray(payload.jointNames) ? payload.jointNames : [];

    for (const key of JOINT_MONITOR_GROUP_KEYS) {
        const panel = monitorEls.groups[key];
        const group = groups[key];
        if (!panel || !group) continue;
        const chartInfo = drawJointChart(panel.canvas, monitorHistory, group.indices);
        const rangeText = (Number.isFinite(chartInfo.minDeg) && Number.isFinite(chartInfo.maxDeg))
            ? `${chartInfo.minDeg.toFixed(1)}~${chartInfo.maxDeg.toFixed(1)}°`
            : '-';
        if (panel.meta) panel.meta.textContent = `${group.indices.length} joints | ${rangeText}`;
        if (panel.values) panel.values.textContent = formatJointValues(group.indices, qNow, jointNames);
    }

    const stateText = String(payload.stateText || '监控中');
    setMonitorState(stateText, false);
    monitorLastSnapshotPerfTs = performance.now();
    monitorLastSnapshotRealTs = Date.now();
    setMonitorUpdated(monitorLastSnapshotRealTs);
}

function handleMonitorStatus(payload) {
    const available = payload?.available !== false;
    if (available) return;
    monitorHistory = [];
    monitorQDim = 0;
    monitorLastSnapshotPerfTs = 0;
    monitorLastSnapshotRealTs = 0;
    setMonitorState(payload?.message || '等待主页面数据', false);
    setMonitorUpdated(0);
    resetMonitorCards();
}

function handleMonitorMessage(event) {
    const payload = event?.data;
    if (!payload || payload.channel !== JOINT_MONITOR_CHANNEL_NAME) return;
    if (payload.type === 'snapshot') {
        applyMonitorSnapshot(payload);
        return;
    }
    if (payload.type === 'status') {
        handleMonitorStatus(payload);
    }
}

function refreshMonitorFreshness() {
    if (!monitorLastSnapshotPerfTs) return;
    const ageMs = performance.now() - monitorLastSnapshotPerfTs;
    if (ageMs >= JOINT_MONITOR_STALE_RESET_MS) {
        setMonitorState(`连接中断 ${(ageMs / 1000).toFixed(1)}s，请检查主页面`, true);
        return;
    }
    if (ageMs >= JOINT_MONITOR_STALE_WARN_MS) {
        setMonitorState(`数据延迟 ${(ageMs / 1000).toFixed(1)}s`, true);
        return;
    }
    if (monitorEls.state?.classList.contains('stale')) {
        setMonitorState(monitorEls.state.textContent || '监控中', false);
    }
}

function initJointMonitorWindow() {
    resetMonitorCards();
    setMonitorState('等待主页面数据', false);
    setMonitorUpdated(0);

    if (typeof window.BroadcastChannel !== 'function') {
        setMonitorState('当前浏览器不支持 BroadcastChannel', true);
        return;
    }

    try {
        monitorChannel = new BroadcastChannel(JOINT_MONITOR_CHANNEL_NAME);
        monitorChannel.onmessage = handleMonitorMessage;
    } catch (error) {
        console.error('独立关节监控页初始化失败:', error);
        setMonitorState('初始化失败，无法接收实时数据', true);
        return;
    }

    setInterval(refreshMonitorFreshness, 500);
    window.addEventListener('beforeunload', () => {
        if (monitorChannel) {
            monitorChannel.close();
            monitorChannel = null;
        }
    });
}

initJointMonitorWindow();
