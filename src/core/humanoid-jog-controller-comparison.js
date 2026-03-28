const CONTROLLERS = [
    { solverMode: 'vfi', key: 'qp', label: 'new qpJ' },
    { solverMode: 'priority', key: 'nullspace', label: '零空间投影' },
    { solverMode: 'mj_restore', key: 'ours', label: 'Ours' },
    { solverMode: 'mj_restore_pre_w', key: 'ours_vfiw', label: 'Ours + VFI-w' }
];

const TASKS = [
    { mode: 'absolute', key: 'absolute', label: 'absolute', controlMode: 'absolute', includeWaist: false, keepBodyZVertical: false },
    { mode: 'decoupled-upright', key: 'decoupled_upright', label: 'decoupled-upright', controlMode: 'decoupled-upright', includeWaist: true, keepBodyZVertical: true }
];

const POSTURES = [
    { profile: 'nominal', key: 'nominal', label: 'nominal' },
    { profile: 'near-limit', key: 'near_limit', label: 'near-limit' }
];

export function buildHumanoidControllerComparisonTasks() {
    const tasks = [];
    for (const task of TASKS) {
        for (const posture of POSTURES) {
            for (const controller of CONTROLLERS) {
                tasks.push({
                    ...task,
                    ...posture,
                    ...controller,
                    scenarioLabel: `${task.label} | ${posture.label} | ${controller.label}`
                });
            }
        }
    }
    return tasks;
}

export function summarizeHumanoidControllerComparisonRows(results = []) {
    const buckets = new Map();
    for (const result of Array.isArray(results) ? results : []) {
        const mode = typeof result?.mode === 'string' ? result.mode : 'unknown';
        const posture = typeof result?.profile === 'string' ? result.profile : 'unknown';
        const controllerKey = typeof result?.controllerKey === 'string' ? result.controllerKey : 'unknown';
        const bucketKey = `${mode}::${posture}`;
        if (!buckets.has(bucketKey)) {
            buckets.set(bucketKey, {
                mode,
                start_posture: posture,
                qp: Number.NaN,
                nullspace: Number.NaN,
                ours: Number.NaN,
                ours_vfiw: Number.NaN
            });
        }
        const row = buckets.get(bucketKey);
        row[controllerKey] = Number.isFinite(result?.sustainableTicks) ? result.sustainableTicks : Number.NaN;
    }
    return Array.from(buckets.values());
}

export function getHumanoidControllerComparisonColumns() {
    return [
        { key: 'mode', label: 'Mode' },
        { key: 'start_posture', label: 'Start posture' },
        { key: 'qp', label: 'new qpJ', digits: 0 },
        { key: 'nullspace', label: 'Null-space', digits: 0 },
        { key: 'ours', label: 'Ours', digits: 0 },
        { key: 'ours_vfiw', label: 'Ours + VFI-w', digits: 0 }
    ];
}
