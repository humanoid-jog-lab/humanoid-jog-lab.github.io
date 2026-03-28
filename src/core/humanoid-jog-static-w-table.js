function asFinite(value) {
    return Number.isFinite(value) ? value : Number.NaN;
}

export function formatHumanoidStaticManipTaskLabel(task = {}) {
    const pieces = [
        task.taskKind,
        task.structure
    ];
    if (task.locked === true) {
        pieces.push('locked');
    }
    pieces.push(task.metricKey === 'w_abs_hier_vfi' ? 'w_hier_vfi+' : 'w_hier+');
    return pieces.join(' | ');
}

export function buildHumanoidStaticManipTasks({
    fullBodyAbsoluteMode = 'decoupled',
    lockedOnly = false
} = {}) {
    const tasks = [];
    const taskKinds = ['absolute', 'relative'];
    const structures = ['dual-arm', 'full-body'];
    const lockStates = lockedOnly ? [true] : [false, true];
    const metrics = ['w_abs_hier', 'w_abs_hier_vfi'];

    for (const taskKind of taskKinds) {
        for (const structure of structures) {
            for (const locked of lockStates) {
                for (const metricKey of metrics) {
                    let controlMode = taskKind;
                    if (taskKind === 'absolute' && structure === 'full-body') {
                        controlMode = fullBodyAbsoluteMode;
                    } else if (taskKind === 'relative' && structure === 'full-body') {
                        controlMode = 'relative-fullbody';
                    }
                    tasks.push({
                        taskKind,
                        structure,
                        locked,
                        metricKey,
                        controlMode,
                        includeWaist: structure === 'full-body',
                        keepBodyZVertical: controlMode === 'decoupled-upright'
                    });
                }
            }
        }
    }
    return tasks;
}

export function summarizeHumanoidStaticManipRow({
    task = null,
    before = null,
    after = null
} = {}) {
    const wHierBefore = asFinite(before?.cctsm?.w_abs_arm ?? before?.wHier);
    const wHierAfter = asFinite(after?.cctsm?.w_abs_arm ?? after?.wHier);
    const wHierVfiBefore = asFinite(before?.cctsm?.w_abs_arm_vfi ?? before?.wHierVfi);
    const wHierVfiAfter = asFinite(after?.cctsm?.w_abs_arm_vfi ?? after?.wHierVfi);
    const dminBefore = asFinite(before?.dmin);
    const dminAfter = asFinite(after?.dmin);
    return {
        taskDescriptor: formatHumanoidStaticManipTaskLabel(task),
        w_hier_before: wHierBefore,
        w_hier_after: wHierAfter,
        delta_w_hier: Number.isFinite(wHierBefore) && Number.isFinite(wHierAfter)
            ? wHierAfter - wHierBefore
            : Number.NaN,
        w_hier_vfi_before: wHierVfiBefore,
        w_hier_vfi_after: wHierVfiAfter,
        delta_w_hier_vfi: Number.isFinite(wHierVfiBefore) && Number.isFinite(wHierVfiAfter)
            ? wHierVfiAfter - wHierVfiBefore
            : Number.NaN,
        dmin_before: dminBefore,
        dmin_after: dminAfter
    };
}
