export const JOINT2_LOCK_TRAJECTORY_CASE = {
    taskKind: 'absolute',
    structure: 'full-body',
    controlMode: 'decoupled',
    includeWaist: true,
    keepBodyZVertical: false,
    solverMode: 'mj_restore',
    repeatCountDefault: 200,
    baseQ: [
        -0.3, 0.9, -0.6, 0,
        -0.5236, 0.19, 0, -1.0471, 0, 0, 0,
        0.5236, -0.19, 0, 1.0471, 0, 0, 0,
        0, 0
    ],
    metrics: [
        {
            key: 'w_abs_hier',
            label: 'w_hier+',
            method: 'optimizeHumanoidHierarchicalAbsoluteManipulability'
        },
        {
            key: 'w_abs_hier_vfi',
            label: 'w_hier_vfi+',
            method: 'optimizeHumanoidHierarchicalAbsoluteVFIManipulability'
        }
    ],
    joints: [
        {
            name: 'ljoint2',
            label: 'Left joint2',
            index: 5,
            lower: -0.1,
            upper: 0.2
        },
        {
            name: 'rjoint2',
            label: 'Right joint2',
            index: 12,
            lower: -0.2,
            upper: 0.1
        }
    ]
};

export function buildJoint2LockTrajectoryPlotPayload({
    repeatCount = JOINT2_LOCK_TRAJECTORY_CASE.repeatCountDefault,
    samplesByMetric = {}
} = {}) {
    return {
        title: 'Absolute | full-body | joint2 lock',
        repeatCount,
        metrics: JOINT2_LOCK_TRAJECTORY_CASE.metrics.map(({ key, label }) => ({ key, label })),
        joints: JOINT2_LOCK_TRAJECTORY_CASE.joints.map((joint) => ({
            name: joint.name,
            label: joint.label,
            lower: joint.lower,
            upper: joint.upper,
            series: JOINT2_LOCK_TRAJECTORY_CASE.metrics.map((metric) => ({
                key: metric.key,
                label: metric.label,
                steps: (samplesByMetric[metric.key] || []).map((sample) => Number(sample?.step ?? Number.NaN)),
                values: (samplesByMetric[metric.key] || []).map((sample) => Number(sample?.q?.[joint.index] ?? Number.NaN))
            }))
        }))
    };
}
