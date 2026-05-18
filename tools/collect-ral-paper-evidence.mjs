#!/usr/bin/env node

import { mkdir, readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, '..');
const OUTPUT_DIR = process.env.RAL_EVIDENCE_OUTPUT_DIR
    || path.join(REPO_ROOT, 'artifacts', 'paper-evidence');

const CONTROLLER_SOURCE = path.join(
    REPO_ROOT,
    'artifacts',
    'humanoid-manip-limit-experiment',
    'controller_comparison_target_075_tr_w_wvfi.json'
);
const POLISHING_SOURCE = path.join(
    REPO_ROOT,
    'artifacts',
    'humanoid-surface-polishing',
    'cooperative_surface_dq_setpoint_waist_cylinder_capture_30ticks_tr_waistcorridor_m0p5_0p2_current.json'
);

const taskLabels = new Map([
    ['absolute', 'Arm abs.'],
    ['decoupled-upright', 'Waist abs.'],
    ['waist-distance', 'Waist dist.']
]);

function relativePath(filePath) {
    return path.relative(REPO_ROOT, filePath).replaceAll(path.sep, '/');
}

function csvEscape(value) {
    if (value === null || value === undefined || Number.isNaN(value)) return '';
    const text = String(value);
    return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function buildCsv(rows, columns) {
    return `${[
        columns.map((column) => column.header),
        ...rows.map((row) => columns.map((column) => row[column.key]))
    ].map((row) => row.map(csvEscape).join(',')).join('\n')}\n`;
}

function finiteNumber(value) {
    if (value === null || value === undefined || value === '') return null;
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric : null;
}

function meanOf(values) {
    const finite = values.map(Number).filter(Number.isFinite);
    if (finite.length === 0) return null;
    return finite.reduce((sum, value) => sum + value, 0) / finite.length;
}

function maxOf(values) {
    const finite = values.map(Number).filter(Number.isFinite);
    if (finite.length === 0) return null;
    return Math.max(...finite);
}

function fixed(value, digits = 3) {
    const numeric = finiteNumber(value);
    return numeric === null ? '--' : numeric.toFixed(digits);
}

function compactFixed(value, digits = 4) {
    const numeric = finiteNumber(value);
    return numeric === null ? '--' : numeric.toFixed(digits);
}

function scientific(value, digits = 3) {
    const numeric = finiteNumber(value);
    if (numeric === null) return '--';
    return numeric.toExponential(digits).replace('e', 'e');
}

function latexNum(value, digits = 3) {
    const text = scientific(value, digits);
    return text === '--' ? '--' : `\\num{${text}}`;
}

function methodLatexLabel(method = '') {
    if (method === 'ProxQP+w_vfi') return 'ProxQP+$w^{\\mathrm{vfi}}$';
    if (method === 'ProxQP+w') return 'ProxQP+$w$';
    return 'ProxQP';
}

function methodCsvLabel(controllerLabel = '') {
    if (controllerLabel.includes('TR+w_vfi')) return 'ProxQP+w_vfi';
    if (controllerLabel.includes('TR+w')) return 'ProxQP+w';
    return 'ProxQP';
}

function findSummaryRow(rows, run) {
    return rows.find((row) => row.scenario === run.mode && row.controller === run.controllerLabel) || {};
}

function countAcceptedTicks(steps = []) {
    return steps.filter((step) => step?.ok !== false && step?.converged !== false).length;
}

function buildClosedLoopRows(controllerData) {
    const runs = Array.isArray(controllerData.runs) ? controllerData.runs : [];
    const rows = Array.isArray(controllerData.rows) ? controllerData.rows : [];
    return runs.map((run) => {
        const summaryRow = findSummaryRow(rows, run);
        const steps = Array.isArray(run.steps) ? run.steps : [];
        const acceptedTicks = countAcceptedTicks(steps);
        const rejectedTicks = Math.max(0, steps.length - acceptedTicks);
        return {
            task: taskLabels.get(run.mode) || run.mode,
            source_task: run.mode,
            method: methodCsvLabel(run.controllerLabel),
            avg_command_residual: finiteNumber(summaryRow.avg_task_deviation),
            avg_preserved_residual: finiteNumber(summaryRow.avg_invariant_deviation),
            avg_joint_clearance_rad: finiteNumber(summaryRow.avg_joint_clearance_rad),
            final_joint_clearance_rad: finiteNumber(run.finalJointClearanceMin),
            avg_waist_clearance_m: finiteNumber(summaryRow.avg_waist_clearance_m),
            final_waist_clearance_m: finiteNumber(run.finalWaistClearanceMin),
            final_w: finiteNumber(run.finalCCTSMW),
            final_w_vfi: finiteNumber(run.finalCCTSMWvfi),
            terminal_reason: summaryRow.terminal_reason || run.terminalReason || '',
            accepted_ticks: acceptedTicks,
            rejected_ticks: rejectedTicks,
            source_artifact: relativePath(CONTROLLER_SOURCE)
        };
    });
}

function buildClosedLoopLatex(rows) {
    const taskOrder = ['Arm abs.', 'Waist abs.', 'Waist dist.'];
    const grouped = taskOrder.map((task) => rows.filter((row) => row.task === task)).filter((group) => group.length > 0);
    const body = [];
    for (const [groupIndex, group] of grouped.entries()) {
        if (groupIndex > 0) body.push('\\midrule');
        for (const [rowIndex, row] of group.entries()) {
            const taskCell = rowIndex === 0 ? `\\multirow{${group.length}}{*}{${row.task}}` : '';
            const waist = row.avg_waist_clearance_m === null
                ? '--'
                : `${fixed(row.avg_waist_clearance_m, 3)}/${compactFixed(row.final_waist_clearance_m, 4)}`;
            body.push([
                taskCell,
                methodLatexLabel(row.method),
                `${latexNum(row.avg_command_residual, 2)}/${latexNum(row.avg_preserved_residual, 2)}`,
                `${fixed(row.avg_joint_clearance_rad, 3)}/${compactFixed(row.final_joint_clearance_rad, 4)}`,
                waist,
                `${latexNum(row.final_w, 3)}/${latexNum(row.final_w_vfi, 3)}`
            ].join(' & ') + ' \\\\');
        }
    }
    return `\\begin{tabular}{llcccc}
\\toprule
Task & Method & Avg. dev. $e_y/e_z$ & Joint clr. & Waist clr. & Final $w/w^{\\mathrm{vfi}}$ \\\\
\\midrule
${body.join('\n')}
\\bottomrule
\\end{tabular}
`;
}

function buildPolishingDemo(polishingData) {
    const steps = Array.isArray(polishingData.steps) ? polishingData.steps : [];
    const summary = polishingData.summary || {};
    return {
        demo: 'Curved-surface polishing',
        evidence_type: 'controller-log',
        mean_command_residual: finiteNumber(summary.meanCommandError),
        max_command_residual: maxOf(steps.map((step) => step.commandError)),
        mean_invariant_residual: finiteNumber(summary.meanInvariantError),
        max_invariant_residual: maxOf(steps.map((step) => step.invariantError)),
        max_violation: finiteNumber(summary.maxViolation),
        min_joint_clearance_rad: finiteNumber(summary.minJointClearance),
        min_waist_clearance_m: finiteNumber(summary.minWaistClearance),
        applied_ticks: finiteNumber(summary.appliedTicks),
        source_artifact: relativePath(POLISHING_SOURCE)
    };
}

function buildDemoRows(polishingData) {
    return [
        {
            demo: 'Box grasping',
            evidence_type: 'task-video',
            mean_command_residual: null,
            max_command_residual: null,
            mean_invariant_residual: null,
            max_invariant_residual: null,
            max_violation: null,
            min_joint_clearance_rad: null,
            min_waist_clearance_m: null,
            applied_ticks: null,
            source_artifact: 'videos/video1.mp4'
        },
        {
            demo: 'Box carrying',
            evidence_type: 'task-video',
            mean_command_residual: null,
            max_command_residual: null,
            mean_invariant_residual: null,
            max_invariant_residual: null,
            max_violation: null,
            min_joint_clearance_rad: null,
            min_waist_clearance_m: null,
            applied_ticks: null,
            source_artifact: 'videos/video3.mp4'
        },
        buildPolishingDemo(polishingData)
    ];
}

function buildReadme(manifest) {
    return `# RA-L Paper Evidence Export

Generated by \`npm run collect:ral-evidence\`.

This folder packages the controller-log rows used by the RA-L manuscript and the task-demo evidence summary shown on the project page. The export keeps task, clearance, manipulability, acceptance-count, and provenance fields only.

## Sources

- Closed-loop controller comparison: \`${manifest.sources.closed_loop_controller_log}\`
- Curved-surface polishing log: \`${manifest.sources.polishing_controller_log}\`
- Qualitative task videos: \`videos/video1.mp4\`, \`videos/video3.mp4\`, \`videos/video2.mp4\`

## Outputs

- \`${manifest.outputs.closed_loop_csv}\`
- \`${manifest.outputs.closed_loop_json}\`
- \`${manifest.outputs.closed_loop_latex}\`
- \`${manifest.outputs.task_demo_csv}\`
- \`${manifest.outputs.task_demo_json}\`
`;
}

const controllerData = JSON.parse(await readFile(CONTROLLER_SOURCE, 'utf8'));
const polishingData = JSON.parse(await readFile(POLISHING_SOURCE, 'utf8'));

const closedLoopRows = buildClosedLoopRows(controllerData);
const demoRows = buildDemoRows(polishingData);

const closedLoopColumns = [
    { key: 'task', header: 'Task' },
    { key: 'method', header: 'Method' },
    { key: 'avg_command_residual', header: 'Avg. command residual' },
    { key: 'avg_preserved_residual', header: 'Avg. preserved residual' },
    { key: 'avg_joint_clearance_rad', header: 'Avg. joint clearance (rad)' },
    { key: 'final_joint_clearance_rad', header: 'Final joint clearance (rad)' },
    { key: 'avg_waist_clearance_m', header: 'Avg. waist clearance (m)' },
    { key: 'final_waist_clearance_m', header: 'Final waist clearance (m)' },
    { key: 'final_w', header: 'Final w' },
    { key: 'final_w_vfi', header: 'Final w_vfi' },
    { key: 'terminal_reason', header: 'Terminal reason' },
    { key: 'accepted_ticks', header: 'Accepted ticks' },
    { key: 'rejected_ticks', header: 'Rejected ticks' },
    { key: 'source_artifact', header: 'Source artifact' }
];

const demoColumns = [
    { key: 'demo', header: 'Demo' },
    { key: 'evidence_type', header: 'Evidence type' },
    { key: 'mean_command_residual', header: 'Mean command residual' },
    { key: 'max_command_residual', header: 'Max command residual' },
    { key: 'mean_invariant_residual', header: 'Mean invariant residual' },
    { key: 'max_invariant_residual', header: 'Max invariant residual' },
    { key: 'max_violation', header: 'Max violation' },
    { key: 'min_joint_clearance_rad', header: 'Min joint clearance (rad)' },
    { key: 'min_waist_clearance_m', header: 'Min waist clearance (m)' },
    { key: 'applied_ticks', header: 'Applied ticks' },
    { key: 'source_artifact', header: 'Source artifact' }
];

await mkdir(OUTPUT_DIR, { recursive: true });

const outputPaths = {
    closedLoopCsv: path.join(OUTPUT_DIR, 'closed_loop_hardware_log_summary.csv'),
    closedLoopJson: path.join(OUTPUT_DIR, 'closed_loop_hardware_log_summary.json'),
    closedLoopLatex: path.join(OUTPUT_DIR, 'closed_loop_hardware_log_summary_latex.tex'),
    taskDemoCsv: path.join(OUTPUT_DIR, 'task_demo_summary.csv'),
    taskDemoJson: path.join(OUTPUT_DIR, 'task_demo_summary.json'),
    manifestJson: path.join(OUTPUT_DIR, 'paper_evidence_manifest.json'),
    readme: path.join(OUTPUT_DIR, 'README.md')
};

await writeFile(outputPaths.closedLoopCsv, buildCsv(closedLoopRows, closedLoopColumns), 'utf8');
await writeFile(outputPaths.closedLoopJson, `${JSON.stringify(closedLoopRows, null, 2)}\n`, 'utf8');
await writeFile(outputPaths.closedLoopLatex, buildClosedLoopLatex(closedLoopRows), 'utf8');
await writeFile(outputPaths.taskDemoCsv, buildCsv(demoRows, demoColumns), 'utf8');
await writeFile(outputPaths.taskDemoJson, `${JSON.stringify(demoRows, null, 2)}\n`, 'utf8');

const manifest = {
    generated_at: new Date().toISOString(),
    generator: relativePath(fileURLToPath(import.meta.url)),
    note: 'Controller-log evidence export for the RA-L manuscript. The export keeps task, clearance, manipulability, acceptance-count, and provenance fields only.',
    sources: {
        closed_loop_controller_log: relativePath(CONTROLLER_SOURCE),
        polishing_controller_log: relativePath(POLISHING_SOURCE)
    },
    outputs: {
        closed_loop_csv: relativePath(outputPaths.closedLoopCsv),
        closed_loop_json: relativePath(outputPaths.closedLoopJson),
        closed_loop_latex: relativePath(outputPaths.closedLoopLatex),
        task_demo_csv: relativePath(outputPaths.taskDemoCsv),
        task_demo_json: relativePath(outputPaths.taskDemoJson)
    },
    closed_loop_rows: closedLoopRows.length,
    task_demo_rows: demoRows.length
};

await writeFile(outputPaths.manifestJson, `${JSON.stringify(manifest, null, 2)}\n`, 'utf8');
await writeFile(outputPaths.readme, buildReadme(manifest), 'utf8');

for (const outputPath of Object.values(outputPaths)) {
    console.log(relativePath(outputPath));
}
