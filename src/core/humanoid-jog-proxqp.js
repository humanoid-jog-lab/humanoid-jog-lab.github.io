import {
    buildHardConstraintRows,
    buildTaskQP,
    dot,
    finiteNumber,
    inferDimension,
    maxHalfspaceViolation,
    normalizeTaskBlocks,
    positiveViolationNorm,
    taskResidualNorm,
    vectorNorm
} from './humanoid-jog-qp-controller-utils.js';
import { projectOntoHalfspaces } from './vfi-utils.js';

function qAppliedFrom(qActive, dq) {
    if (!Array.isArray(qActive)) return null;
    return qActive.map((q, i) => finiteNumber(q, 0) + finiteNumber(dq?.[i], 0));
}

function matVecDense(M, x) {
    const rows = Array.isArray(M) ? M.length : 0;
    const out = new Array(rows).fill(0);
    for (let i = 0; i < rows; i++) out[i] = dot(M[i], x);
    return out;
}

function transposedActiveMatVec(A, active, z, n) {
    const out = new Array(n).fill(0);
    for (let a = 0; a < active.length; a++) {
        const rowIndex = active[a];
        const zi = finiteNumber(z?.[rowIndex], 0);
        if (!zi) continue;
        const row = A[rowIndex] || [];
        for (let j = 0; j < n; j++) out[j] += finiteNumber(row[j], 0) * zi;
    }
    return out;
}

function solveLinearSystem(A, b) {
    const n = Array.isArray(A) ? A.length : 0;
    if (n === 0) return [];
    const M = A.map((row, i) => {
        const dense = new Array(n + 1).fill(0);
        for (let j = 0; j < n; j++) dense[j] = finiteNumber(row?.[j], 0);
        dense[n] = finiteNumber(b?.[i], 0);
        return dense;
    });

    for (let col = 0; col < n; col++) {
        let pivotRow = col;
        let pivotMag = Math.abs(finiteNumber(M[col]?.[col], 0));
        for (let row = col + 1; row < n; row++) {
            const mag = Math.abs(finiteNumber(M[row]?.[col], 0));
            if (mag > pivotMag) {
                pivotMag = mag;
                pivotRow = row;
            }
        }
        if (pivotMag < 1e-12) return null;
        if (pivotRow !== col) [M[col], M[pivotRow]] = [M[pivotRow], M[col]];
        const pivot = finiteNumber(M[col][col], 1);
        for (let j = col; j <= n; j++) M[col][j] /= pivot;
        for (let row = 0; row < n; row++) {
            if (row === col) continue;
            const factor = finiteNumber(M[row][col], 0);
            if (Math.abs(factor) < 1e-18) continue;
            for (let j = col; j <= n; j++) M[row][j] -= factor * M[col][j];
        }
    }
    return M.map((row) => finiteNumber(row[n], 0));
}

function buildActiveSet(A, b, x, zOuter, z, mu, alpha) {
    const active = [];
    const eps = 1e-10;
    for (let i = 0; i < A.length; i++) {
        const shifted = dot(A[i], x)
            + mu * finiteNumber(zOuter?.[i], 0)
            + mu * (alpha - 1) * finiteNumber(z?.[i], 0)
            - finiteNumber(b?.[i], 0);
        if (shifted > eps || finiteNumber(z?.[i], 0) > eps) active.push(i);
    }
    return active;
}

function qpObjective(H, f, x) {
    const Hx = matVecDense(H, x);
    return 0.5 * dot(x, Hx) + dot(f, x);
}

function proxMerit({ H, f, A, b, x, xOuter, zOuter, z, rho, mu, alpha }) {
    let value = qpObjective(H, f, x);
    const diff = x.map((xi, i) => finiteNumber(xi, 0) - finiteNumber(xOuter?.[i], 0));
    value += 0.5 * rho * dot(diff, diff);

    let penalty = 0;
    for (let i = 0; i < A.length; i++) {
        const shifted = dot(A[i], x)
            - finiteNumber(b?.[i], 0)
            + mu * (finiteNumber(zOuter?.[i], 0) + (alpha - 1) * finiteNumber(z?.[i], 0));
        const positive = Math.max(0, shifted);
        penalty += positive * positive;
    }
    value += penalty / (2 * Math.max(1e-12, alpha * mu));
    value += 0.5 * Math.max(0, 1 - alpha) * mu * dot(z, z);
    return value;
}

function computeResiduals(H, f, A, b, x, z) {
    const Hx = matVecDense(H, x);
    const stationarity = Hx.map((value, i) => finiteNumber(value, 0) + finiteNumber(f?.[i], 0));
    for (let r = 0; r < A.length; r++) {
        const zr = finiteNumber(z?.[r], 0);
        if (!zr) continue;
        for (let j = 0; j < stationarity.length; j++) stationarity[j] += finiteNumber(A[r]?.[j], 0) * zr;
    }
    const dualResidual = stationarity.reduce((max, value) => Math.max(max, Math.abs(finiteNumber(value, 0))), 0);
    const primalResidual = maxHalfspaceViolation(A, b, x);
    let complementarity = 0;
    for (let r = 0; r < A.length; r++) {
        const slack = finiteNumber(b?.[r], 0) - dot(A[r], x);
        complementarity = Math.max(complementarity, Math.abs(finiteNumber(z?.[r], 0) * slack));
    }
    return { primalResidual, dualResidual, complementarity };
}

function buildKktSystem({ H, f, A, b, x, xOuter, zOuter, z, active, rho, mu, n }) {
    const activeCount = active.length;
    const dim = n + activeCount;
    const K = Array.from({ length: dim }, () => new Array(dim).fill(0));
    const rhs = new Array(dim).fill(0);
    const Hx = matVecDense(H, x);
    const Atz = transposedActiveMatVec(A, active, z, n);

    for (let i = 0; i < n; i++) {
        rhs[i] = -(
            finiteNumber(Hx[i], 0)
            + finiteNumber(f?.[i], 0)
            + rho * (finiteNumber(x[i], 0) - finiteNumber(xOuter?.[i], 0))
            + finiteNumber(Atz[i], 0)
        );
        for (let j = 0; j < n; j++) K[i][j] = finiteNumber(H[i]?.[j], 0);
        K[i][i] += rho;
    }

    for (let a = 0; a < activeCount; a++) {
        const rowIndex = active[a];
        const row = A[rowIndex] || [];
        const zCol = n + a;
        for (let j = 0; j < n; j++) {
            const value = finiteNumber(row[j], 0);
            K[j][zCol] = value;
            K[zCol][j] = value;
        }
        K[zCol][zCol] = -mu;
        rhs[zCol] = -(
            dot(row, x)
            + mu * finiteNumber(zOuter?.[rowIndex], 0)
            - mu * finiteNumber(z?.[rowIndex], 0)
            - finiteNumber(b?.[rowIndex], 0)
        );
    }
    return { K, rhs };
}

function computeNewtonStep(params) {
    const { active, n, z } = params;
    const { K, rhs } = buildKktSystem(params);
    const solution = solveLinearSystem(K, rhs);
    if (!Array.isArray(solution)) return null;
    const dx = solution.slice(0, n);
    const dz = z.map((value) => -finiteNumber(value, 0));
    for (let a = 0; a < active.length; a++) {
        dz[active[a]] = finiteNumber(solution[n + a], 0);
    }
    return { dx, dz };
}

function lineSearchUpdate(params, dx, dz) {
    const { H, f, A, b, x, xOuter, zOuter, z, rho, mu, alpha } = params;
    const currentMerit = proxMerit({ H, f, A, b, x, xOuter, zOuter, z, rho, mu, alpha });
    let step = 1;
    let bestX = x;
    let bestZ = z;
    let bestMerit = currentMerit;

    for (let trial = 0; trial < 12; trial++) {
        const candidateX = x.map((value, i) => finiteNumber(value, 0) + step * finiteNumber(dx?.[i], 0));
        const candidateZ = z.map((value, i) => Math.max(0, finiteNumber(value, 0) + step * finiteNumber(dz?.[i], 0)));
        const merit = proxMerit({
            H,
            f,
            A,
            b,
            x: candidateX,
            xOuter,
            zOuter,
            z: candidateZ,
            rho,
            mu,
            alpha
        });
        if (Number.isFinite(merit) && merit <= currentMerit + 1e-12) {
            return { x: candidateX, z: candidateZ, merit, accepted: true };
        }
        if (Number.isFinite(merit) && merit < bestMerit) {
            bestMerit = merit;
            bestX = candidateX;
            bestZ = candidateZ;
        }
        step *= 0.5;
    }
    return {
        x: bestX === x ? x.map((value, i) => finiteNumber(value, 0) + 1e-3 * finiteNumber(dx?.[i], 0)) : bestX,
        z: bestZ === z ? z.map((value, i) => Math.max(0, finiteNumber(value, 0) + 1e-3 * finiteNumber(dz?.[i], 0))) : bestZ,
        merit: bestMerit,
        accepted: bestX !== x
    };
}

function runProxQP({ H, f, A, b, x0, options }) {
    const n = H.length;
    const m = A.length;
    let xOuter = Array.isArray(x0) ? x0.slice(0, n).map((v) => finiteNumber(v, 0)) : new Array(n).fill(0);
    while (xOuter.length < n) xOuter.push(0);
    let x = xOuter.slice();
    let zOuter = new Array(m).fill(0);
    let z = zOuter.slice();
    let mu = Math.max(1e-9, finiteNumber(options.mu, 1e-2));
    const rho = Math.max(1e-12, finiteNumber(options.rho, 1e-7));
    const alpha = Math.max(0.05, Math.min(0.99, finiteNumber(options.alpha, 0.95)));
    const outerMaxIterations = Math.max(1, Math.floor(finiteNumber(options.outerMaxIterations, 12)));
    const innerMaxIterations = Math.max(1, Math.floor(finiteNumber(options.innerMaxIterations, 16)));
    const tolerance = Math.max(0, finiteNumber(options.tolerance, 1e-8));
    let bclTolerance = Math.max(tolerance, finiteNumber(options.bclTolerance, 1e-6));
    const muShrink = Math.max(0.05, Math.min(0.95, finiteNumber(options.muShrink, 0.5)));
    let totalInnerIterations = 0;
    let lastActiveSize = 0;
    let acceptedOuter = 0;

    for (let outer = 0; outer < outerMaxIterations; outer++) {
        x = xOuter.slice();
        z = zOuter.slice();

        for (let inner = 0; inner < innerMaxIterations; inner++) {
            const active = buildActiveSet(A, b, x, zOuter, z, mu, alpha);
            lastActiveSize = active.length;
            const step = computeNewtonStep({ H, f, A, b, x, xOuter, zOuter, z, active, rho, mu, n });
            totalInnerIterations += 1;
            if (!step) break;
            const stepNorm = Math.max(vectorNorm(step.dx), vectorNorm(step.dz));
            const update = lineSearchUpdate({ H, f, A, b, x, xOuter, zOuter, z, rho, mu, alpha }, step.dx, step.dz);
            x = update.x.map((value) => finiteNumber(value, 0));
            z = update.z.map((value) => Math.max(0, finiteNumber(value, 0)));
            const residuals = computeResiduals(H, f, A, b, x, z);
            if (stepNorm <= Math.max(tolerance, 1e-10) || (
                residuals.primalResidual <= bclTolerance &&
                residuals.dualResidual <= Math.max(1e-6, Math.sqrt(Math.max(tolerance, 1e-16)))
            )) {
                break;
            }
        }

        const residuals = computeResiduals(H, f, A, b, x, z);
        const acceptDual = residuals.primalResidual <= bclTolerance || outer === outerMaxIterations - 1;
        xOuter = x.slice();
        if (acceptDual) {
            zOuter = z.slice();
            acceptedOuter += 1;
            bclTolerance = Math.max(tolerance, bclTolerance * 0.5);
        } else {
            mu = Math.max(1e-9, mu * muShrink);
        }
        const finalResiduals = computeResiduals(H, f, A, b, xOuter, zOuter);
        if (
            finalResiduals.primalResidual <= tolerance &&
            finalResiduals.dualResidual <= Math.max(1e-6, Math.sqrt(Math.max(tolerance, 1e-16))) &&
            finalResiduals.complementarity <= Math.max(1e-8, Math.sqrt(Math.max(tolerance, 1e-16)))
        ) {
            return {
                x: xOuter,
                z: zOuter,
                converged: true,
                outerIterations: outer + 1,
                innerIterations: totalInnerIterations,
                acceptedOuter,
                activeSetSize: lastActiveSize,
                mu,
                rho,
                ...finalResiduals
            };
        }
    }

    const residuals = computeResiduals(H, f, A, b, xOuter, zOuter);
    return {
        x: xOuter,
        z: zOuter,
        converged: false,
        outerIterations: outerMaxIterations,
        innerIterations: totalInnerIterations,
        acceptedOuter,
        activeSetSize: lastActiveSize,
        mu,
        rho,
        ...residuals
    };
}

export function solveHumanoidJogProxQP(options = {}) {
    const n = inferDimension(options);
    if (n <= 0) {
        return {
            solver: 'proxqp',
            converged: true,
            reason: 'empty_problem',
            dq: [],
            qApplied: Array.isArray(options.qActive) ? options.qActive.slice() : null,
            activeResidualNorm: 0,
            maxViolation: 0,
            iterations: 0,
            stages: [],
            qp_status: 'empty',
            proxqp_outer_iterations: 0,
            proxqp_inner_iterations: 0,
            proxqp_primal_residual: 0,
            proxqp_dual_residual: 0,
            proxqp_complementarity: 0,
            proxqp_mu: 0,
            proxqp_rho: 0
        };
    }

    const blocks = normalizeTaskBlocks(options.taskBlocks, n);
    const hardConstraints = options.hardConstraints || {};
    const { A, b } = buildHardConstraintRows(n, hardConstraints);
    const damping = Math.max(0, finiteNumber(options.damping, 1e-6));
    const { H, f } = buildTaskQP(n, blocks, damping);
    const tolerance = Math.max(0, finiteNumber(options.tolerance, 1e-8));
    const feasibleAcceptanceTolerance = Math.max(
        tolerance,
        Math.max(0, finiteNumber(options.feasibleAcceptanceTolerance, 1e-8))
    );
    const x0 = Array.isArray(options.x0) ? options.x0 : new Array(n).fill(0);

    const prox = runProxQP({
        H,
        f,
        A,
        b,
        x0,
        options: {
            ...options,
            tolerance
        }
    });

    let dq = Array.isArray(prox.x) ? prox.x.slice(0, n).map((v) => finiteNumber(v, 0)) : new Array(n).fill(0);
    const rawViolation = maxHalfspaceViolation(A, b, dq);
    let closestFeasibleShiftNorm = 0;
    let projected = false;
    if (rawViolation > feasibleAcceptanceTolerance && A.length > 0) {
        const candidate = projectOntoHalfspaces(dq, A, b, {
            passes: Math.max(12, Math.floor(finiteNumber(options.projectionPasses, 24))),
            tolerance: feasibleAcceptanceTolerance
        }).slice(0, n).map((v) => finiteNumber(v, 0));
        const candidateViolation = maxHalfspaceViolation(A, b, candidate);
        if (candidateViolation < rawViolation - 1e-12) {
            closestFeasibleShiftNorm = vectorNorm(candidate.map((value, i) => value - finiteNumber(dq[i], 0)));
            dq = candidate;
            projected = true;
        }
    }

    const residuals = computeResiduals(H, f, A, b, dq, prox.z || new Array(A.length).fill(0));
    const maxViolation = maxHalfspaceViolation(A, b, dq);
    const slackNorm = positiveViolationNorm(A, b, dq);
    const activeResidualNorm = taskResidualNorm(blocks, dq);
    const hardFeasible = maxViolation <= feasibleAcceptanceTolerance;
    const converged = hardFeasible && (prox.converged || residuals.dualResidual <= Math.max(1e-5, Math.sqrt(Math.max(tolerance, 1e-16))));
    const qpStatus = converged ? 'solved' : (hardFeasible ? 'limited' : 'failed');
    const reason = converged
        ? 'converged'
        : (hardFeasible ? 'dual_limited' : 'constraint_violation');
    const messageParts = [
        `prox_outer=${prox.outerIterations}`,
        `prox_inner=${prox.innerIterations}`,
        `primal=${residuals.primalResidual.toExponential(2)}`,
        `dual=${residuals.dualResidual.toExponential(2)}`,
        `compl=${residuals.complementarity.toExponential(2)}`,
        `mu=${finiteNumber(prox.mu, 0).toExponential(2)}`
    ];
    if (projected) messageParts.push(`closest_feasible_shift=${closestFeasibleShiftNorm.toExponential(2)}`);
    if (!converged) messageParts.push('best_effort');

    return {
        solver: 'proxqp',
        converged,
        reason,
        dq,
        qApplied: qAppliedFrom(options.qActive, dq),
        activeResidualNorm,
        maxViolation,
        iterations: prox.innerIterations,
        stages: [{
            label: 'proxqp-proximal-augmented-lagrangian',
            rows: blocks.reduce((sum, block) => sum + block.J.length, 0),
            activeRows: A.length,
            activeSetSize: prox.activeSetSize,
            primalResidual: residuals.primalResidual,
            dualResidual: residuals.dualResidual,
            complementarity: residuals.complementarity
        }],
        active_rows: A.length,
        slack_norm: slackNorm,
        qp_status: qpStatus,
        qp_objective: qpObjective(H, f, dq),
        qp_message: messageParts.join('; '),
        stepNorm: vectorNorm(dq),
        proxqp_outer_iterations: prox.outerIterations,
        proxqp_inner_iterations: prox.innerIterations,
        proxqp_primal_residual: residuals.primalResidual,
        proxqp_dual_residual: residuals.dualResidual,
        proxqp_complementarity: residuals.complementarity,
        proxqp_mu: prox.mu,
        proxqp_rho: prox.rho,
        proxqp_active_set_size: prox.activeSetSize,
        proxqp_closest_feasible_shift_norm: closestFeasibleShiftNorm
    };
}
