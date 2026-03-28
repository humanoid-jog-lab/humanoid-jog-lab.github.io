/**
 * Manipulability-Aware Predictive Certification Controller
 * Algorithm 1: branch generation → task-equivalent projection → branch-local QP
 *              → predictive rollout → horizon-aware selection
 *
 * Reference: Section V of the paper
 */
function asFinite(v, fb = 0) { return Number.isFinite(v) ? v : fb; }

/**
 * Solve linear system Ax = b via Gaussian elimination with partial pivoting.
 * Returns x (length n), or null on failure.
 */
function solveLinear(A, b) {
    const n = A.length;
    // Augmented matrix [A | b]
    const M = A.map((row, i) => [...row.map(v => asFinite(v)), asFinite(b[i])]);
    for (let i = 0; i < n; i++) {
        let maxRow = i;
        for (let k = i + 1; k < n; k++) if (Math.abs(M[k][i]) > Math.abs(M[maxRow][i])) maxRow = k;
        [M[i], M[maxRow]] = [M[maxRow], M[i]];
        if (Math.abs(M[i][i]) < 1e-14) return null;
        for (let k = i + 1; k < n; k++) {
            const f = M[k][i] / M[i][i];
            for (let j = i; j <= n; j++) M[k][j] -= f * M[i][j];
        }
    }
    const x = new Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) {
        x[i] = M[i][n];
        for (let j = i + 1; j < n; j++) x[i] -= M[i][j] * x[j];
        x[i] /= M[i][i];
    }
    return x;
}

/**
 * DLS (damped least squares) solve: dq = J^T (J J^T + λ²I)^{-1} e
 * For m×n J, e is m-vector, returns n-vector dq
 */
function dlsSolve(J, e, lambda) {
    const m = J.length, n = J[0].length;
    // Build m×m matrix A = J J^T + λ²I
    const A = Array.from({length: m}, () => new Array(m).fill(0));
    for (let i = 0; i < m; i++) {
        A[i][i] = lambda * lambda;
        for (let j = 0; j < m; j++) {
            for (let k = 0; k < n; k++) A[i][j] += J[i][k] * J[j][k];
        }
    }
    // Solve A * x = e  →  x = A^{-1} e
    const x = solveLinear(A, e);
    if (!x) return new Array(n).fill(0);
    // dq = J^T * x
    const dq = new Array(n).fill(0);
    for (let j = 0; j < n; j++) for (let i = 0; i < m; i++) dq[j] += J[i][j] * x[i];
    return dq;
}

/**
 * Stage 2a: Generate branch seeds along the CCTSM gradient.
 * g_w = ∇w_m^vfi (eq. 1267), seed i: q̃^(i) = q + αᵢ * ĝ_w
 *
 * @param {number[]} q          current active joint vector (length n_active)
 * @param {number[]} scales     scale set A (includes 0 for real branch)
 * @param {function} gradFn     q → gradient of w_m^vfi (length n_active)
 * @returns {number[][]}        array of seeds, one per scale
 */
export function generateBranchSeeds(q, scales, gradFn) {
    const n = q.length;
    const g = gradFn(q);
    const gNorm = Array.isArray(g) ? Math.sqrt(g.reduce((s, v) => s + asFinite(v) ** 2, 0)) : 0;
    return scales.map((alpha) => {
        const seed = q.slice();
        if (gNorm > 1e-12 && alpha !== 0) {
            for (let j = 0; j < n; j++) seed[j] += alpha * asFinite(g[j]) / gNorm;
        }
        return seed;
    });
}

/**
 * Stage 2b: Project a seed to the task-equivalent manifold via iterative DLS IK.
 * Iteratively corrects q̃ so that φ_m(q) ≈ φ_m(q_ref) within tolerance εp.
 *
 * @param {number[]}  seed        initial (seeded) joint vector
 * @param {function}  taskErrFn   q → error vector e (length d_phi, e→0 at target)
 * @param {function}  taskJacFn   q → Jacobian J (d_phi × n_active)
 * @param {number[][]|null} hardLimits  [[lo,hi], ...] per active joint, or null
 * @param {object}    [opts]
 * @returns {number[]} projected joint vector
 */
export function projectToManifold(seed, taskErrFn, taskJacFn, hardLimits = null, opts = {}) {
    const { maxIter = 14, tol = 2e-4, maxStep = 0.01, damping = 2e-3 } = opts;
    const n = seed.length;
    let q = seed.slice();
    for (let iter = 0; iter < maxIter; iter++) {
        const e = taskErrFn(q);
        if (!Array.isArray(e)) break;
        const eNorm = Math.sqrt(e.reduce((s, v) => s + asFinite(v) ** 2, 0));
        if (eNorm < tol) break;
        const J = taskJacFn(q);
        if (!Array.isArray(J) || J.length === 0) break;
        const dq = dlsSolve(J, e, damping);
        const stepNorm = Math.sqrt(dq.reduce((s, v) => s + asFinite(v) ** 2, 0));
        const sc = stepNorm > maxStep ? maxStep / stepNorm : 1.0;
        for (let j = 0; j < n; j++) q[j] += sc * asFinite(dq[j]);
        if (hardLimits) {
            for (let j = 0; j < n; j++) {
                if (hardLimits[j]) q[j] = Math.max(hardLimits[j][0], Math.min(hardLimits[j][1], q[j]));
            }
        }
    }
    return q;
}

/**
 * Stage 4: Roll a branch forward H steps.
 * At each step, calls stepFn(q) → { dq, slack, qNext, reserve, sigma }.
 * Certified horizon h = max ℓ s.t. all steps 0..ℓ-1 satisfy slack≤slackTol and reserve≥0.
 *
 * @param {object} opts
 * @param {number[]}  opts.qStart       starting joint vector
 * @param {number}    opts.horizon       maximum rollout horizon H
 * @param {function}  opts.stepFn        q → { dq, slack, qNext, reserve, sigma } | null
 * @param {number}    [opts.slackTol]   acceptance slack threshold η̄ (default 1e-6)
 * @returns {{ h: number, mu: number, sigma: number, qTerminal: number[] }}
 */
export function rolloutBranch({ qStart, horizon, stepFn, slackTol = 1e-6 }) {
    let q = qStart.slice();
    let h = 0;
    let mu = Infinity;
    let sigma = Infinity;
    for (let ell = 0; ell < horizon; ell++) {
        const result = stepFn(q);
        if (!result) break;
        const { slack, qNext, reserve, sigma: sig } = result;
        if (!Array.isArray(qNext)) break;
        if (asFinite(slack, Infinity) > slackTol) break;   // horizon-limited
        if (asFinite(reserve, -1) < -1e-10) break;            // margin-limited (float-point tolerant)
        h++;
        mu = Math.min(mu, asFinite(reserve, 0));
        sigma = Math.min(sigma, asFinite(sig, 0));
        q = qNext;
    }
    return { h, mu: mu === Infinity ? 0 : mu, sigma: sigma === Infinity ? 0 : sigma, qTerminal: q };
}

/**
 * Stage 5: Select branch by lexicographic rule (Algorithm 1 Stage 5).
 * Ranking: max h → max mu → max sigma → max deltaW → min cost.
 * Hysteresis: keep previous branch if it is in the σ-filtered set and cost satisfies band.
 *
 * @param {Array<{h,mu,sigma,deltaW,cost,branchIdx,dq,qNext}>} branches  one per scale
 * @param {{branchIdx:number, dwell:number}|null} prevState
 * @param {object} [opts]
 * @returns {{ selected: object|null, state: object|null, label: string }}
 */
export function selectBranch(branches, prevState = null, opts = {}) {
    const { hysteresisAbs = 5e-4, hysteresisRel = 0.2, minHold = 2 } = opts;
    const feasible = branches.filter((b) => b.h >= 1);
    if (feasible.length === 0) {
        return { selected: null, state: null, label: 'unreachable' };
    }

    const maxH = Math.max(...feasible.map((b) => b.h));
    const byH = feasible.filter((b) => b.h === maxH);
    const maxMu = Math.max(...byH.map((b) => asFinite(b.mu, 0)));
    const byMu = byH.filter((b) => asFinite(b.mu, 0) >= maxMu - 1e-9);
    const maxSig = Math.max(...byMu.map((b) => asFinite(b.sigma, 0)));
    const bySigma = byMu.filter((b) => asFinite(b.sigma, 0) >= maxSig - 1e-9);
    const maxDeltaW = Math.max(...bySigma.map((b) => asFinite(b.deltaW, 0)));
    const byDeltaW = bySigma.filter((b) => asFinite(b.deltaW, 0) >= maxDeltaW - 1e-12);

    byDeltaW.sort((a, b) => asFinite(a.cost, 0) - asFinite(b.cost, 0));
    const winner = byDeltaW[0];

    // Hysteresis check
    if (prevState && prevState.dwell > 0) {
        const prev = byDeltaW.find((b) => b.branchIdx === prevState.branchIdx);
        if (prev) {
            const costsOk = asFinite(prev.cost, 0) <= asFinite(winner.cost, 0) + hysteresisAbs + hysteresisRel * Math.abs(asFinite(winner.cost, 0));
            if (costsOk) {
                return {
                    selected: prev,
                    state: { branchIdx: prev.branchIdx, dwell: Math.max(0, prevState.dwell - 1) },
                    label: maxH >= /* horizon */ (opts.horizon ?? 3) ? 'reachable' : (prev.mu < 0 ? 'margin-limited' : 'horizon-limited')
                };
            }
        }
    }

    const switched = winner.branchIdx !== (prevState?.branchIdx ?? winner.branchIdx);
    return {
        selected: winner,
        state: { branchIdx: winner.branchIdx, dwell: switched ? minHold : Math.max(0, (prevState?.dwell ?? 0) - 1) },
        label: maxH >= (opts.horizon ?? 3) ? 'reachable' : (winner.mu < 0 ? 'margin-limited' : 'horizon-limited')
    };
}
