/**
 * Constrained Cooperative Task-Space Manipulability (CCTSM)
 *
 * Implements Section IV of the paper:
 *   - Dual-quaternion twist-space Jacobian (resolves DQ degeneracy)
 *   - Null-space basis computation via SVD
 *   - Equality-constrained CCTSM  w_m = sqrt(det(J_bar * J_bar^T))
 *   - Inequality-constrained CCTSM  w_m^vfi with VFI attenuation matrix
 *
 * Reference equations: (896), (939), (969), (982)
 */
function asFinite(v, fb = 0) {
    return Number.isFinite(v) ? v : fb;
}

// ─── Pure-JS linear algebra (replaces numeric dependency) ─────────────────────

/**
 * Gaussian elimination determinant for n×n matrix (in-place copy).
 */
function det(A) {
    const n = A.length;
    const M = A.map(row => row.slice());
    let sign = 1;
    for (let i = 0; i < n; i++) {
        let maxRow = i;
        for (let k = i + 1; k < n; k++) {
            if (Math.abs(asFinite(M[k][i])) > Math.abs(asFinite(M[maxRow][i]))) maxRow = k;
        }
        if (maxRow !== i) { [M[i], M[maxRow]] = [M[maxRow], M[i]]; sign = -sign; }
        if (Math.abs(asFinite(M[i][i])) < 1e-14) return 0;
        for (let k = i + 1; k < n; k++) {
            const f = asFinite(M[k][i]) / asFinite(M[i][i]);
            for (let j = i; j < n; j++) M[k][j] = asFinite(M[k][j]) - f * asFinite(M[i][j]);
        }
    }
    let d = sign;
    for (let i = 0; i < n; i++) d *= asFinite(M[i][i]);
    return Number.isFinite(d) ? d : 0;
}

/**
 * Modified Gram-Schmidt null-space basis for m×n matrix J (no numeric needed).
 * Returns n×k array whose columns span null(J), or null if none.
 */
function mgsNullSpace(J, tol = 1e-9) {
    const m = J.length, n = J[0].length;
    // Step 1: orthonormalise rows of J → row-space basis
    const rowBasis = [];
    for (let i = 0; i < m; i++) {
        let v = J[i].map(asFinite);
        for (const u of rowBasis) {
            const d = v.reduce((s, vj, j) => s + vj * u[j], 0);
            for (let j = 0; j < n; j++) v[j] -= d * u[j];
        }
        const norm = Math.sqrt(v.reduce((s, vj) => s + vj * vj, 0));
        if (norm > tol) { for (let j = 0; j < n; j++) v[j] /= norm; rowBasis.push(v); }
    }
    // Step 2: project standard basis vectors against rowBasis → null space
    const nullBasis = [];
    for (let i = 0; i < n; i++) {
        let v = new Array(n).fill(0); v[i] = 1.0;
        for (const u of rowBasis) {
            const d = v.reduce((s, vj, j) => s + vj * u[j], 0);
            for (let j = 0; j < n; j++) v[j] -= d * u[j];
        }
        // Also orthogonalize against already-collected null vectors
        for (const u of nullBasis) {
            const d = v.reduce((s, vj, j) => s + vj * u[j], 0);
            for (let j = 0; j < n; j++) v[j] -= d * u[j];
        }
        const norm = Math.sqrt(v.reduce((s, vj) => s + vj * vj, 0));
        if (norm > tol) { for (let j = 0; j < n; j++) v[j] /= norm; nullBasis.push(v); }
    }
    if (nullBasis.length === 0) return null;
    // Return as n×k column matrix
    const k = nullBasis.length;
    const N = Array.from({ length: n }, () => new Array(k).fill(0));
    for (let i = 0; i < n; i++) for (let ci = 0; ci < k; ci++) N[i][ci] = nullBasis[ci][i];
    return N;
}

// ─── Matrix helpers ───────────────────────────────────────────────────────────

function matMul(A, B) {
    const m = A.length;
    const k = B.length;
    const n = B[0].length;
    const C = Array.from({ length: m }, () => new Array(n).fill(0));
    for (let i = 0; i < m; i++) {
        for (let p = 0; p < k; p++) {
            const aip = asFinite(A[i][p]);
            if (aip === 0) continue;
            for (let j = 0; j < n; j++) {
                C[i][j] += aip * asFinite(B[p][j]);
            }
        }
    }
    return C;
}

function transpose(A) {
    const m = A.length;
    const n = A[0].length;
    return Array.from({ length: n }, (_, j) => Array.from({ length: m }, (__, i) => asFinite(A[i][j])));
}

// ─── Hamilton operators (eq. 891–895) ─────────────────────────────────────────

/**
 * 4×4 left Hamilton operator H4+(q) for quaternion q = [q0, q1, q2, q3].
 * Satisfies: H4+(q) * vec4(p) = vec4(q * p).
 */
export function hamiltonLeft4(q) {
    const [q0, q1, q2, q3] = q.map((v) => asFinite(v));
    return [
        [q0, -q1, -q2, -q3],
        [q1,  q0, -q3,  q2],
        [q2,  q3,  q0, -q1],
        [q3, -q2,  q1,  q0]
    ];
}

/**
 * 8×8 left Hamilton operator H8+(x) for dual quaternion x = [r0,r1,r2,r3,d0,d1,d2,d3].
 * Block form: [[H4+(r), 0], [H4+(d), H4+(r)]]
 */
export function hamiltonLeft8(x) {
    const r = x.slice(0, 4);
    const d = x.slice(4, 8);
    const Hr = hamiltonLeft4(r);
    const Hd = hamiltonLeft4(d);
    const z = [0, 0, 0, 0];
    return [
        [...Hr[0], ...z],
        [...Hr[1], ...z],
        [...Hr[2], ...z],
        [...Hr[3], ...z],
        [...Hd[0], ...Hr[0]],
        [...Hd[1], ...Hr[1]],
        [...Hd[2], ...Hr[2]],
        [...Hd[3], ...Hr[3]]
    ];
}

/**
 * DQ conjugate: x^* = [r0, -r1, -r2, -r3, -d0, d1, d2, d3]
 */
export function dqConjugate(x) {
    return [
        asFinite(x[0]),
        -asFinite(x[1]),
        -asFinite(x[2]),
        -asFinite(x[3]),
        -asFinite(x[4]),
        asFinite(x[5]),
        asFinite(x[6]),
        asFinite(x[7])
    ];
}

// ─── Twist-space Jacobians (eq. 896–906) ──────────────────────────────────────

/**
 * Compute the 6×n body-frame twist Jacobian from an 8×n DQ pose Jacobian.
 *
 *   J_xi = 2 * P6 * H8+(x^*) * J8
 *
 * P6 selects imaginary-component rows {1,2,3,5,6,7} (0-indexed) of the
 * 8-vector, yielding [omega_x, omega_y, omega_z, v_x, v_y, v_z].
 *
 * @param {number[][]} J8  – 8×n DQ Jacobian (J8[row][col])
 * @param {number[]}   x8  – 8-element current DQ pose array
 * @returns {number[][] | null} 6×n twist Jacobian, or null on error
 */
let _twistJacLogCount = 0;
export function computeTwistJacobian(J8, x8) {
    if (!Array.isArray(J8) || J8.length < 8 || !Array.isArray(x8) || x8.length < 8) return null;
    const n = J8[0].length;
    if (n <= 0) return null;
    const H = hamiltonLeft8(dqConjugate(x8));
    const P6 = [1, 2, 3, 5, 6, 7]; // imaginary-component row indices
    const J_xi = Array.from({ length: 6 }, () => new Array(n).fill(0));
    for (let col = 0; col < n; col++) {
        // Apply H to column col of J8
        for (let ri = 0; ri < 6; ri++) {
            const Hrow = H[P6[ri]];
            let sum = 0;
            for (let k = 0; k < 8; k++) {
                sum += asFinite(Hrow[k]) * asFinite(J8[k][col]);
            }
            J_xi[ri][col] = 2 * sum;
        }
    }
    if (_twistJacLogCount < 2) {
        const cnt = ++_twistJacLogCount;
        const label = cnt === 1 ? 'J_xi_y(abs)' : 'J_xi_z(rel)';
        const rowNorms = J_xi.map(r => Math.sqrt(r.reduce((s,v)=>s+v*v,0)).toExponential(3));
        const j8rowNorms = J8.map(r => Math.sqrt(r.reduce((s,v)=>s+asFinite(v)*asFinite(v),0)).toExponential(3));
        console.log('[twistJac]', label,
            '| J8:', J8.length,'×',n, '| x8:', x8.map(v=>v.toFixed(4)),
            '| J8 row-norms:', j8rowNorms,
            '| J_xi row-norms:', rowNorms);
    }
    return J_xi;
}

/**
 * Compute the 3×n angular Jacobian from the 4×n quaternion (primary) Jacobian.
 * Used for the relative-rotation preserved quantity in distance-closing mode.
 *
 *   J_omega = 2 * P3 * H4+(r^*) * J_prim
 *
 * @param {number[][]} J4  – 4×n primary (quaternion) Jacobian
 * @param {number[]}   r4  – 4-element rotation quaternion
 * @returns {number[][] | null} 3×n angular Jacobian
 */
export function computeAngularJacobian(J4, r4) {
    if (!Array.isArray(J4) || J4.length < 4 || !Array.isArray(r4) || r4.length < 4) return null;
    const n = J4[0].length;
    if (n <= 0) return null;
    const r_conj = [asFinite(r4[0]), -asFinite(r4[1]), -asFinite(r4[2]), -asFinite(r4[3])];
    const H = hamiltonLeft4(r_conj);
    const J_omega = Array.from({ length: 3 }, () => new Array(n).fill(0));
    for (let col = 0; col < n; col++) {
        for (let ri = 0; ri < 3; ri++) {
            const Hrow = H[ri + 1]; // rows 1,2,3 → imaginary parts
            let sum = 0;
            for (let k = 0; k < 4; k++) {
                sum += asFinite(Hrow[k]) * asFinite(J4[k][col]);
            }
            J_omega[ri][col] = 2 * sum;
        }
    }
    return J_omega;
}

// ─── Null-space basis (SVD-based) ────────────────────────────────────────────

/**
 * Compute an orthonormal null-space basis N of J (m×n) via SVD.
 * Returns an n×k matrix whose columns span null(J), where k = n − rank(J).
 * Returns null if J has full column rank (no null space).
 *
 * @param {number[][]} J
 * @param {number} [tol]  singular-value threshold (auto if omitted)
 * @returns {number[][] | null}
 */
let _nullSpaceLogCount = 0;
export function nullSpaceBasis(J, tol = null) {
    if (!Array.isArray(J) || J.length === 0 || !Array.isArray(J[0]) || J[0].length === 0) return null;
    const m = J.length, n = J[0].length;
    const threshold = tol ?? 1e-9;
    const N = mgsNullSpace(J, threshold);
    if (_nullSpaceLogCount++ < 2) {
        const k = N ? N[0].length : 0;
        console.log('[nullSpaceBasis] MGS', m, '×', n, '→ null dim', k);
    }
    return N;
}

// ─── CCTSM (eq. 932–943) ─────────────────────────────────────────────────────

/**
 * Compute the equality-constrained cooperative task-space manipulability.
 *
 *   J_bar = J_xi_y * N_z          (constrained commanded Jacobian, d_y × k)
 *   w_m   = sqrt(det(J_bar J_bar^T))
 *
 * @param {number[][]} J_xi_y  commanded twist Jacobian (d_y × n)
 * @param {number[][]} J_xi_z  preserved twist Jacobian (d_z × n)
 * @param {number[][]} [N_z]   optional precomputed null-space basis of J_xi_z
 * @returns {{ w: number, J_bar: number[][], N_z: number[][], Gram: number[][] } | null}
 */
let _cctsmLogCount = 0;
export function computeCCTSM(J_xi_y, J_xi_z, N_z = null) {
    if (!Array.isArray(J_xi_y) || !Array.isArray(J_xi_z)) return null;
    const Nz = N_z ?? nullSpaceBasis(J_xi_z);
    if (!Nz) return null;

    const J_bar = matMul(J_xi_y, Nz);               // d_y × k
    const Gram = matMul(J_bar, transpose(J_bar));    // d_y × d_y

    const detVal = det(Gram);
    const w = Math.sqrt(Math.max(0, asFinite(detVal, 0)));

    if (_cctsmLogCount++ < 2) {
        const jbarRowNorms = J_bar.map(r => Math.sqrt(r.reduce((s,v)=>s+v*v,0)).toExponential(3));
        const gramDiag = Gram.map((r,i) => r[i].toExponential(3));
        console.log('[computeCCTSM]',
            '| J_xi_y:', J_xi_y.length,'×',J_xi_y[0]?.length,
            '| Nz:', Nz.length,'×',Nz[0]?.length,
            '| J_bar row-norms:', jbarRowNorms,
            '| Gram diag:', gramDiag,
            '| det:', detVal.toExponential(3), '| w:', w.toExponential(3));
        console.log('[computeCCTSM] J_xi_y[0]:', J_xi_y[0]?.map(v=>v.toExponential(3)));
        console.log('[computeCCTSM] Nz col0:', Nz.map(r=>r[0]?.toExponential(3)));
    }

    return { w, J_bar, N_z: Nz, Gram };
}

/**
 * Directional CCTSM along unit direction v_hat (d_y-vector).
 *   sigma = v_hat^T * Gram * v_hat
 */
export function directionalCCTSM(Gram, vHat) {
    if (!Array.isArray(Gram) || !Array.isArray(vHat)) return 0;
    const m = Gram.length;
    let sigma = 0;
    for (let i = 0; i < m; i++) {
        let row_sum = 0;
        for (let j = 0; j < m; j++) {
            row_sum += asFinite(Gram[i][j]) * asFinite(vHat[j]);
        }
        sigma += asFinite(vHat[i]) * row_sum;
    }
    return Math.max(0, sigma);
}

// ─── VFI attenuation matrix (eq. 969) ────────────────────────────────────────

/**
 * Build the joint-space VFI attenuation matrix W_vfi (n×n).
 *
 *   W_vfi = I - sum_{i in A_vfi} (1 - omega_i) * j_i^T j_i / ||j_i||^2
 *
 * where omega_i = d_i / d_bar_i in (0,1) for near-active constraints.
 *
 * @param {Array<{row: number[], distance: number, threshold: number}>} constraints
 *   Each entry is one near-active VFI constraint with its Jacobian row (length n),
 *   current scalar distance, and activation threshold.
 * @param {number} n  joint-space dimension
 * @returns {number[][]}  n×n attenuation matrix
 */
export function computeVFIAttenuationMatrix(constraints, n) {
    // Start with identity
    const W = Array.from({ length: n }, (_, i) => {
        const row = new Array(n).fill(0);
        row[i] = 1;
        return row;
    });
    for (const c of (Array.isArray(constraints) ? constraints : [])) {
        if (!Array.isArray(c.row) || !Number.isFinite(c.distance) || !Number.isFinite(c.threshold)) continue;
        if (c.threshold <= 0) continue;
        const omega = c.distance / c.threshold;
        if (omega >= 1 || omega < 0) continue; // outside activation zone or already violated
        const baseScale = 1 - omega; // ∈ (0,1]
        const scale = baseScale;
        const jrow = c.row;
        const norm2 = jrow.reduce((s, v) => s + asFinite(v) ** 2, 0);
        if (norm2 < 1e-24) continue;
        for (let i = 0; i < n; i++) {
            for (let j = 0; j < n; j++) {
                W[i][j] -= scale * asFinite(jrow[i]) * asFinite(jrow[j]) / norm2;
            }
        }
    }
    return W;
}

// ─── Inequality-constrained CCTSM (eq. 974–982) ──────────────────────────────

/**
 * Compute the inequality-constrained CCTSM incorporating VFI attenuation.
 *
 *   G_m^vfi = J_bar * (N_z^T * W_vfi * N_z) * J_bar^T
 *   w_m^vfi = sqrt(det(G_m^vfi))
 *
 * @param {number[][]} J_xi_y       commanded twist Jacobian (d_y × n)
 * @param {number[][]} N_z          null-space basis of J_xi_z (n × k)
 * @param {number[][]} W_vfi        VFI attenuation matrix (n × n)
 * @returns {{ w_vfi: number, G_vfi: number[][] } | null}
 */
export function computeVFICCTSM(J_xi_y, N_z, W_vfi) {
    if (!Array.isArray(J_xi_y) || !Array.isArray(N_z) || !Array.isArray(W_vfi)) return null;

    const J_bar = matMul(J_xi_y, N_z);                   // d_y × k
    const NzT = transpose(N_z);                           // k × n
    const NzT_W = matMul(NzT, W_vfi);                    // k × n
    const M = matMul(NzT_W, N_z);                         // k × k  (= N_z^T W N_z)
    const G_vfi = matMul(matMul(J_bar, M), transpose(J_bar)); // d_y × d_y

    const w_vfi = Math.sqrt(Math.max(0, asFinite(det(G_vfi), 0)));

    return { w_vfi, G_vfi };
}

/**
 * Combined-task CCTSM: stacks commanded (J_xi_y) and constraint (J_xi_z) Jacobians
 * so that both tasks contribute to the manipulability measure.
 *
 *   J_full = [ J_xi_y ]   ((d_y + d_z) × n)
 *            [ J_xi_z ]
 *   Gram   = J_full J_full^T
 *   w      = sqrt(det(Gram))
 *
 * No null-space projection is used; the full 12×n Jacobian directly represents
 * the combined velocity mapping of absolute and relative poses.
 */
export function computeCombinedCCTSM(J_xi_y, J_xi_z) {
    if (!Array.isArray(J_xi_y) || !Array.isArray(J_xi_z)) return null;

    const J_full = [...J_xi_y, ...J_xi_z];          // (d_y+d_z) × n
    const Gram = matMul(J_full, transpose(J_full));  // (d_y+d_z) × (d_y+d_z)
    const w = Math.sqrt(Math.max(0, asFinite(det(Gram), 0)));

    return { w, J_bar: null, N_z: null, Gram, J_full };
}

/**
 * VFI-attenuated combined-task CCTSM.
 *
 *   G_vfi = J_full * W_vfi * J_full^T   ((d_y+d_z) × (d_y+d_z))
 *   w_vfi = sqrt(det(G_vfi))
 */
export function computeCombinedVFICCTSM(J_full, W_vfi) {
    if (!Array.isArray(J_full) || !Array.isArray(W_vfi)) return null;
    const G_vfi = matMul(matMul(J_full, W_vfi), transpose(J_full));
    const w_vfi = Math.sqrt(Math.max(0, asFinite(det(G_vfi), 0)));
    return { w_vfi, G_vfi };
}

/**
 * Directional inequality-constrained CCTSM along unit direction v_hat.
 *   sigma^vfi = v_hat^T * G_vfi * v_hat
 */
export function directionalVFICCTSM(G_vfi, vHat) {
    return directionalCCTSM(G_vfi, vHat);
}

/**
 * Worst-case (isotropic) CCTSM: minimum eigenvalue of a symmetric PSD matrix G.
 *   sigma_m(Theta) = lambda_min(G)
 *
 * Equals the squared radius of the largest ball fitting inside the velocity
 * ellipsoid. Maximising this avoids singularity in every direction simultaneously.
 *
 * Uses power iteration on (lambda_max * I - G) to find the direction of minimum
 * gain, then a Rayleigh quotient for the final value.
 *
 * @param {number[][]} G  d_y × d_y symmetric PSD matrix (Gram or G_vfi)
 * @returns {number}      lambda_min(G), clamped to [0, ∞)
 */
export function minEigenvalueCCTSM(G) {
    if (!Array.isArray(G) || G.length === 0) return 0;
    const n = G.length;

    // Power iteration for λ_max
    let v = new Array(n).fill(1 / Math.sqrt(n));
    let lambdaMax = 0;
    for (let k = 0; k < 24; k++) {
        const Gv = new Array(n).fill(0);
        for (let i = 0; i < n; i++)
            for (let j = 0; j < n; j++) Gv[i] += asFinite(G[i][j]) * v[j];
        const norm = Math.sqrt(Gv.reduce((s, x) => s + x * x, 0));
        if (norm < 1e-14) return 0;
        lambdaMax = 0;
        for (let i = 0; i < n; i++) { v[i] = Gv[i] / norm; lambdaMax += v[i] * Gv[i]; }
    }

    // Power iteration on (λ_max*I − G) to find the min-eigenvalue direction
    let u = new Array(n).fill(0); u[0] = 1;
    for (let k = 0; k < 24; k++) {
        const Gu = new Array(n).fill(0);
        for (let i = 0; i < n; i++) {
            Gu[i] = lambdaMax * u[i];
            for (let j = 0; j < n; j++) Gu[i] -= asFinite(G[i][j]) * u[j];
        }
        const norm = Math.sqrt(Gu.reduce((s, x) => s + x * x, 0));
        if (norm < 1e-14) break;
        for (let i = 0; i < n; i++) u[i] = Gu[i] / norm;
    }

    // Rayleigh quotient: u^T G u
    let sigma = 0;
    for (let i = 0; i < n; i++) {
        let row = 0;
        for (let j = 0; j < n; j++) row += asFinite(G[i][j]) * u[j];
        sigma += u[i] * row;
    }
    return Math.max(0, sigma);
}

// ─── Gradient of VFI-CCTSM (for branch seed generation, eq. 1267) ────────────

/**
 * Numerically estimate the gradient of w_m^vfi with respect to active joints.
 * Uses central differences on computeVFICCTSM with perturbed joint configs.
 *
 * @param {function(number[]): {w_vfi: number} | null} evalFn
 *   Function that takes joint config and returns CCTSM result.
 * @param {number[]} q       current joint config (length n_active)
 * @param {number}  [eps]    finite-difference step size
 * @returns {number[]}  gradient vector (length n_active), zeros on failure
 */
export function gradientVFICCTSM(evalFn, q, eps = 1e-5) {
    const n = Array.isArray(q) ? q.length : 0;
    const grad = new Array(n).fill(0);
    if (n === 0 || typeof evalFn !== 'function') return grad;

    for (let j = 0; j < n; j++) {
        const qp = q.slice(); qp[j] += eps;
        const qm = q.slice(); qm[j] -= eps;
        const rp = evalFn(qp);
        const rm = evalFn(qm);
        if (rp && rm) {
            grad[j] = (asFinite(rp.w_vfi, 0) - asFinite(rm.w_vfi, 0)) / (2 * eps);
        }
    }
    return grad;
}
