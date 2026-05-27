export const DEFAULT_HUMANOID_INITIAL_Q = Object.freeze([
    -0.3, 0.9, -0.6, 0,
    -0.5236, 0, 0, -1.0471, 0, 0, 0,
    0.5236, 0, 0, 1.0471, 0, 0, 0,
    0, 0
]);

export const DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2 = Object.freeze([
    -0.3, 0.9, -0.6, 0,
    -0.51865, 0.66105, 0.90855, -1.5711, -0.93562, 0.67402, 1.40137,
    0.56421, -0.68286, -0.98744, 1.58165, 1.00933, -0.71114, -1.35432,
    0, 0
]);

export const DEFAULT_HUMANOID_INITIAL_Q_PROFILE_ID = 'default1';

export const DEFAULT_HUMANOID_INITIAL_Q_PROFILES = Object.freeze({
    default1: DEFAULT_HUMANOID_INITIAL_Q,
    default2: DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2
});

export function normalizeHumanoidInitialQProfileId(profileIdRaw) {
    return profileIdRaw === 'default2' ? 'default2' : DEFAULT_HUMANOID_INITIAL_Q_PROFILE_ID;
}

export function resolveHumanoidInitialQ(config = {}, jointCount = 0, profileIdRaw = DEFAULT_HUMANOID_INITIAL_Q_PROFILE_ID) {
    const resolvedJointCount = Math.max(0, Math.floor(Number(jointCount) || 0));
    if (Array.isArray(config?.initial_q) && config.initial_q.length === resolvedJointCount) {
        return config.initial_q.slice();
    }
    const fallback = new Array(resolvedJointCount).fill(0);
    const profileId = normalizeHumanoidInitialQProfileId(config?.initial_q_profile ?? profileIdRaw);
    const source = DEFAULT_HUMANOID_INITIAL_Q_PROFILES[profileId];
    const limit = Math.min(resolvedJointCount, source.length);
    for (let index = 0; index < limit; index++) {
        fallback[index] = source[index];
    }
    return fallback;
}
