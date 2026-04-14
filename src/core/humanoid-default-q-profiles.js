export const DEFAULT_HUMANOID_INITIAL_Q = Object.freeze([
    -0.3, 0.9, -0.6, 0,
    -0.52603, -0.0244, 0.00474, -1.04101, 0.0621, 0.09207, 0.00193,
    0.52604, 0.02439, -0.00477, 1.04099, -0.06206, -0.09196, -0.00193,
    0, 0
]);

export const DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2 = Object.freeze([
    -0.3, 0.9, -0.6, 0,
    -0.5427, 0.67517, 0.99198, -1.63849, -0.89152, 0.84762, -0.13232,
    0.54379, -0.67584, -0.99304, 1.63814, 0.89344, -0.84737, 0.13325,
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
