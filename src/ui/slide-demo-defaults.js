export const SLIDE_DEMO_STORAGE_DEFAULTS = Object.freeze({
    'humanoid-default-q-profile': 'default1',
    'humanoid-jog-solver-mode': 'proxqp_w_vfi',
    'humanoid-jog-default-version': 'proxqp-three-solver',
    'humanoid-manifold-jog-metric': 'w_abs_hier_vfi',
    'humanoid-waist-x-corridor-enabled': '1',
    'humanoid-waist-x-corridor-min': '-0.4',
    'humanoid-waist-x-corridor-max': '-0.1',
    'humanoid-test-dual-arm-lock-enabled': '0',
    'teach-speed-percent': '25',
    'teach-multiply-order-delta-first': '0',
    'humanoid-cooperative-polishing-surface-source': 'cylindrical-patch'
});

export function isSlideDemoLaunch(search = '') {
    const rawSearch = typeof search === 'string'
        ? search
        : (typeof window !== 'undefined' ? window.location.search : '');
    const normalizedSearch = rawSearch.startsWith('?') ? rawSearch : `?${rawSearch}`;
    const params = new URLSearchParams(normalizedSearch);
    return params.get('slidevDemo') === '1';
}

export function applySlideDemoStorageDefaults(options = {}) {
    const {
        search = typeof window !== 'undefined' ? window.location.search : '',
        storage = typeof localStorage !== 'undefined' ? localStorage : null
    } = options;
    if (!isSlideDemoLaunch(search) || !storage) return false;
    try {
        Object.entries(SLIDE_DEMO_STORAGE_DEFAULTS).forEach(([key, value]) => {
            storage.setItem(key, value);
        });
    } catch (_) {
        return false;
    }
    if (typeof window !== 'undefined') {
        window.__humanoidJogSlideDemo = true;
    }
    return true;
}
