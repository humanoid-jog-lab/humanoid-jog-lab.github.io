function defaultBaseUrl() {
    return typeof import.meta !== 'undefined' && import.meta?.env?.BASE_URL
        ? import.meta.env.BASE_URL
        : '/';
}

function normalizeBaseUrl(baseUrl) {
    const raw = typeof baseUrl === 'string' && baseUrl.length > 0 ? baseUrl : '/';
    return raw.endsWith('/') ? raw : `${raw}/`;
}

function normalizeWorkerScriptVersion(workerScriptVersion) {
    if (typeof workerScriptVersion !== 'string') return '';
    const trimmed = workerScriptVersion.trim();
    return trimmed.length > 0 ? trimmed : '';
}

function stableModelKey(modelSpec) {
    return JSON.stringify(modelSpec ?? null);
}

function nowMs() {
    if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
        return performance.now();
    }
    return Date.now();
}

export class HumanoidJogMJWorkerClient {
    constructor({
        workerFactory = null,
        baseUrl = defaultBaseUrl(),
        workerScriptVersion = ''
    } = {}) {
        this._workerFactory = workerFactory;
        this._baseUrl = normalizeBaseUrl(baseUrl);
        this._workerScriptVersion = normalizeWorkerScriptVersion(workerScriptVersion);
        this._worker = null;
        this._nextId = 1;
        this._pending = new Map();
        this._readyModelKey = '';
        this._readyPromise = null;
    }

    _buildWorkerUrl() {
        const workerPath = `${this._baseUrl}workers/humanoid-jog-mj-worker.js`;
        if (!this._workerScriptVersion) return workerPath;
        return `${workerPath}?v=${encodeURIComponent(this._workerScriptVersion)}`;
    }

    _createDefaultWorker() {
        if (typeof Worker !== 'function') return null;
        return new Worker(this._buildWorkerUrl());
    }

    _ensureWorker() {
        if (this._worker) return this._worker;
        const worker = typeof this._workerFactory === 'function'
            ? this._workerFactory()
            : this._createDefaultWorker();
        if (!worker) return null;
        worker.onmessage = (event) => {
            const data = event?.data ?? {};
            const pending = this._pending.get(data.id);
            if (!pending) return;
            this._pending.delete(data.id);
            if (data.ok === true) {
                pending.resolve(data.payload ?? null);
            } else {
                pending.reject(new Error(typeof data.error === 'string' ? data.error : 'worker_error'));
            }
        };
        worker.onerror = (event) => {
            const error = new Error(event?.message || 'worker_error');
            for (const pending of this._pending.values()) {
                pending.reject(error);
            }
            this._pending.clear();
        };
        this._worker = worker;
        return worker;
    }

    _post(type, payload) {
        const worker = this._ensureWorker();
        if (!worker) return Promise.resolve(null);
        const id = this._nextId++;
        return new Promise((resolve, reject) => {
            this._pending.set(id, { resolve, reject });
            worker.postMessage({ id, type, payload });
        });
    }

    async ensureModel(modelSpec) {
        const modelKey = stableModelKey(modelSpec);
        if (!modelKey || modelKey === 'null') return false;
        if (this._readyModelKey === modelKey) return true;
        if (this._readyPromise && this._readyPromise.modelKey === modelKey) {
            await this._readyPromise.promise;
            return true;
        }
        const promise = this._post('init', {
            baseUrl: this._baseUrl,
            modelSpec
        });
        this._readyPromise = { modelKey, promise };
        await promise;
        this._readyModelKey = modelKey;
        return true;
    }

    async solve({ modelSpec, solverMode = 'mj_restore', request, secondaryMetricRequest = null } = {}) {
        const worker = this._ensureWorker();
        if (!worker || !modelSpec || !request) return null;
        const ensureStartMs = nowMs();
        await this.ensureModel(modelSpec);
        const workerInitMs = nowMs() - ensureStartMs;
        const solveRoundTripStartMs = nowMs();
        const payload = await this._post('solve', { solverMode, request, secondaryMetricRequest });
        const solveRoundTripMs = nowMs() - solveRoundTripStartMs;
        if (payload && typeof payload === 'object') {
            return {
                ...payload,
                workerInitMs,
                solveRoundTripMs
            };
        }
        return {
            workerInitMs,
            solveRoundTripMs
        };
    }

    close() {
        if (this._worker && typeof this._worker.terminate === 'function') {
            this._worker.terminate();
        }
        this._worker = null;
        for (const pending of this._pending.values()) {
            pending.reject(new Error('worker_closed'));
        }
        this._pending.clear();
        this._readyModelKey = '';
        this._readyPromise = null;
    }
}
