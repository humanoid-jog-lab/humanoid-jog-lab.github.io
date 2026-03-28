import test from 'node:test';
import assert from 'node:assert/strict';

import { HumanoidJogMJWorkerClient } from './humanoid-jog-mj-worker-client.js';

class FakeWorker {
    constructor() {
        this.messages = [];
        this.onmessage = null;
        this.onerror = null;
        this.terminated = false;
    }

    postMessage(message) {
        this.messages.push(message);
    }

    terminate() {
        this.terminated = true;
    }

    replyOk(id, payload = null) {
        this.onmessage?.({ data: { id, ok: true, payload } });
    }

    replyError(id, error = 'worker_error') {
        this.onmessage?.({ data: { id, ok: false, error } });
    }
}

test('HumanoidJogMJWorkerClient initializes a model once and reuses it for solve', async () => {
    const worker = new FakeWorker();
    const client = new HumanoidJogMJWorkerClient({
        workerFactory: () => worker,
        baseUrl: '/app'
    });
    const modelSpec = { baseDQ: [1, 0, 0, 0, 0, 0, 0, 0], waistChain: [] };
    const request = { qCurrent: [0, 0], absoluteTarget: new Array(8).fill(0) };

    const solvePromise = client.solve({ modelSpec, request });
    assert.equal(worker.messages.length, 1);
    assert.deepEqual(worker.messages[0], {
        id: 1,
        type: 'init',
        payload: {
            baseUrl: '/app/',
            modelSpec
        }
    });

    worker.replyOk(1, { initialized: true });
    await new Promise((resolve) => setTimeout(resolve, 0));

    assert.equal(worker.messages.length, 2);
    assert.deepEqual(worker.messages[1], {
        id: 2,
        type: 'solve',
        payload: {
            request,
            secondaryMetricRequest: null
        }
    });

    worker.replyOk(2, { converged: true, solveMs: 1.25 });
    const result = await solvePromise;
    assert.equal(result.converged, true);
    assert.equal(result.solveMs, 1.25);
    assert.ok(Number.isFinite(result.workerInitMs));
    assert.ok(Number.isFinite(result.solveRoundTripMs));
    assert.ok(result.workerInitMs >= 0);
    assert.ok(result.solveRoundTripMs >= 0);

    const secondSolvePromise = client.solve({ modelSpec, request: { qCurrent: [1, 2] } });
    await new Promise((resolve) => setTimeout(resolve, 0));
    assert.equal(worker.messages.length, 3);
    assert.deepEqual(worker.messages[2], {
        id: 3,
        type: 'solve',
        payload: {
            request: { qCurrent: [1, 2] },
            secondaryMetricRequest: null
        }
    });
    worker.replyOk(3, { converged: false, solveMs: 0.75 });
    const secondResult = await secondSolvePromise;
    assert.equal(secondResult.converged, false);
    assert.equal(secondResult.solveMs, 0.75);
    assert.ok(Number.isFinite(secondResult.workerInitMs));
    assert.ok(Number.isFinite(secondResult.solveRoundTripMs));
});

test('HumanoidJogMJWorkerClient reinitializes when the model spec changes', async () => {
    const worker = new FakeWorker();
    const client = new HumanoidJogMJWorkerClient({
        workerFactory: () => worker
    });

    const firstEnsure = client.ensureModel({ waistChain: [1] });
    assert.equal(worker.messages.length, 1);
    worker.replyOk(1, {});
    await firstEnsure;

    const secondEnsure = client.ensureModel({ waistChain: [2] });
    assert.equal(worker.messages.length, 2);
    assert.equal(worker.messages[1].type, 'init');
    worker.replyOk(2, {});
    await secondEnsure;
});

test('HumanoidJogMJWorkerClient rejects pending work when closed', async () => {
    const worker = new FakeWorker();
    const client = new HumanoidJogMJWorkerClient({
        workerFactory: () => worker
    });

    const promise = client.ensureModel({ waistChain: [1] });
    client.close();

    await assert.rejects(promise, /worker_closed/);
    assert.equal(worker.terminated, true);
});

test('HumanoidJogMJWorkerClient appends a worker version token to the default worker URL', () => {
    const createdUrls = [];
    const OriginalWorker = globalThis.Worker;

    class CaptureWorker extends FakeWorker {
        constructor(url) {
            super();
            this.url = url;
            createdUrls.push(url);
        }
    }

    globalThis.Worker = CaptureWorker;
    try {
        const client = new HumanoidJogMJWorkerClient({
            baseUrl: '/app',
            workerScriptVersion: 'build-123'
        });
        client._ensureWorker();
        assert.deepEqual(createdUrls, ['/app/workers/humanoid-jog-mj-worker.js?v=build-123']);
    } finally {
        if (OriginalWorker === undefined) {
            delete globalThis.Worker;
        } else {
            globalThis.Worker = OriginalWorker;
        }
    }
});
