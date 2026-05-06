import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '..');

function createFakeTextElement(initialValue = '') {
    return {
        textContent: initialValue,
        innerHTML: initialValue,
        placeholder: '',
        title: '',
        value: '',
        dataset: {},
        listeners: new Map(),
        addEventListener(type, handler) {
            this.listeners.set(type, handler);
        }
    };
}

function createFakeStorage(seed = {}) {
    const store = new Map(Object.entries(seed));
    return {
        getItem(key) {
            return store.has(key) ? store.get(key) : null;
        },
        setItem(key, value) {
            store.set(key, String(value));
        },
        removeItem(key) {
            store.delete(key);
        }
    };
}

function createFakeDocument(selectorMap = {}, elementById = {}) {
    return {
        title: '',
        documentElement: { lang: '' },
        querySelectorAll(selector) {
            const value = selectorMap[selector];
            if (!value) return [];
            return Array.isArray(value) ? value : [value];
        },
        querySelector(selector) {
            const value = selectorMap[selector];
            if (Array.isArray(value)) return value[0] || null;
            return value || null;
        },
        getElementById(id) {
            return elementById[id] || null;
        }
    };
}

test('app shell defaults to English and exposes a language switcher', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');
    assert.match(html, /<html lang="en">/);
    assert.match(html, /id="language-select"/);
    assert.match(html, />Control Panel</);
});

test('i18n defaults to English, persists language choice, and restores Chinese', async () => {
    const loadingText = createFakeTextElement();
    const ikInput = createFakeTextElement();
    const languageSelect = createFakeTextElement();
    const document = createFakeDocument(
        {
            '#loading p': loadingText,
            '#ik-dq-input': ikInput
        },
        {
            'language-select': languageSelect
        }
    );
    const storage = createFakeStorage();

    const i18n = await import('./i18n.js');
    i18n.initializeI18n({ document, storage });

    assert.equal(i18n.DEFAULT_LANGUAGE, 'en');
    assert.equal(i18n.getCurrentLanguage(), 'en');
    assert.equal(document.documentElement.lang, 'en');
    assert.equal(document.title, 'DQ Robotics Visualizer');
    assert.equal(loadingText.textContent, 'Loading WASM module...');
    assert.equal(ikInput.placeholder, "qw qx qy qz qw' qx' qy' qz'");
    assert.equal(languageSelect.value, 'en');

    i18n.setLanguage('zh', { document, storage });
    assert.equal(storage.getItem('ui-language'), 'zh');
    assert.equal(document.documentElement.lang, 'zh-CN');
    assert.equal(document.title, 'DQ Robotics 可视化');
    assert.equal(loadingText.textContent, '正在加载 WASM 模块...');
    assert.equal(languageSelect.value, 'zh');

    const restoredLoadingText = createFakeTextElement();
    const restoredIkInput = createFakeTextElement();
    const restoredLanguageSelect = createFakeTextElement();
    const restoredDocument = createFakeDocument(
        {
            '#loading p': restoredLoadingText,
            '#ik-dq-input': restoredIkInput
        },
        {
            'language-select': restoredLanguageSelect
        }
    );

    i18n.initializeI18n({ document: restoredDocument, storage });
    assert.equal(i18n.getCurrentLanguage(), 'zh');
    assert.equal(restoredDocument.documentElement.lang, 'zh-CN');
    assert.equal(restoredDocument.title, 'DQ Robotics 可视化');
    assert.equal(restoredLoadingText.textContent, '正在加载 WASM 模块...');
    assert.equal(restoredLanguageSelect.value, 'zh');
});
