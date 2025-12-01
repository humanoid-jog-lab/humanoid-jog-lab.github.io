import { defineConfig } from 'vite';

const isProd = process.env.NODE_ENV === 'production';
const isElectron = process.env.VITE_DEV_SERVER_URL?.includes('electron');

export default defineConfig({
  // Electron 应用使用 / 路径，GitHub Pages 使用 /dq_vizlab/
  base: isElectron || !isProd ? '/' : '/dq_vizlab/',
  server: {
    port: 5173,
    open: !isElectron, // Electron 模式下不自动打开浏览器
    headers: {
      // 确保 WASM 文件可以被正确加载
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp'
    }
  },
  build: {
    target: 'esnext',
    outDir: 'dist',
    emptyOutDir: true
  },
  publicDir: 'public',
  assetsInclude: ['**/*.wasm']
});

