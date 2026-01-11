import { defineConfig } from 'vite';

// 构建目标：
// - 默认本地分发 dist/（静态服务器根目录）：base 必须是相对路径 ./，否则会去根路径找 /xxx/assets 导致资源 404
// - Dev Server：base = /
// - GitHub Pages（或任何子路径部署）：通过 VITE_BASE 显式指定，如 /dq_vizlab/
const isElectronBuild = process.env.VITE_TARGET === 'electron';

export default defineConfig(({ command }) => {
  const isBuild = command === 'build';
  const base =
    process.env.VITE_BASE ??
    (isElectronBuild ? './' : (isBuild ? './' : '/'));

  return {
  base,
  server: {
    port: 5173,
    open: true,
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
  };
});

