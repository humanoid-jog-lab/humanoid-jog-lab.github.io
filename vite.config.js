import { defineConfig } from 'vite';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

// 构建目标：
// - 默认本地分发 dist/（静态服务器根目录）：base 必须是相对路径 ./，否则会去根路径找 /xxx/assets 导致资源 404
// - Dev Server：base = /
// - GitHub Pages（或任何子路径部署）：通过 VITE_BASE 显式指定，如 /dq_vizlab/

export default defineConfig(({ command }) => {
  const isBuild = command === 'build';
  const base = process.env.VITE_BASE ?? (isBuild ? './' : '/');
  const rootDir = fileURLToPath(new URL('.', import.meta.url));

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
    emptyOutDir: true,
    chunkSizeWarningLimit: 550,
    rollupOptions: {
      input: {
        main: resolve(rootDir, 'index.html'),
        jointMonitor: resolve(rootDir, 'joint-monitor.html'),
      },
      output: {
        manualChunks(id) {
          if (!id.includes('node_modules')) return;
          if (id.includes('/node_modules/three/examples/')) return 'three-examples';
          if (id.includes('/node_modules/three-stdlib/')) return 'three-stdlib';
          if (id.includes('/node_modules/three/src/renderers/')) return 'three-renderers';
          if (id.includes('/node_modules/three/')) return 'three-core';
          if (id.includes('/node_modules/urdf-loader/')) return 'urdf-loader';
          if (id.includes('/node_modules/js-yaml/')) return 'js-yaml';
          return 'vendor';
        }
      }
    }
  },
  publicDir: 'public',
  assetsInclude: ['**/*.wasm']
  };
});
