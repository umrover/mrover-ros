import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { nodePolyfills } from 'vite-plugin-node-polyfills'


// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    vue(),
    // BSON package nonsense
    nodePolyfills(),
  ],
  server: {
    host: 'localhost',
    port: 8080
  },
  // More BSON package nonsense
  define: {
    global: {},
  },
})