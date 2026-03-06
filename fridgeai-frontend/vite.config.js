import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/items':  'http://localhost:8000',
      '/alerts': 'http://localhost:8000',
      '/status': 'http://localhost:8000',
      '/lookup': 'http://localhost:8000',
      '/vision': 'http://localhost:8000',
      '/ws':     { target: 'ws://localhost:8000', ws: true, rewriteWsOrigin: true },
    }
  }
})
