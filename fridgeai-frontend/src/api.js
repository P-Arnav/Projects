// REST helpers — all relative paths, proxied by Vite to localhost:8000
export const api = {
  getItems: (since) =>
    fetch(since ? `/items?updated_since=${encodeURIComponent(since)}` : '/items')
      .then(r => r.json()),

  postItem: (body) =>
    fetch('/items', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    }).then(r => { if (!r.ok) throw r; return r.json() }),

  deleteItem: (id, reason = 'consumed') =>
    fetch(`/items/${id}?reason=${reason}`, { method: 'DELETE' }),

  patchItem: (id, body) =>
    fetch(`/items/${id}`, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    }).then(r => r.json()),

  getAlerts: (since) =>
    fetch(since ? `/alerts?since=${encodeURIComponent(since)}` : '/alerts')
      .then(r => r.json()),

  getStatus: () => fetch('/status').then(r => r.json()),

  scanFridge: (blob) => {
    const fd = new FormData()
    fd.append('file', blob, 'scan.jpg')
    return fetch('/vision/scan', { method: 'POST', body: fd })
      .then(r => { if (!r.ok) throw r; return r.json() })
  },

  lookupBarcode: (barcode) =>
    fetch(`/lookup/barcode/${encodeURIComponent(barcode)}`).then(r => r.json()),

  getShelfLife: (category) =>
    fetch(`/lookup/shelf-life/${encodeURIComponent(category)}`).then(r => r.json()),
}

// WebSocket singleton with auto-reconnect
export function createWsClient(dispatch) {
  let ws = null
  let dead = false

  function connect() {
    if (dead) return
    const proto = location.protocol === 'https:' ? 'wss' : 'ws'
    ws = new WebSocket(`${proto}://${location.host}/ws?client_type=web`)

    ws.onopen = () => dispatch({ type: 'WS_STATUS', status: 'connected' })

    ws.onclose = () => {
      dispatch({ type: 'WS_STATUS', status: 'disconnected' })
      if (!dead) setTimeout(connect, 3000)
    }

    ws.onerror = () => ws.close()

    ws.onmessage = (e) => {
      try {
        dispatch({ type: 'WS_MESSAGE', msg: JSON.parse(e.data) })
      } catch (_) {}
    }
  }

  dispatch({ type: 'WS_STATUS', status: 'connecting' })
  connect()

  return () => { dead = true; ws?.close() }
}
