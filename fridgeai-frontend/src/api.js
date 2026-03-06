// In production, set VITE_API_URL to the deployed backend (e.g. https://fridgeai.railway.app)
// In dev, leave unset — Vite proxy forwards to localhost:8000
const BASE = import.meta.env.VITE_API_URL ?? ''

// REST helpers
export const api = {
  getItems: (since) =>
    fetch(since ? `${BASE}/items?updated_since=${encodeURIComponent(since)}` : `${BASE}/items`)
      .then(r => r.json()),

  postItem: (body) =>
    fetch(`${BASE}/items`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    }).then(r => { if (!r.ok) throw r; return r.json() }),

  deleteItem: (id, reason = 'consumed') =>
    fetch(`${BASE}/items/${id}?reason=${reason}`, { method: 'DELETE' }),

  patchItem: (id, body) =>
    fetch(`${BASE}/items/${id}`, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    }).then(r => r.json()),

  getAlerts: (since) =>
    fetch(since ? `${BASE}/alerts?since=${encodeURIComponent(since)}` : `${BASE}/alerts`)
      .then(r => r.json()),

  getStatus: () => fetch(`${BASE}/status`).then(r => r.json()),

  scanFridge: (blob) => {
    const fd = new FormData()
    fd.append('file', blob, 'scan.jpg')
    return fetch(`${BASE}/vision/scan`, { method: 'POST', body: fd })
      .then(r => { if (!r.ok) throw r; return r.json() })
  },

  lookupBarcode: (barcode) =>
    fetch(`${BASE}/lookup/barcode/${encodeURIComponent(barcode)}`).then(r => r.json()),

  getShelfLife: (category) =>
    fetch(`${BASE}/lookup/shelf-life/${encodeURIComponent(category)}`).then(r => r.json()),

  lookupItem: (name) =>
    fetch(`${BASE}/lookup/item/${encodeURIComponent(name)}`).then(r => { if (!r.ok) throw r; return r.json() }),
}

// WebSocket singleton with auto-reconnect
export function createWsClient(dispatch) {
  let ws = null
  let dead = false

  function connect() {
    if (dead) return
    const wsBase = BASE
      ? BASE.replace(/^http/, 'ws')
      : `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}`
    ws = new WebSocket(`${wsBase}/ws?client_type=web`)

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
