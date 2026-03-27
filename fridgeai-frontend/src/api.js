// In production, set VITE_API_URL to the deployed backend (e.g. https://fridgeai.railway.app)
// In dev, leave unset — Vite proxy forwards to localhost:8000
const BASE = import.meta.env.VITE_API_URL ?? ''

function authHeader() {
  const token = localStorage.getItem('fridge_token')
  return token ? { Authorization: `Bearer ${token}` } : {}
}

function jsonHeaders() {
  return { 'Content-Type': 'application/json', ...authHeader() }
}

// REST helpers
export const api = {
  getItems: (since) =>
    fetch(since ? `${BASE}/items?updated_since=${encodeURIComponent(since)}` : `${BASE}/items`,
      { headers: authHeader() }).then(r => r.json()),

  postItem: (body) =>
    fetch(`${BASE}/items`, {
      method: 'POST',
      headers: jsonHeaders(),
      body: JSON.stringify(body),
    }).then(r => { if (!r.ok) throw r; return r.json() }),

  deleteItem: (id, reason = 'consumed') =>
    fetch(`${BASE}/items/${id}?reason=${reason}`, { method: 'DELETE', headers: authHeader() }),

  patchItem: (id, body) =>
    fetch(`${BASE}/items/${id}`, {
      method: 'PATCH',
      headers: jsonHeaders(),
      body: JSON.stringify(body),
    }).then(r => r.json()),

  getAlerts: (since) =>
    fetch(since ? `${BASE}/alerts?since=${encodeURIComponent(since)}` : `${BASE}/alerts`,
      { headers: authHeader() }).then(r => r.json()),

  getStatus: () => fetch(`${BASE}/status`, { headers: authHeader() }).then(r => r.json()),

  scanFridge: (blob) => {
    const fd = new FormData()
    fd.append('file', blob, 'scan.jpg')
    return fetch(`${BASE}/vision/scan`, { method: 'POST', body: fd, headers: authHeader() })
      .then(r => { if (!r.ok) throw r; return r.json() })
  },

  lookupBarcode: (barcode) =>
    fetch(`${BASE}/lookup/barcode/${encodeURIComponent(barcode)}`, { headers: authHeader() }).then(r => r.json()),

  getShelfLife: (category) =>
    fetch(`${BASE}/lookup/shelf-life/${encodeURIComponent(category)}`, { headers: authHeader() }).then(r => r.json()),

  lookupItem: (name) =>
    fetch(`${BASE}/lookup/item/${encodeURIComponent(name)}`, { headers: authHeader() })
      .then(r => { if (!r.ok) throw r; return r.json() }),

  // Grocery list
  getGrocery: () =>
    fetch(`${BASE}/grocery`, { headers: authHeader() }).then(r => r.json()),

  addGrocery: (body) =>
    fetch(`${BASE}/grocery`, {
      method: 'POST',
      headers: jsonHeaders(),
      body: JSON.stringify(body),
    }).then(r => { if (!r.ok) throw r; return r.json() }),

  updateGrocery: (id, body) =>
    fetch(`${BASE}/grocery/${id}`, {
      method: 'PATCH',
      headers: jsonHeaders(),
      body: JSON.stringify(body),
    }).then(r => r.json()),

  deleteGrocery: (id) =>
    fetch(`${BASE}/grocery/${id}`, { method: 'DELETE', headers: authHeader() }),

  clearCheckedGrocery: () =>
    fetch(`${BASE}/grocery/checked`, { method: 'DELETE', headers: authHeader() }),

  // Restock suggestions
  getRestock: () =>
    fetch(`${BASE}/restock`, { headers: authHeader() }).then(r => r.json()),

  // Recipes
  getRecipes: () =>
    fetch(`${BASE}/recipes/suggestions`, { headers: authHeader() })
      .then(r => { if (!r.ok) throw r; return r.json() }),

  getRecipeDetails: (mealId) =>
    fetch(`${BASE}/recipes/${mealId}/details`, { headers: authHeader() })
      .then(r => { if (!r.ok) throw r; return r.json() }),

  cookRecipe: (mealId, itemIds) =>
    fetch(`${BASE}/recipes/${mealId}/cook`, {
      method: 'POST',
      headers: jsonHeaders(),
      body: JSON.stringify({ item_ids: itemIds }),
    }).then(r => r.json()),

  // Adaptive feedback
  submitFeedback: (id, body) =>
    fetch(`${BASE}/items/${id}/feedback`, {
      method: 'POST',
      headers: jsonHeaders(),
      body: JSON.stringify(body),
    }).then(r => { if (!r.ok) throw r; return r.json() }),

  // Receipt OCR
  parseReceiptText: (text) =>
    fetch(`${BASE}/receipt/parse-text`, {
      method: 'POST',
      headers: jsonHeaders(),
      body: JSON.stringify({ text }),
    }).then(r => { if (!r.ok) throw r; return r.json() }),

  scanReceipt: (file) => {
    const fd = new FormData()
    fd.append('file', file, file.name)
    return fetch(`${BASE}/receipt/scan`, { method: 'POST', body: fd, headers: authHeader() })
      .then(r => { if (!r.ok) throw r; return r.json() })
  },

  // Auth
  authConfig: () =>
    fetch(`${BASE}/auth/config`).then(r => r.json()),

  getMe: () =>
    fetch(`${BASE}/auth/me`, { headers: authHeader() })
      .then(r => { if (!r.ok) throw r; return r.json() }),

  getPrefs: () =>
    fetch(`${BASE}/auth/prefs`, { headers: authHeader() }).then(r => r.json()),

  updatePrefs: (body) =>
    fetch(`${BASE}/auth/prefs`, {
      method: 'PATCH',
      headers: jsonHeaders(),
      body: JSON.stringify(body),
    }).then(r => r.json()),

  // Analytics
  getConsumptionTrend: (days = 30) =>
    fetch(`${BASE}/analytics/consumption?days=${days}`, { headers: authHeader() }).then(r => r.json()),

  getWastePatterns: () =>
    fetch(`${BASE}/analytics/waste-patterns`, { headers: authHeader() }).then(r => r.json()),

  getAnalyticsSummary: (days = 30) =>
    fetch(`${BASE}/analytics/summary?days=${days}`, { headers: authHeader() }).then(r => r.json()),

  getConsumptionPredictions: () =>
    fetch(`${BASE}/analytics/predictions`, { headers: authHeader() }).then(r => r.json()),

  addGroceryToFridge: (grocery_id) =>
    fetch(`${BASE}/grocery/${grocery_id}/add-to-fridge`, {
      method: 'POST', headers: authHeader(),
    }).then(r => r.json()),
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
    const token = localStorage.getItem('fridge_token')
    const url = token
      ? `${wsBase}/ws?client_type=web&token=${encodeURIComponent(token)}`
      : `${wsBase}/ws?client_type=web`
    ws = new WebSocket(url)

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
