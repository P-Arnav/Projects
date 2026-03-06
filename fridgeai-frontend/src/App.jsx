import { useReducer, useEffect, useRef } from 'react'
import { C } from './constants.js'
import { api, createWsClient } from './api.js'
import Inventory from './views/Inventory.jsx'
import Alerts from './views/Alerts.jsx'
import Analytics from './views/Analytics.jsx'
import AlertBanner from './components/AlertBanner.jsx'

// ─── Reducer ────────────────────────────────────────────────────────────────

const initial = {
  items: [],
  alerts: [],
  toasts: [],
  wsStatus: 'connecting',
  view: 'inventory',
}

function reducer(state, action) {
  switch (action.type) {
    case 'INIT_ITEMS':  return { ...state, items: action.items }
    case 'INIT_ALERTS': return { ...state, alerts: action.alerts }
    case 'WS_STATUS':   return { ...state, wsStatus: action.status }
    case 'SET_VIEW':    return { ...state, view: action.view }

    case 'ADD_ITEM':
      return { ...state, items: [action.item, ...state.items] }

    case 'UPDATE_ITEM': {
      const patch = action.patch
      return {
        ...state,
        items: state.items.map(i =>
          i.item_id === patch.item_id ? { ...i, ...patch } : i
        ),
      }
    }

    case 'REMOVE_ITEM':
      return { ...state, items: state.items.filter(i => i.item_id !== action.item_id) }

    case 'ADD_ALERT': {
      const toast = { ...action.alert, _toastId: Date.now() + Math.random() }
      return {
        ...state,
        alerts: [action.alert, ...state.alerts],
        toasts: [toast, ...state.toasts].slice(0, 3),
      }
    }

    case 'REMOVE_TOAST':
      return { ...state, toasts: state.toasts.filter(t => t._toastId !== action.id) }

    case 'WS_MESSAGE': {
      const { event, data } = action.msg
      if (event === 'ITEM_INSERTED') return reducer(state, { type: 'ADD_ITEM', item: data })
      if (event === 'ITEM_SCORED')   return reducer(state, { type: 'UPDATE_ITEM', patch: data })
      if (event === 'ITEM_UPDATED')  return reducer(state, { type: 'UPDATE_ITEM', patch: { item_id: data.item_id, ...data.changed_fields } })
      if (event === 'ITEM_DELETED')  return reducer(state, { type: 'REMOVE_ITEM', item_id: data.item_id })
      if (event === 'ALERT_FIRED')   return reducer(state, { type: 'ADD_ALERT', alert: data })
      return state
    }

    default: return state
  }
}

// ─── App ────────────────────────────────────────────────────────────────────

export default function App() {
  const [state, dispatch] = useReducer(reducer, initial)
  const toastTimers = useRef({})

  // Initial data load
  useEffect(() => {
    api.getItems().then(items => dispatch({ type: 'INIT_ITEMS', items })).catch(() => {})
    api.getAlerts().then(alerts => dispatch({ type: 'INIT_ALERTS', alerts })).catch(() => {})
  }, [])

  // WebSocket
  useEffect(() => {
    const cleanup = createWsClient(dispatch)
    return cleanup
  }, [])

  // Auto-dismiss toasts after 5s
  useEffect(() => {
    const latest = state.toasts[0]
    if (!latest) return
    if (toastTimers.current[latest._toastId]) return
    const tid = setTimeout(() => {
      dispatch({ type: 'REMOVE_TOAST', id: latest._toastId })
      delete toastTimers.current[latest._toastId]
    }, 5000)
    toastTimers.current[latest._toastId] = tid
    return () => clearTimeout(tid)
  }, [state.toasts])

  const alertCount = state.alerts.length

  return (
    <div style={{ minHeight: '100vh', background: C.bg }}>
      <AlertBanner toasts={state.toasts} dispatch={dispatch} />

      {/* Header */}
      <header style={{
        borderBottom: `1px solid ${C.border}`,
        padding: '0 28px',
        display: 'flex', alignItems: 'center', justifyContent: 'space-between',
        height: 56,
      }}>
        <div style={{
          fontWeight: 700, fontSize: 18, color: C.teal,
          letterSpacing: '-0.02em', display: 'flex', alignItems: 'center', gap: 10,
        }}>
          🧊 FridgeAI
        </div>

        <nav style={{ display: 'flex', gap: 4 }}>
          {[
            { id: 'inventory', label: 'Inventory' },
            { id: 'alerts',    label: `Alerts${alertCount ? ` (${alertCount})` : ''}` },
            { id: 'analytics', label: 'Analytics' },
          ].map(tab => (
            <NavTab
              key={tab.id}
              label={tab.label}
              active={state.view === tab.id}
              onClick={() => dispatch({ type: 'SET_VIEW', view: tab.id })}
            />
          ))}
        </nav>

        <WsIndicator status={state.wsStatus} />
      </header>

      {/* Body */}
      <main style={{ maxWidth: 1100, margin: '0 auto', padding: '28px 24px' }}>
        {state.view === 'inventory' && <Inventory items={state.items} />}
        {state.view === 'alerts'    && <Alerts alerts={state.alerts} />}
        {state.view === 'analytics' && <Analytics items={state.items} />}
      </main>
    </div>
  )
}

function NavTab({ label, active, onClick }) {
  return (
    <button onClick={onClick} style={{
      background: active ? C.surface2 : 'none',
      border: active ? `1px solid ${C.border2}` : '1px solid transparent',
      color: active ? C.text : C.muted,
      borderRadius: 8, padding: '6px 16px', cursor: 'pointer',
      fontSize: 13, fontFamily: "'Syne', sans-serif", fontWeight: active ? 600 : 400,
    }}>
      {label}
    </button>
  )
}

function WsIndicator({ status }) {
  const dot = { connecting: C.warn, connected: C.safe, disconnected: C.critical }[status] ?? C.muted
  const label = { connecting: 'Connecting…', connected: 'Live', disconnected: 'Offline' }[status]

  return (
    <div style={{
      display: 'flex', alignItems: 'center', gap: 7,
      fontSize: 11, color: C.muted,
      fontFamily: "'JetBrains Mono', monospace",
    }}>
      <div style={{
        width: 7, height: 7, borderRadius: '50%', background: dot,
        boxShadow: status === 'connected' ? `0 0 6px ${dot}` : 'none',
      }} />
      {label}
    </div>
  )
}
