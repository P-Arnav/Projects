import { useReducer, useEffect, useRef, useState, useCallback } from 'react'
import { C } from './constants.js'
import { api, createWsClient } from './api.js'
import Inventory from './views/Inventory.jsx'
import Alerts from './views/Alerts.jsx'
import Analytics from './views/Analytics.jsx'
import GroceryList from './views/GroceryList.jsx'
import Recipes from './views/Recipes.jsx'
import Login from './views/Login.jsx'
import AlertBanner from './components/AlertBanner.jsx'

// ─── Reducer ────────────────────────────────────────────────────────────────

const initial = {
  items: [],
  alerts: [],
  groceryItems: [],
  toasts: [],
  wsStatus: 'connecting',
  view: 'inventory',
}

function reducer(state, action) {
  switch (action.type) {
    case 'INIT_ITEMS':   return { ...state, items: action.items }
    case 'INIT_ALERTS':  return { ...state, alerts: action.alerts }
    case 'INIT_GROCERY': return { ...state, groceryItems: action.items }
    case 'WS_STATUS':    return { ...state, wsStatus: action.status }
    case 'SET_VIEW':     return { ...state, view: action.view }

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

    case 'ADD_GROCERY':
      return { ...state, groceryItems: [action.item, ...state.groceryItems] }

    case 'UPDATE_GROCERY':
      return {
        ...state,
        groceryItems: state.groceryItems.map(g =>
          g.grocery_id === action.item.grocery_id ? action.item : g
        ),
      }

    case 'REMOVE_GROCERY':
      return { ...state, groceryItems: state.groceryItems.filter(g => g.grocery_id !== action.grocery_id) }

    case 'CLEAR_CHECKED_GROCERY':
      return { ...state, groceryItems: state.groceryItems.filter(g => !g.checked) }

    case 'WS_MESSAGE': {
      const { event, data } = action.msg
      if (event === 'ITEM_INSERTED') return reducer(state, { type: 'ADD_ITEM', item: data })
      if (event === 'ITEM_SCORED')   return reducer(state, { type: 'UPDATE_ITEM', patch: data })
      if (event === 'ITEM_UPDATED')  return reducer(state, { type: 'UPDATE_ITEM', patch: { item_id: data.item_id, ...data.changed_fields } })
      if (event === 'ITEM_DELETED')  return reducer(state, { type: 'REMOVE_ITEM', item_id: data.item_id })
      if (event === 'ALERT_FIRED')   return reducer(state, { type: 'ADD_ALERT', alert: data })
      if (event === 'AUTO_RESTOCK') {
        const toast = { alert_type: 'AUTO_RESTOCK', message: data.message, _toastId: Date.now() + Math.random() }
        return { ...state, toasts: [toast, ...state.toasts].slice(0, 3) }
      }
      if (event === 'GROCERY_UPDATED') {
        if (data.deleted)         return reducer(state, { type: 'REMOVE_GROCERY', grocery_id: data.grocery_id })
        if (data.cleared_checked) return reducer(state, { type: 'CLEAR_CHECKED_GROCERY' })
        const exists = state.groceryItems.some(g => g.grocery_id === data.grocery_id)
        if (exists) return reducer(state, { type: 'UPDATE_GROCERY', item: data })
        return reducer(state, { type: 'ADD_GROCERY', item: data })
      }
      return state
    }

    default: return state
  }
}

// ─── App ────────────────────────────────────────────────────────────────────

// Inject keyframes once at module level
const styleEl = document.createElement('style')
styleEl.textContent = `
  @keyframes viewIn {
    from { opacity: 0; }
    to   { opacity: 1; }
  }
  @keyframes spin {
    to { transform: rotate(360deg); }
  }
`
document.head.appendChild(styleEl)

export default function App() {
  const [state, dispatch] = useReducer(reducer, initial)
  const toastTimers = useRef({})
  const [authUser, setAuthUser] = useState(null)
  const [requireAuth, setRequireAuth] = useState(false)
  const [authChecked, setAuthChecked] = useState(false)
  const [dataLoading, setDataLoading] = useState(true)
  const [animKey, setAnimKey] = useState(0)

  // Check auth config then validate any stored token
  useEffect(() => {
    api.authConfig()
      .then(async cfg => {
        setRequireAuth(cfg.require_auth)
        const token = localStorage.getItem('fridge_token')
        if (token) {
          try {
            const user = await api.getMe()
            localStorage.setItem('fridge_user', JSON.stringify(user))
            setAuthUser(user)
          } catch {
            // Token invalid or expired — clear it
            localStorage.removeItem('fridge_token')
            localStorage.removeItem('fridge_user')
          }
        }
        setAuthChecked(true)
      })
      .catch(() => setAuthChecked(true))
  }, [])

  // Initial data load (runs when auth is resolved)
  useEffect(() => {
    if (!authChecked) return
    if (requireAuth && !authUser) return
    setDataLoading(true)
    Promise.all([
      api.getItems().then(items => dispatch({ type: 'INIT_ITEMS', items })).catch(() => {}),
      api.getAlerts().then(alerts => dispatch({ type: 'INIT_ALERTS', alerts })).catch(() => {}),
      api.getGrocery().then(items => dispatch({ type: 'INIT_GROCERY', items })).catch(() => {}),
    ]).finally(() => setDataLoading(false))
  }, [authChecked, authUser, requireAuth])

  const setView = useCallback((view) => {
    dispatch({ type: 'SET_VIEW', view })
    setAnimKey(k => k + 1)
  }, [])

  // WebSocket
  useEffect(() => {
    if (!authChecked) return
    if (requireAuth && !authUser) return
    const cleanup = createWsClient(dispatch)
    return cleanup
  }, [authChecked, authUser, requireAuth])

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

  if (!authChecked) return null

  if (requireAuth && !authUser) {
    return <Login onAuth={(user) => setAuthUser(user)} />
  }

  const handleLogout = () => {
    localStorage.removeItem('fridge_token')
    localStorage.removeItem('fridge_user')
    setAuthUser(null)
  }

  const alertCount = state.alerts.length
  const groceryCount = state.groceryItems.filter(g => !g.checked).length

  const navItems = [
    { id: 'inventory', label: 'Inventory',  icon: <GridIcon />,  color: C.teal },
    { id: 'alerts',    label: 'Alerts',     icon: <BellIcon />,  color: C.critical, badge: alertCount },
    { id: 'analytics', label: 'Analytics',  icon: <ChartIcon />, color: C.blue },
    { id: 'grocery',   label: 'Grocery',    icon: <CartIcon />,  color: C.warn,     badge: groceryCount },
    { id: 'recipes',   label: 'Recipes',    icon: <CupIcon />,   color: C.orange },
  ]

  const activeNav = navItems.find(n => n.id === state.view)
  const viewLabel = activeNav?.label ?? ''
  const viewColor = activeNav?.color ?? C.teal

  return (
    <div style={{ display: 'flex', height: '100vh', background: C.bg }}>
      <AlertBanner toasts={state.toasts} dispatch={dispatch} />

      {/* ── Sidebar ── */}
      <aside style={{
        width: 64, minHeight: '100vh', background: C.surface,
        borderRight: `1px solid ${C.border}`,
        display: 'flex', flexDirection: 'column', alignItems: 'center',
        paddingTop: 14, gap: 2, flexShrink: 0,
      }}>
        {/* Logo mark */}
        <div style={{
          width: 36, height: 36, borderRadius: 10,
          background: C.teal + '18', border: `1px solid ${C.teal}44`,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          marginBottom: 18,
        }}>
          <span style={{ color: C.teal, fontWeight: 800, fontSize: 15, fontFamily: 'sans-serif' }}>F</span>
        </div>

        {navItems.map(item => (
          <SidebarBtn
            key={item.id}
            icon={item.icon}
            label={item.label}
            badge={item.badge}
            color={item.color}
            active={state.view === item.id}
            onClick={() => setView(item.id)}
          />
        ))}
      </aside>

      {/* ── Main ── */}
      <div style={{ flex: 1, display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>
        {/* Top header */}
        <header style={{
          height: 56, background: C.surface, borderBottom: `1px solid ${C.border}`,
          display: 'flex', alignItems: 'center', justifyContent: 'space-between',
          padding: '0 24px', flexShrink: 0,
        }}>
          <div style={{
            fontSize: 11, fontWeight: 700, color: viewColor,
            letterSpacing: '0.12em', textTransform: 'uppercase',
            fontFamily: "'Syne', sans-serif",
          }}>
            {viewLabel}
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: 14 }}>
            <WsIndicator status={state.wsStatus} />
            {authUser && (
              <>
                <span style={{ fontSize: 12, color: C.muted, fontFamily: "'Syne', sans-serif" }}>
                  {authUser.username}
                </span>
                <button onClick={handleLogout} style={{
                  background: 'none', border: `1px solid ${C.border2}`,
                  borderRadius: 6, padding: '4px 10px', color: C.muted,
                  cursor: 'pointer', fontSize: 11, fontFamily: "'Syne', sans-serif",
                }}>Sign out</button>
              </>
            )}
          </div>
        </header>

        {/* Content */}
        <main style={{ flex: 1, overflowY: 'auto', padding: '24px 28px', position: 'relative' }}>
          {dataLoading ? (
            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', height: '60vh', gap: 16 }}>
              <div style={{
                width: 34, height: 34, borderRadius: '50%',
                border: `3px solid ${C.border2}`,
                borderTopColor: C.teal,
                animation: 'spin 0.75s linear infinite',
              }} />
              <span style={{ fontSize: 12, color: C.muted, fontFamily: "'Syne', sans-serif", letterSpacing: '0.08em' }}>
                LOADING
              </span>
            </div>
          ) : (
            <div key={animKey} style={{ animation: 'viewIn 0.22s ease both' }}>
              {state.view === 'inventory' && <Inventory items={state.items} />}
              {state.view === 'alerts'    && <Alerts alerts={state.alerts} />}
              {state.view === 'analytics' && <Analytics items={state.items} dispatch={dispatch} groceryItems={state.groceryItems} />}
              {state.view === 'grocery'   && <GroceryList groceryItems={state.groceryItems} dispatch={dispatch} />}
              {state.view === 'recipes'   && <Recipes items={state.items} />}
            </div>
          )}
        </main>
      </div>
    </div>
  )
}

function SidebarBtn({ icon, label, active, badge, color, onClick }) {
  return (
    <button
      onClick={onClick}
      title={label}
      style={{
        position: 'relative',
        width: 44, height: 44, borderRadius: 10,
        background: active ? color + '1a' : 'none',
        border: `1px solid ${active ? color + '55' : 'transparent'}`,
        color: active ? color : C.muted,
        cursor: 'pointer',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        transition: 'all 0.15s',
      }}
      onMouseEnter={e => { if (!active) { e.currentTarget.style.background = color + '12'; e.currentTarget.style.color = color } }}
      onMouseLeave={e => { if (!active) { e.currentTarget.style.background = 'none'; e.currentTarget.style.color = C.muted } }}
    >
      {icon}
      {badge > 0 && (
        <span style={{
          position: 'absolute', top: 6, right: 6,
          width: 7, height: 7, borderRadius: '50%',
          background: color,
          boxShadow: `0 0 4px ${color}`,
        }} />
      )}
    </button>
  )
}

function WsIndicator({ status }) {
  const dot = { connecting: C.warn, connected: C.safe, disconnected: C.critical }[status] ?? C.muted
  const label = { connecting: 'Connecting', connected: 'Live', disconnected: 'Offline' }[status]
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 11, color: C.muted, fontFamily: "'JetBrains Mono', monospace" }}>
      <div style={{ width: 6, height: 6, borderRadius: '50%', background: dot, boxShadow: status === 'connected' ? `0 0 5px ${dot}` : 'none' }} />
      {label}
    </div>
  )
}

// ── Sidebar SVG Icons ────────────────────────────────────────────────────────

function GridIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
      <rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/>
      <rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>
    </svg>
  )
}
function BellIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
      <path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"/>
      <path d="M13.73 21a2 2 0 0 1-3.46 0"/>
    </svg>
  )
}
function ChartIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
      <line x1="18" y1="20" x2="18" y2="10"/><line x1="12" y1="20" x2="12" y2="4"/>
      <line x1="6" y1="20" x2="6" y2="14"/><line x1="3" y1="20" x2="21" y2="20"/>
    </svg>
  )
}
function CartIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="9" cy="21" r="1"/><circle cx="20" cy="21" r="1"/>
      <path d="M1 1h4l2.68 13.39a2 2 0 0 0 2 1.61h9.72a2 2 0 0 0 2-1.61L23 6H6"/>
    </svg>
  )
}
function CupIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
      <path d="M18 8h1a4 4 0 0 1 0 8h-1"/>
      <path d="M2 8h16v9a4 4 0 0 1-4 4H6a4 4 0 0 1-4-4V8z"/>
      <line x1="6" y1="1" x2="6" y2="4"/><line x1="10" y1="1" x2="10" y2="4"/><line x1="14" y1="1" x2="14" y2="4"/>
    </svg>
  )
}
