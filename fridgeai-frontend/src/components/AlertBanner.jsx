import { C, ALERT_COLOR, ALERT_LABEL } from '../constants.js'

export default function AlertBanner({ toasts, dispatch }) {
  if (!toasts.length) return null

  return (
    <div style={{
      position: 'fixed', top: 20, right: 20, zIndex: 200,
      display: 'flex', flexDirection: 'column', gap: 10,
      maxWidth: 340, pointerEvents: 'none',
    }}>
      {toasts.slice(0, 3).map(t => (
        <Toast key={t._toastId} toast={t} dispatch={dispatch} />
      ))}
    </div>
  )
}

function Toast({ toast, dispatch }) {
  const color = ALERT_COLOR[toast.alert_type] ?? C.muted
  const label = ALERT_LABEL[toast.alert_type] ?? toast.alert_type

  return (
    <div style={{
      background: C.surface,
      border: `1px solid ${color}66`,
      borderLeft: `3px solid ${color}`,
      borderRadius: 9,
      padding: '11px 14px',
      pointerEvents: 'all',
      display: 'flex',
      alignItems: 'flex-start',
      gap: 10,
    }}>
      <div style={{
        background: color + '22', color, borderRadius: 4,
        padding: '1px 6px', fontSize: 10, fontWeight: 700,
        fontFamily: "'JetBrains Mono', monospace",
        letterSpacing: '0.05em', whiteSpace: 'nowrap', marginTop: 1,
      }}>
        {label}
      </div>
      <div style={{ flex: 1 }}>
        <div style={{ fontSize: 13, fontWeight: 600, color: C.text }}>{toast.item_name}</div>
        <div style={{ fontSize: 11, color: C.muted, marginTop: 2 }}>{toast.message}</div>
      </div>
      <button
        onClick={() => dispatch({ type: 'REMOVE_TOAST', id: toast._toastId })}
        style={{
          background: 'none', border: 'none', color: C.muted,
          cursor: 'pointer', fontSize: 14, padding: 0, lineHeight: 1,
        }}
      >✕</button>
    </div>
  )
}
