import { C, ALERT_COLOR, ALERT_LABEL } from '../constants.js'

function fmtTime(iso) {
  try {
    return new Date(iso).toLocaleString(undefined, {
      month: 'short', day: 'numeric',
      hour: '2-digit', minute: '2-digit',
    })
  } catch (e) {
    return iso
  }
}

export default function Alerts({ alerts }) {
  if (alerts.length === 0) {
    return (
      <div style={{ textAlign: 'center', color: C.muted, padding: '60px 0' }}>
        <div style={{ fontSize: 40, marginBottom: 12 }}>✅</div>
        <div style={{ fontSize: 15 }}>No alerts yet — everything looks fresh.</div>
      </div>
    )
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
      {alerts.map(a => <AlertRow key={a.alert_id} alert={a} />)}
    </div>
  )
}

function AlertRow({ alert }) {
  const color = ALERT_COLOR[alert.alert_type] ?? C.muted
  const label = ALERT_LABEL[alert.alert_type] ?? alert.alert_type

  return (
    <div style={{
      background: C.surface,
      border: `1px solid ${C.border}`,
      borderLeft: `3px solid ${color}`,
      borderRadius: 9,
      padding: '12px 16px',
      display: 'flex',
      alignItems: 'center',
      gap: 14,
    }}>
      {/* Type badge */}
      <div style={{
        background: color + '22', color, borderRadius: 5,
        padding: '3px 9px', fontSize: 10, fontWeight: 700,
        fontFamily: "'JetBrains Mono', monospace",
        letterSpacing: '0.05em', whiteSpace: 'nowrap', flexShrink: 0,
      }}>
        {label}
      </div>

      {/* Content */}
      <div style={{ flex: 1, minWidth: 0 }}>
        <div style={{ fontSize: 14, fontWeight: 600, color: C.text }}>{alert.item_name}</div>
        <div style={{ fontSize: 12, color: C.muted, marginTop: 2, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
          {alert.message}
        </div>
      </div>

      {/* Stats */}
      <div style={{ display: 'flex', gap: 14, fontFamily: "'JetBrains Mono', monospace", fontSize: 11, flexShrink: 0 }}>
        {alert.P_spoil != null && (
          <Stat label="P_spoil" value={`${(alert.P_spoil * 100).toFixed(0)}%`} color={color} />
        )}
        {alert.RSL != null && (
          <Stat label="RSL" value={`${Number(alert.RSL).toFixed(1)}d`} color={alert.RSL < 1 ? C.critical : C.text} />
        )}
      </div>

      {/* Timestamp */}
      <div style={{ fontSize: 11, color: C.muted, whiteSpace: 'nowrap', flexShrink: 0 }}>
        {fmtTime(alert.created_at)}
      </div>
    </div>
  )
}

function Stat({ label, value, color }) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 1, alignItems: 'center' }}>
      <div style={{ color: C.muted, fontSize: 9, textTransform: 'uppercase', letterSpacing: '0.06em' }}>{label}</div>
      <div style={{ color, fontWeight: 500 }}>{value}</div>
    </div>
  )
}
