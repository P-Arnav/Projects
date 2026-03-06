import { C, riskColor, riskLabel } from '../constants.js'
import { api } from '../api.js'

const fmt = (n, digits = 1) => n == null ? '—' : Number(n).toFixed(digits)
const fmtPct = (n) => n == null ? '—' : `${(n * 100).toFixed(0)}%`

export default function ItemCard({ item }) {
  const risk = riskColor(item.P_spoil)
  const label = riskLabel(item.P_spoil)
  const rslLow = item.RSL != null && item.RSL < 1

  const cardStyle = {
    background: C.surface,
    border: `1px solid ${item.P_spoil > 0.80 ? C.critical + '55' : C.border}`,
    borderRadius: 10,
    padding: '14px 16px',
    display: 'flex',
    flexDirection: 'column',
    gap: 10,
    position: 'relative',
    transition: 'border-color 0.3s',
  }

  return (
    <div style={cardStyle}>
      {/* Header row */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', gap: 8 }}>
        <div>
          <div style={{ fontWeight: 600, fontSize: 15, color: C.text }}>{item.name}</div>
          <div style={{ fontSize: 11, color: C.muted, fontFamily: "'JetBrains Mono', monospace", marginTop: 2 }}>
            {item.category} · qty {item.quantity}
          </div>
        </div>
        <div style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
          <Chip color={risk} label={label} />
          <button
            onClick={() => api.deleteItem(item.item_id)}
            title="Remove item"
            style={{
              background: 'none', border: `1px solid ${C.border2}`, color: C.muted,
              borderRadius: 6, padding: '3px 8px', cursor: 'pointer', fontSize: 12,
              lineHeight: 1,
            }}
          >✕</button>
        </div>
      </div>

      {/* Risk bar */}
      <div style={{ background: C.surface2, borderRadius: 4, height: 6, overflow: 'hidden' }}>
        <div style={{
          width: item.P_spoil == null ? '0%' : `${item.P_spoil * 100}%`,
          height: '100%',
          background: risk,
          borderRadius: 4,
          transition: 'width 0.6s ease, background 0.3s',
        }} />
      </div>

      {/* Stats row */}
      <div style={{ display: 'flex', gap: 16, fontFamily: "'JetBrains Mono', monospace", fontSize: 11 }}>
        <Stat label="P_spoil" value={fmtPct(item.P_spoil)} color={risk} />
        <Stat label="RSL" value={item.RSL == null ? '—' : `${fmt(item.RSL)}d`} color={rslLow ? C.critical : C.text} />
        <Stat label="FAPF" value={item.fapf_score == null ? '—' : fmt(item.fapf_score, 3)} color={C.text} />
        <Stat label="tier" value={item.confidence_tier} color={C.muted} />
      </div>
    </div>
  )
}

function Chip({ color, label }) {
  return (
    <div style={{
      background: color + '22',
      border: `1px solid ${color}55`,
      color,
      borderRadius: 5,
      padding: '2px 7px',
      fontSize: 10,
      fontWeight: 700,
      fontFamily: "'JetBrains Mono', monospace",
      letterSpacing: '0.05em',
      whiteSpace: 'nowrap',
    }}>
      {label}
    </div>
  )
}

function Stat({ label, value, color }) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
      <div style={{ color: C.muted, fontSize: 10, textTransform: 'uppercase', letterSpacing: '0.06em' }}>{label}</div>
      <div style={{ color, fontWeight: 500 }}>{value}</div>
    </div>
  )
}
