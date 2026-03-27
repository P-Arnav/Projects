import { C, riskColor, CAT_COLOR } from '../constants.js'
import { api } from '../api.js'

export default function ItemCard({ item }) {
  const risk = riskColor(item.P_spoil)
  const catColor = CAT_COLOR[item.category] || C.muted
  const rslPct = item.RSL == null ? 0 : Math.min(100, (item.RSL / Math.max(item.shelf_life, 1)) * 100)
  const ps = item.P_spoil

  return (
    <div
      style={{
        display: 'flex', alignItems: 'center', gap: 14,
        padding: '10px 18px',
        borderBottom: `1px solid ${C.border}`,
        transition: 'background 0.12s',
        cursor: 'default',
      }}
      onMouseEnter={e => e.currentTarget.style.background = C.surface2}
      onMouseLeave={e => e.currentTarget.style.background = 'transparent'}
    >
      {/* Status dot */}
      <div style={{
        width: 8, height: 8, borderRadius: '50%', background: risk, flexShrink: 0,
        boxShadow: ps != null && ps > 0.5 ? `0 0 7px ${risk}99` : 'none',
      }} />

      {/* Name */}
      <div style={{ flex: '0 0 150px', minWidth: 0 }}>
        <div style={{
          fontWeight: 600, fontSize: 13, color: C.text,
          overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap',
          fontFamily: "'Syne', sans-serif",
        }}>{item.name}</div>
        <div style={{ fontSize: 10, color: catColor, marginTop: 1, fontFamily: "'Syne', sans-serif" }}>
          {item.category} · qty {item.quantity}
        </div>
      </div>

      {/* PAIF action tag */}
      <div style={{ flex: '0 0 110px' }}>
        {item.paif_action ? (
          <span style={{
            fontSize: 10, fontWeight: 700, color: ps > 0.8 ? C.critical : C.warn,
            background: (ps > 0.8 ? C.critical : C.warn) + '18',
            borderRadius: 4, padding: '2px 7px',
            fontFamily: "'JetBrains Mono', monospace",
            letterSpacing: '0.03em',
          }}>
            {item.paif_action}
          </span>
        ) : (
          <span style={{ fontSize: 10, color: C.muted, fontFamily: "'JetBrains Mono', monospace" }}>—</span>
        )}
      </div>

      {/* RSL progress bar */}
      <div style={{ flex: 1, display: 'flex', alignItems: 'center', gap: 10 }}>
        <div style={{ flex: 1, height: 4, background: C.surface2, borderRadius: 2, overflow: 'hidden' }}>
          <div style={{
            width: ps == null ? '100%' : `${rslPct}%`,
            height: '100%',
            background: ps == null ? C.muted + '44' : risk,
            borderRadius: 2,
            transition: 'width 0.6s ease',
          }} />
        </div>
        <div style={{
          fontSize: 11, color: item.RSL != null && item.RSL < 1 ? C.critical : C.muted,
          fontFamily: "'JetBrains Mono', monospace",
          width: 36, textAlign: 'right', flexShrink: 0,
        }}>
          {item.RSL == null ? '—' : `${item.RSL.toFixed(1)}d`}
        </div>
      </div>

      {/* P_spoil */}
      <div style={{
        fontSize: 12, fontWeight: 700, color: risk,
        fontFamily: "'JetBrains Mono', monospace",
        width: 40, textAlign: 'right', flexShrink: 0,
      }}>
        {ps == null ? '—' : `${(ps * 100).toFixed(0)}%`}
      </div>

      {/* Actions */}
      <div style={{ display: 'flex', gap: 4, flexShrink: 0 }}>
        {ps != null && (
          <IconBtn
            onClick={() => api.submitFeedback(item.item_id, { still_good: true })}
            title="Still good — improve predictions"
            color={C.safe}
          >✓</IconBtn>
        )}
        <IconBtn
          onClick={() => api.deleteItem(item.item_id)}
          title="Remove item"
          color={C.muted}
        >✕</IconBtn>
      </div>
    </div>
  )
}

function IconBtn({ onClick, title, color, children }) {
  return (
    <button
      onClick={onClick}
      title={title}
      style={{
        background: 'none', border: `1px solid ${color}44`,
        color, borderRadius: 5, width: 24, height: 24,
        cursor: 'pointer', fontSize: 11, lineHeight: 1,
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        transition: 'background 0.12s',
      }}
      onMouseEnter={e => e.currentTarget.style.background = color + '18'}
      onMouseLeave={e => e.currentTarget.style.background = 'none'}
    >
      {children}
    </button>
  )
}
