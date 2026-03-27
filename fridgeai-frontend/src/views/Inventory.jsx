import { useState } from 'react'
import { C, CATEGORIES, CAT_COLOR, riskColor } from '../constants.js'
import ItemCard from '../components/ItemCard.jsx'
import AddItemModal from '../components/AddItemModal.jsx'
import ScanModal from '../components/ScanModal.jsx'
import ReceiptModal from '../components/ReceiptModal.jsx'

export default function Inventory({ items }) {
  const [filter, setFilter] = useState('all')
  const [showModal, setShowModal] = useState(false)
  const [showScan, setShowScan] = useState(false)
  const [showReceipt, setShowReceipt] = useState(false)

  const scored = items.filter(i => i.P_spoil != null)
  const critical = items.filter(i => i.P_spoil > 0.80).length
  const warning  = items.filter(i => i.P_spoil > 0.50 && i.P_spoil <= 0.80).length
  const safe     = items.filter(i => i.P_spoil != null && i.P_spoil <= 0.50).length
  const expiring = items.filter(i => i.RSL != null && i.RSL < 1).length

  const visible = filter === 'all' ? items : items.filter(i => i.category === filter)

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 20 }}>

      {/* ── Stat cards ── */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 14 }}>
        <StatCard
          label="TOTAL ITEMS"
          primary={items.length}
          secondary={`${scored.length} scored`}
          accent={C.teal}
          total={items.length}
        />
        <StatCard
          label="CRITICAL"
          primary={critical}
          secondary={`${warning} warning`}
          accent={C.critical}
          total={items.length}
        />
        <StatCard
          label="EXPIRING TODAY"
          primary={expiring}
          secondary="RSL < 1 day"
          accent={expiring > 0 ? C.warn : C.muted}
          total={items.length}
        />
        <StatCard
          label="SAFE"
          primary={safe}
          secondary={`${items.length > 0 ? Math.round(safe / items.length * 100) : 0}% of inventory`}
          accent={C.safe}
          total={items.length}
        />
      </div>

      {/* ── Category dot strip ── */}
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
        {CATEGORIES.map(cat => {
          const n = items.filter(i => i.category === cat).length
          if (n === 0) return null
          const color = CAT_COLOR[cat] || C.muted
          return (
            <button key={cat} onClick={() => setFilter(filter === cat ? 'all' : cat)} style={{
              display: 'flex', alignItems: 'center', gap: 7,
              background: filter === cat ? color + '18' : 'none',
              border: `1px solid ${filter === cat ? color + '66' : C.border}`,
              borderRadius: 20, padding: '5px 12px', cursor: 'pointer',
              transition: 'all 0.15s',
            }}>
              <span style={{ width: 7, height: 7, borderRadius: '50%', background: color, display: 'inline-block' }} />
              <span style={{ fontSize: 12, color: filter === cat ? color : C.muted, fontFamily: "'Syne', sans-serif", fontWeight: filter === cat ? 600 : 400 }}>
                {cat}
              </span>
              <span style={{ fontSize: 10, color: filter === cat ? color + 'cc' : C.muted + '88', fontFamily: "'JetBrains Mono', monospace" }}>
                {n}
              </span>
            </button>
          )
        })}
        {filter !== 'all' && (
          <button onClick={() => setFilter('all')} style={{
            fontSize: 11, color: C.muted, background: 'none',
            border: `1px solid ${C.border}`, borderRadius: 20,
            padding: '5px 12px', cursor: 'pointer', fontFamily: "'Syne', sans-serif",
          }}>
            Show all
          </button>
        )}
      </div>

      {/* ── Item list panel ── */}
      <div style={{ background: C.surface, border: `1px solid ${C.border}`, borderRadius: 12, overflow: 'hidden' }}>
        {/* Panel header */}
        <div style={{
          display: 'flex', alignItems: 'center', justifyContent: 'space-between',
          padding: '14px 18px', borderBottom: `1px solid ${C.border}`,
          background: C.surface2,
        }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
            <span style={{ fontSize: 11, fontWeight: 700, color: C.muted, letterSpacing: '0.1em', textTransform: 'uppercase', fontFamily: "'Syne', sans-serif" }}>
              Items Online
            </span>
            <span style={{
              fontSize: 10, color: C.teal, background: C.teal + '18',
              borderRadius: 10, padding: '1px 8px', fontFamily: "'JetBrains Mono', monospace", fontWeight: 700,
            }}>{visible.length}</span>
          </div>
          <div style={{ display: 'flex', gap: 8 }}>
            <ActionBtn onClick={() => setShowReceipt(true)} secondary>Upload Receipt</ActionBtn>
            <ActionBtn onClick={() => setShowScan(true)} outline>Scan Fridge</ActionBtn>
            <ActionBtn onClick={() => setShowModal(true)} primary>+ Add Item</ActionBtn>
          </div>
        </div>

        {/* Column headers */}
        {visible.length > 0 && (
          <div style={{
            display: 'flex', alignItems: 'center', gap: 14,
            padding: '6px 18px', borderBottom: `1px solid ${C.border}`,
          }}>
            <div style={{ width: 8, flexShrink: 0 }} />
            <div style={{ flex: '0 0 150px', fontSize: 10, color: C.muted, letterSpacing: '0.08em', textTransform: 'uppercase', fontFamily: "'Syne', sans-serif" }}>Name</div>
            <div style={{ flex: '0 0 110px', fontSize: 10, color: C.muted, letterSpacing: '0.08em', textTransform: 'uppercase', fontFamily: "'Syne', sans-serif" }}>Action</div>
            <div style={{ flex: 1, fontSize: 10, color: C.muted, letterSpacing: '0.08em', textTransform: 'uppercase', fontFamily: "'Syne', sans-serif" }}>RSL</div>
            <div style={{ width: 40, fontSize: 10, color: C.muted, letterSpacing: '0.08em', textTransform: 'uppercase', textAlign: 'right', fontFamily: "'Syne', sans-serif" }}>Spoil</div>
            <div style={{ width: 56, flexShrink: 0 }} />
          </div>
        )}

        {/* Rows */}
        {visible.length === 0 ? (
          <div style={{ padding: '52px 0', textAlign: 'center', color: C.muted }}>
            <div style={{ fontSize: 32, marginBottom: 10 }}>🧊</div>
            <div style={{ fontSize: 14, fontFamily: "'Syne', sans-serif", color: C.text }}>
              {filter === 'all' ? 'No items in the fridge yet.' : `No ${filter} items.`}
            </div>
            {filter === 'all' && (
              <div style={{ display: 'flex', gap: 10, justifyContent: 'center', marginTop: 20 }}>
                <ActionBtn onClick={() => setShowReceipt(true)} secondary>Upload Receipt</ActionBtn>
                <ActionBtn onClick={() => setShowScan(true)} outline>Scan Fridge</ActionBtn>
                <ActionBtn onClick={() => setShowModal(true)} primary>+ Add Item</ActionBtn>
              </div>
            )}
          </div>
        ) : (
          visible.map(item => <ItemCard key={item.item_id} item={item} />)
        )}
      </div>

      {showModal   && <AddItemModal   onClose={() => setShowModal(false)} />}
      {showScan    && <ScanModal      onClose={() => setShowScan(false)} />}
      {showReceipt && <ReceiptModal   onClose={() => setShowReceipt(false)} />}
    </div>
  )
}

function StatCard({ label, primary, secondary, accent, total }) {
  const pct = total > 0 ? Math.min(100, Math.round((primary / total) * 100)) : (primary > 0 ? 100 : 0)
  return (
    <div style={{
      background: C.surface, border: `1px solid ${C.border}`,
      borderRadius: 12, padding: '18px 20px',
      display: 'flex', flexDirection: 'column', gap: 6,
    }}>
      <div style={{ fontSize: 10, fontWeight: 700, color: C.muted, letterSpacing: '0.12em', textTransform: 'uppercase', fontFamily: "'Syne', sans-serif" }}>
        {label}
      </div>
      <div style={{ fontSize: 32, fontWeight: 800, color: accent, fontFamily: "'Syne', sans-serif", lineHeight: 1 }}>
        {primary}
      </div>
      <div style={{ fontSize: 11, color: C.muted, fontFamily: "'Syne', sans-serif" }}>
        {secondary}
      </div>
      {/* Accent bar — width reflects actual proportion */}
      <div style={{ height: 2, background: accent + '33', borderRadius: 1, marginTop: 4 }}>
        <div style={{ width: `${pct}%`, height: '100%', background: accent, borderRadius: 1, transition: 'width 0.4s ease' }} />
      </div>
    </div>
  )
}

function ActionBtn({ onClick, children, primary, outline, secondary }) {
  const bg = primary ? C.teal : 'none'
  const color = primary ? C.bg : outline ? C.teal : C.muted
  const border = primary ? 'none' : `1px solid ${outline ? C.teal + '88' : C.border2}`
  return (
    <button onClick={onClick} style={{
      background: bg, color, border, borderRadius: 7,
      padding: '7px 14px', cursor: 'pointer', fontSize: 12,
      fontFamily: "'Syne', sans-serif", fontWeight: 600,
    }}>
      {children}
    </button>
  )
}
