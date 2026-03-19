import { useState } from 'react'
import { C, CATEGORIES } from '../constants.js'
import ItemCard from '../components/ItemCard.jsx'
import AddItemModal from '../components/AddItemModal.jsx'
import ScanModal from '../components/ScanModal.jsx'
import ReceiptModal from '../components/ReceiptModal.jsx'

export default function Inventory({ items }) {
  const [filter, setFilter] = useState('all')
  const [showModal, setShowModal] = useState(false)
  const [showScan, setShowScan] = useState(false)
  const [showReceipt, setShowReceipt] = useState(false)

  const visible = filter === 'all' ? items : items.filter(i => i.category === filter)

  return (
    <div>
      {/* Toolbar */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 20, flexWrap: 'wrap', gap: 12 }}>
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
          <Pill label="All" active={filter === 'all'} onClick={() => setFilter('all')} count={items.length} />
          {CATEGORIES.map(cat => {
            const n = items.filter(i => i.category === cat).length
            if (n === 0) return null
            return <Pill key={cat} label={cat} active={filter === cat} onClick={() => setFilter(cat)} count={n} />
          })}
        </div>
        <div style={{ display: 'flex', gap: 8 }}>
          <button onClick={() => setShowReceipt(true)} style={{
            background: 'none', color: C.muted, border: `1px solid ${C.border2}`, borderRadius: 8,
            padding: '9px 16px', fontWeight: 600, cursor: 'pointer', fontSize: 13,
            fontFamily: "'Syne', sans-serif",
          }}>
            Upload Receipt
          </button>
          <button onClick={() => setShowScan(true)} style={{
            background: 'none', color: C.teal, border: `1px solid ${C.teal}`, borderRadius: 8,
            padding: '9px 16px', fontWeight: 600, cursor: 'pointer', fontSize: 13,
            fontFamily: "'Syne', sans-serif",
          }}>
            Scan Fridge
          </button>
          <button onClick={() => setShowModal(true)} style={{
            background: C.teal, color: C.bg, border: 'none', borderRadius: 8,
            padding: '9px 18px', fontWeight: 700, cursor: 'pointer', fontSize: 13,
            fontFamily: "'Syne', sans-serif",
          }}>
            + Add Item
          </button>
        </div>
      </div>

      {/* Grid */}
      {visible.length === 0 ? (
        <Empty filter={filter} />
      ) : (
        <div style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fill, minmax(280px, 1fr))',
          gap: 14,
        }}>
          {visible.map(item => <ItemCard key={item.item_id} item={item} />)}
        </div>
      )}

      {showModal   && <AddItemModal   onClose={() => setShowModal(false)} />}
      {showScan    && <ScanModal      onClose={() => setShowScan(false)} />}
      {showReceipt && <ReceiptModal   onClose={() => setShowReceipt(false)} />}
    </div>
  )
}

function Pill({ label, active, onClick, count }) {
  return (
    <button onClick={onClick} style={{
      background: active ? C.teal + '22' : 'none',
      border: `1px solid ${active ? C.teal : C.border2}`,
      color: active ? C.teal : C.muted,
      borderRadius: 20, padding: '5px 13px', cursor: 'pointer', fontSize: 12,
      fontFamily: "'Syne', sans-serif", fontWeight: active ? 600 : 400,
      display: 'flex', alignItems: 'center', gap: 6,
    }}>
      {label}
      <span style={{
        background: active ? C.teal + '33' : C.surface2,
        color: active ? C.teal : C.muted,
        borderRadius: 10, padding: '0 6px', fontSize: 10, fontWeight: 700,
      }}>{count}</span>
    </button>
  )
}

function Empty({ filter }) {
  return (
    <div style={{ textAlign: 'center', color: C.muted, padding: '60px 0' }}>
      <div style={{ fontSize: 40, marginBottom: 12 }}>🧊</div>
      <div style={{ fontSize: 15 }}>
        {filter === 'all' ? 'No items in the fridge yet.' : `No ${filter} items.`}
      </div>
      <div style={{ fontSize: 13, marginTop: 6 }}>Click "+ Add Item" to get started.</div>
    </div>
  )
}
