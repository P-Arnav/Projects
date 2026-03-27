import { useState } from 'react'
import { C, CATEGORIES } from '../constants.js'
import { api } from '../api.js'

export default function GroceryList({ groceryItems, dispatch }) {
  const [newName, setNewName] = useState('')
  const [newQty, setNewQty] = useState(1)
  const [newCat, setNewCat] = useState('vegetable')
  const [adding, setAdding] = useState(false)

  const handleAdd = async () => {
    if (!newName.trim()) return
    setAdding(true)
    try {
      await api.addGrocery({ name: newName.trim(), category: newCat, quantity: newQty, source: 'manual' })
      setNewName('')
      setNewQty(1)
    } finally {
      setAdding(false)
    }
  }

  const handleToggle = async (item) => {
    await api.updateGrocery(item.grocery_id, { checked: !item.checked })
  }

  const handleDelete = async (grocery_id) => {
    await api.deleteGrocery(grocery_id)
  }

  const handleClearChecked = async () => {
    await api.clearCheckedGrocery()
  }

  const handleAddToFridge = async (grocery_id) => {
    await api.addGroceryToFridge(grocery_id)
  }

  const unchecked = groceryItems.filter(i => !i.checked)
  const checked = groceryItems.filter(i => i.checked)

  return (
    <div style={{ maxWidth: 620, margin: '0 auto' }}>
      <div style={{ fontSize: 18, fontWeight: 700, color: C.text, marginBottom: 20 }}>Grocery List</div>

      {/* Add form */}
      <div style={{
        background: C.surface, border: `1px solid ${C.border}`,
        borderRadius: 12, padding: '14px 18px', marginBottom: 24,
        display: 'flex', gap: 10, flexWrap: 'wrap', alignItems: 'center',
      }}>
        <input
          value={newName}
          onChange={e => setNewName(e.target.value)}
          onKeyDown={e => e.key === 'Enter' && handleAdd()}
          placeholder="Item name..."
          style={inputStyle}
        />
        <select value={newCat} onChange={e => setNewCat(e.target.value)} style={selectStyle}>
          {CATEGORIES.map(c => <option key={c} value={c}>{c}</option>)}
        </select>
        <input
          type="number" min={1} max={99} value={newQty}
          onChange={e => setNewQty(Math.max(1, Number(e.target.value)))}
          style={{ ...inputStyle, width: 60, flex: 'none' }}
        />
        <button onClick={handleAdd} disabled={adding || !newName.trim()} style={addBtnStyle}>
          + Add
        </button>
      </div>

      {/* List */}
      {unchecked.length === 0 && checked.length === 0 ? (
        <div style={{ textAlign: 'center', color: C.muted, padding: '60px 0' }}>
          <div style={{ fontSize: 36, marginBottom: 12 }}>🛒</div>
          <div style={{ fontSize: 15 }}>Your grocery list is empty.</div>
          <div style={{ fontSize: 13, marginTop: 6 }}>
            Add items manually or use Restock Suggestions in Analytics.
          </div>
        </div>
      ) : (
        <>
          {unchecked.map(item => (
            <GroceryRow key={item.grocery_id} item={item} onToggle={handleToggle} onDelete={handleDelete} onAddToFridge={handleAddToFridge} />
          ))}

          {checked.length > 0 && (
            <>
              <div style={{
                display: 'flex', justifyContent: 'space-between', alignItems: 'center',
                marginTop: 28, marginBottom: 10,
              }}>
                <span style={{ fontSize: 11, color: C.muted, fontWeight: 600, letterSpacing: '0.07em' }}>
                  CHECKED ({checked.length})
                </span>
                <button onClick={handleClearChecked} style={clearBtnStyle}>
                  Clear all checked
                </button>
              </div>
              {checked.map(item => (
                <GroceryRow key={item.grocery_id} item={item} onToggle={handleToggle} onDelete={handleDelete} onAddToFridge={handleAddToFridge} />
              ))}
            </>
          )}
        </>
      )}
    </div>
  )
}

function GroceryRow({ item, onToggle, onDelete, onAddToFridge }) {
  const sourceColor = item.source === 'restock' ? C.warn : item.source === 'recipe' ? C.blue : C.muted
  return (
    <div style={{
      display: 'flex', alignItems: 'center', gap: 12,
      padding: '11px 16px', marginBottom: 6,
      background: C.surface, border: `1px solid ${C.border}`,
      borderRadius: 8, opacity: item.checked ? 0.45 : 1,
      transition: 'opacity 0.2s',
    }}>
      <input
        type="checkbox" checked={item.checked} onChange={() => onToggle(item)}
        style={{ accentColor: C.teal, width: 16, height: 16, cursor: 'pointer', flexShrink: 0 }}
      />
      <div style={{ flex: 1, minWidth: 0 }}>
        <span style={{
          color: item.checked ? C.muted : C.text,
          fontSize: 14, fontWeight: 500,
          textDecoration: item.checked ? 'line-through' : 'none',
        }}>
          {item.name}
        </span>
        {item.quantity > 1 && (
          <span style={{ color: C.muted, fontSize: 12, marginLeft: 8 }}>×{item.quantity}</span>
        )}
      </div>
      <span style={{
        fontSize: 10, fontWeight: 700, color: sourceColor,
        background: sourceColor + '22', borderRadius: 4,
        padding: '2px 7px', letterSpacing: '0.05em', flexShrink: 0,
      }}>
        {item.source.toUpperCase()}
      </span>
      <span style={{ fontSize: 11, color: C.muted, flexShrink: 0 }}>{item.category}</span>
      {!item.checked && (
        <button
          onClick={() => onAddToFridge(item.grocery_id)}
          title="Add to fridge inventory"
          style={{
            background: C.teal + '18', border: `1px solid ${C.teal}55`,
            color: C.teal, borderRadius: 6, padding: '3px 10px',
            cursor: 'pointer', fontSize: 11, fontWeight: 700,
            fontFamily: "'Syne', sans-serif", flexShrink: 0, whiteSpace: 'nowrap',
          }}
        >
          → Fridge
        </button>
      )}
      <button
        onClick={() => onDelete(item.grocery_id)}
        style={{ background: 'none', border: 'none', color: C.muted, cursor: 'pointer', fontSize: 18, padding: '0 2px', lineHeight: 1, flexShrink: 0 }}
      >
        ×
      </button>
    </div>
  )
}

const inputStyle = {
  background: C.bg, border: `1px solid #1a2e4a`, borderRadius: 6,
  color: '#e8f0fe', padding: '8px 12px', fontSize: 13, flex: 1, minWidth: 110,
  fontFamily: "'Syne', sans-serif", outline: 'none',
}

const selectStyle = {
  background: C.bg, border: `1px solid #1a2e4a`, borderRadius: 6,
  color: '#e8f0fe', padding: '8px 10px', fontSize: 13, flex: 'none',
  fontFamily: "'Syne', sans-serif", cursor: 'pointer', outline: 'none',
}

const addBtnStyle = {
  background: C.teal, color: C.bg, border: 'none', borderRadius: 6,
  padding: '8px 18px', fontWeight: 700, cursor: 'pointer', fontSize: 13,
  fontFamily: "'Syne', sans-serif", whiteSpace: 'nowrap', flexShrink: 0,
}

const clearBtnStyle = {
  background: 'none', border: `1px solid ${C.border}`, color: C.muted,
  borderRadius: 6, padding: '4px 10px', cursor: 'pointer', fontSize: 11,
  fontFamily: "'Syne', sans-serif",
}
