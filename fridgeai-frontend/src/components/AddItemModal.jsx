import { useState } from 'react'
import { C, CATEGORIES } from '../constants.js'
import { api } from '../api.js'

const DEFAULTS = {
  name: '', category: 'dairy', quantity: 1, shelf_life: '',
  location: '', estimated_cost: '', storage_temp: 4.0, humidity: 50.0,
}

export default function AddItemModal({ onClose }) {
  const [form, setForm] = useState(DEFAULTS)
  const [barcode, setBarcode] = useState('')
  const [barcodeStatus, setBarcodeStatus] = useState('')  // '', 'loading', 'found', 'not_found'
  const [nameStatus, setNameStatus]       = useState('')  // '', 'loading', 'found', 'not_found'
  const [error, setError] = useState('')
  const [loading, setLoading] = useState(false)

  const set = (k) => (e) => setForm(f => ({ ...f, [k]: e.target.value }))

  async function handleBarcodeSubmit(e) {
    e.preventDefault()
    const b = barcode.trim()
    if (!b) return
    setBarcodeStatus('loading')
    try {
      const result = await api.lookupBarcode(b)
      setForm(f => ({
        ...f,
        name: result.name || f.name,
        category: result.category,
        shelf_life: String(result.shelf_life),
      }))
      setBarcodeStatus(result.off_found ? 'found' : 'not_found')
    } catch {
      setBarcodeStatus('error')
    }
  }

  async function handleNameLookup() {
    const n = form.name.trim()
    if (!n) return
    setNameStatus('loading')
    try {
      const result = await api.lookupItem(n)
      setForm(f => ({
        ...f,
        category:       result.category,
        shelf_life:     String(result.shelf_life),
        estimated_cost: result.estimated_cost > 0 ? String(result.estimated_cost) : f.estimated_cost,
      }))
      setNameStatus(result.source.startsWith('item-specific') ? 'found' : 'category')
    } catch {
      setNameStatus('not_found')
    }
  }

  async function handleCategoryChange(e) {
    const cat = e.target.value
    setForm(f => ({ ...f, category: cat }))
    try {
      const result = await api.getShelfLife(cat)
      setForm(f => ({ ...f, shelf_life: String(result.shelf_life) }))
    } catch { /* keep existing value */ }
  }

  async function submit(e) {
    e.preventDefault()
    setError('')
    if (!form.name.trim()) return setError('Name is required.')
    if (!form.shelf_life || Number(form.shelf_life) < 1) return setError('Shelf life must be ≥ 1 day.')

    setLoading(true)
    try {
      await api.postItem({
        name: form.name.trim(),
        category: form.category,
        quantity: Number(form.quantity) || 1,
        shelf_life: Number(form.shelf_life),
        location: form.location.trim(),
        estimated_cost: form.estimated_cost !== '' ? Number(form.estimated_cost) : 0,
        storage_temp: Number(form.storage_temp),
        humidity: Number(form.humidity),
      })
      onClose()
    } catch (err) {
      setError('Failed to add item. Is the backend running?')
    } finally {
      setLoading(false)
    }
  }

  return (
    <div style={{
      position: 'fixed', inset: 0, background: '#00000088', zIndex: 100,
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }} onClick={(e) => e.target === e.currentTarget && onClose()}>
      <div style={{
        background: C.surface, border: `1px solid ${C.border2}`, borderRadius: 14,
        padding: 28, width: 440, maxWidth: '95vw',
      }}>
        <div style={{ fontWeight: 700, fontSize: 18, marginBottom: 20, color: C.text }}>Add Item</div>
        {/* Barcode lookup */}
        <form onSubmit={handleBarcodeSubmit} style={{ display: 'flex', gap: 8, marginBottom: 4 }}>
          <Input
            value={barcode}
            onChange={e => { setBarcode(e.target.value); setBarcodeStatus('') }}
            placeholder="Scan / type barcode (optional)"
          />
          <button type="submit" disabled={barcodeStatus === 'loading'} style={{ ...btnPrimary, whiteSpace: 'nowrap', padding: '8px 14px' }}>
            {barcodeStatus === 'loading' ? '…' : 'Look up'}
          </button>
        </form>
        {barcodeStatus === 'found'     && <div style={{ color: C.safe,     fontSize: 12, marginBottom: 4 }}>Found in Open Food Facts — fields pre-filled.</div>}
        {barcodeStatus === 'not_found' && <div style={{ color: C.warn,     fontSize: 12, marginBottom: 4 }}>Barcode not in Open Food Facts — default values used.</div>}
        {barcodeStatus === 'error'     && <div style={{ color: C.critical, fontSize: 12, marginBottom: 4 }}>Barcode lookup failed. Fill fields manually.</div>}

        <form onSubmit={submit} style={{ display: 'flex', flexDirection: 'column', gap: 14 }}>
          <Row label="Name *">
            <div style={{ display: 'flex', gap: 8 }}>
              <Input
                value={form.name}
                onChange={e => { set('name')(e); setNameStatus('') }}
                placeholder="e.g. chicken, apple, milk"
                required
              />
              <button type="button" onClick={handleNameLookup} disabled={nameStatus === 'loading'}
                style={{ ...btnPrimary, whiteSpace: 'nowrap', padding: '8px 14px' }}>
                {nameStatus === 'loading' ? '…' : 'Look up'}
              </button>
            </div>
            {nameStatus === 'found'    && <div style={{ color: C.safe,  fontSize: 12, marginTop: 3 }}>Matched — category, shelf life & cost pre-filled.</div>}
            {nameStatus === 'category' && <div style={{ color: C.warn,  fontSize: 12, marginTop: 3 }}>No exact match — filled from category defaults.</div>}
            {nameStatus === 'not_found'&& <div style={{ color: C.critical, fontSize: 12, marginTop: 3 }}>Lookup failed. Fill fields manually.</div>}
          </Row>
          <Row label="Category *">
            <select value={form.category} onChange={handleCategoryChange} style={selectStyle}>
              {CATEGORIES.map(c => <option key={c} value={c}>{c}</option>)}
            </select>
          </Row>
          <TwoCol>
            <Row label="Qty">
              <Input type="number" value={form.quantity} onChange={set('quantity')} min={1} />
            </Row>
            <Row label="Shelf life (days) *">
              <Input type="number" value={form.shelf_life} onChange={set('shelf_life')} min={1} placeholder="7" required />
            </Row>
          </TwoCol>
          <TwoCol>
            <Row label="Storage temp (°C)">
              <Input type="number" value={form.storage_temp} onChange={set('storage_temp')} step={0.5} />
            </Row>
            <Row label="Humidity (%)">
              <Input type="number" value={form.humidity} onChange={set('humidity')} min={0} max={100} />
            </Row>
          </TwoCol>
          <TwoCol>
            <Row label="Cost (₹)">
              <Input type="number" value={form.estimated_cost} onChange={set('estimated_cost')} step={1} min={0} placeholder="0" />
            </Row>
            <Row label="Location">
              <Input value={form.location} onChange={set('location')} placeholder="shelf 1" />
            </Row>
          </TwoCol>

          {error && <div style={{ color: C.critical, fontSize: 13 }}>{error}</div>}

          <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 10, marginTop: 4 }}>
            <button type="button" onClick={onClose} style={btnSecondary}>Cancel</button>
            <button type="submit" disabled={loading} style={btnPrimary}>
              {loading ? 'Adding…' : 'Add Item'}
            </button>
          </div>
        </form>
      </div>
    </div>
  )
}

function Row({ label, children }) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 5, flex: 1 }}>
      <label style={{ fontSize: 11, color: C.muted, textTransform: 'uppercase', letterSpacing: '0.06em' }}>{label}</label>
      {children}
    </div>
  )
}

function TwoCol({ children }) {
  return <div style={{ display: 'flex', gap: 12 }}>{children}</div>
}

function Input(props) {
  return <input {...props} style={inputStyle} />
}

const inputStyle = {
  background: C.surface2, border: `1px solid ${C.border2}`, borderRadius: 7,
  color: C.text, padding: '8px 10px', fontSize: 13, width: '100%',
  fontFamily: "'Syne', sans-serif", outline: 'none',
}

const selectStyle = {
  ...inputStyle, cursor: 'pointer', appearance: 'auto',
}

const btnPrimary = {
  background: C.teal, color: C.bg, border: 'none', borderRadius: 8,
  padding: '9px 20px', fontWeight: 700, cursor: 'pointer', fontSize: 13,
  fontFamily: "'Syne', sans-serif",
}

const btnSecondary = {
  background: 'none', color: C.muted, border: `1px solid ${C.border2}`,
  borderRadius: 8, padding: '9px 16px', cursor: 'pointer', fontSize: 13,
  fontFamily: "'Syne', sans-serif",
}
