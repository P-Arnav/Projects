import { useState, useRef } from 'react'
import { C, CATEGORIES } from '../constants.js'
import { api } from '../api.js'

export default function ReceiptModal({ onClose }) {
  const [step, setStep] = useState('upload')      // upload | review | adding | done
  const [file, setFile] = useState(null)
  const [preview, setPreview] = useState(null)
  const [scanning, setScanning] = useState(false)
  const [items, setItems] = useState([])
  const [rawText, setRawText] = useState('')
  const [error, setError] = useState(null)
  const [addedCount, setAddedCount] = useState(0)
  const fileRef = useRef()

  const handleFile = (f) => {
    setFile(f)
    setPreview(URL.createObjectURL(f))
    setError(null)
  }

  const handleDrop = (e) => {
    e.preventDefault()
    const f = e.dataTransfer.files[0]
    if (f) handleFile(f)
  }

  const handleScan = async () => {
    if (!file) return
    setScanning(true)
    setError(null)
    try {
      const res = await api.scanReceipt(file)
      setRawText(res.raw_text)
      setItems(res.items.map(item => ({ ...item, selected: true })))
      setStep('review')
    } catch (e) {
      const msg = await e.json?.().catch(() => null)
      setError(msg?.detail || 'Scan failed. Ensure Tesseract is installed and the image is clear.')
    } finally {
      setScanning(false)
    }
  }

  const handleAdd = async () => {
    setStep('adding')
    const selected = items.filter(i => i.selected)
    let count = 0
    for (const item of selected) {
      try {
        await api.postItem({
          name: item.name,
          category: item.category,
          quantity: item.quantity,
          shelf_life: item.shelf_life,
          estimated_cost: item.price ?? 0,
          storage_temp: 4.0,
          humidity: 50.0,
          location: '',
        })
        count++
      } catch {}
    }
    setAddedCount(count)
    setStep('done')
  }

  const toggleItem = (idx) => {
    setItems(prev => prev.map((it, i) => i === idx ? { ...it, selected: !it.selected } : it))
  }

  const updateItem = (idx, field, value) => {
    setItems(prev => prev.map((it, i) => i === idx ? { ...it, [field]: value } : it))
  }

  const selectedCount = items.filter(i => i.selected).length

  return (
    <div style={overlayStyle} onClick={e => e.target === e.currentTarget && onClose()}>
      <div style={modalStyle}>
        {/* Header */}
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 20 }}>
          <div style={{ fontSize: 16, fontWeight: 700, color: C.text }}>Upload Receipt</div>
          <button onClick={onClose} style={{ background: 'none', border: 'none', color: C.muted, cursor: 'pointer', fontSize: 22, lineHeight: 1 }}>×</button>
        </div>

        {/* Step: upload */}
        {step === 'upload' && (
          <>
            <div
              onDrop={handleDrop}
              onDragOver={e => e.preventDefault()}
              onClick={() => fileRef.current.click()}
              style={{
                border: `2px dashed ${preview ? C.teal : C.border2}`,
                borderRadius: 10, padding: '32px 20px', textAlign: 'center',
                cursor: 'pointer', marginBottom: 16,
                background: preview ? C.teal + '08' : 'none',
                transition: 'border-color 0.2s',
              }}
            >
              {preview ? (
                <img src={preview} alt="receipt preview" style={{ maxHeight: 220, maxWidth: '100%', borderRadius: 6 }} />
              ) : (
                <>
                  <div style={{ fontSize: 36, marginBottom: 10 }}>🧾</div>
                  <div style={{ color: C.text, fontSize: 14 }}>Drop receipt image here or click to browse</div>
                  <div style={{ color: C.muted, fontSize: 12, marginTop: 6 }}>JPG, PNG supported</div>
                </>
              )}
            </div>
            <input
              ref={fileRef} type="file" accept="image/*" style={{ display: 'none' }}
              onChange={e => e.target.files[0] && handleFile(e.target.files[0])}
            />
            {error && (
              <div style={{ color: C.critical, fontSize: 13, marginBottom: 12, lineHeight: 1.5 }}>{error}</div>
            )}
            <button onClick={handleScan} disabled={!file || scanning} style={primaryBtnStyle}>
              {scanning ? 'Scanning…' : 'Scan Receipt'}
            </button>
          </>
        )}

        {/* Step: review */}
        {step === 'review' && (
          <>
            <div style={{ fontSize: 13, color: C.muted, marginBottom: 14 }}>
              Found <strong style={{ color: C.text }}>{items.length}</strong> food item(s). Select what to add to pantry:
            </div>

            <div style={{ maxHeight: 340, overflowY: 'auto', display: 'flex', flexDirection: 'column', gap: 7, marginBottom: 16 }}>
              {items.length === 0 ? (
                <div style={{ color: C.muted, textAlign: 'center', padding: 28, fontSize: 13 }}>
                  No food items detected. Try a clearer photo.
                </div>
              ) : (
                items.map((item, i) => (
                  <div key={i} style={{
                    display: 'flex', gap: 10, alignItems: 'center',
                    background: item.selected ? C.surface2 : C.surface,
                    border: `1px solid ${item.selected ? C.border2 : C.border}`,
                    borderRadius: 8, padding: '9px 12px',
                    opacity: item.selected ? 1 : 0.45,
                    transition: 'opacity 0.2s',
                  }}>
                    <input
                      type="checkbox" checked={item.selected} onChange={() => toggleItem(i)}
                      style={{ accentColor: C.teal, flexShrink: 0 }}
                    />
                    <div style={{ flex: 1, fontWeight: 500, color: C.text, fontSize: 13, minWidth: 0, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                      {item.name}
                    </div>
                    <select
                      value={item.category}
                      onChange={e => updateItem(i, 'category', e.target.value)}
                      style={miniSelectStyle}
                    >
                      {CATEGORIES.map(c => <option key={c} value={c}>{c}</option>)}
                    </select>
                    <input
                      type="number" min={1} value={item.quantity}
                      onChange={e => updateItem(i, 'quantity', Math.max(1, Number(e.target.value)))}
                      style={{ ...miniInputStyle, width: 46 }}
                    />
                    {item.price != null && (
                      <span style={{ fontSize: 11, color: C.muted, flexShrink: 0 }}>₹{item.price}</span>
                    )}
                  </div>
                ))
              )}
            </div>

            <div style={{ display: 'flex', gap: 10 }}>
              <button onClick={() => setStep('upload')} style={secondaryBtnStyle}>Back</button>
              <button
                onClick={handleAdd}
                disabled={selectedCount === 0}
                style={{ ...primaryBtnStyle, flex: 2 }}
              >
                Add {selectedCount} Item{selectedCount !== 1 ? 's' : ''} to Pantry
              </button>
            </div>
          </>
        )}

        {/* Step: adding */}
        {step === 'adding' && (
          <div style={{ textAlign: 'center', padding: '48px 0', color: C.muted, fontSize: 14 }}>
            Adding items to pantry…
          </div>
        )}

        {/* Step: done */}
        {step === 'done' && (
          <div style={{ textAlign: 'center', padding: '40px 0' }}>
            <div style={{ fontSize: 40, marginBottom: 12 }}>✓</div>
            <div style={{ color: C.safe, fontSize: 16, fontWeight: 700, marginBottom: 6 }}>
              {addedCount} item{addedCount !== 1 ? 's' : ''} added!
            </div>
            <div style={{ color: C.muted, fontSize: 13, marginBottom: 24 }}>
              They will be scored after the settle timer.
            </div>
            <button onClick={onClose} style={primaryBtnStyle}>Done</button>
          </div>
        )}
      </div>
    </div>
  )
}

const overlayStyle = {
  position: 'fixed', inset: 0, background: 'rgba(0,0,0,0.65)',
  display: 'flex', alignItems: 'center', justifyContent: 'center', zIndex: 1000,
}

const modalStyle = {
  background: C.surface, border: `1px solid #1a2e4a`, borderRadius: 14,
  padding: '24px 28px', width: '100%', maxWidth: 520,
  maxHeight: '90vh', overflowY: 'auto',
}

const primaryBtnStyle = {
  width: '100%', background: C.teal, color: C.bg,
  border: 'none', borderRadius: 8, padding: '11px 0',
  fontWeight: 700, cursor: 'pointer', fontSize: 14,
  fontFamily: "'Syne', sans-serif",
}

const secondaryBtnStyle = {
  flex: 1, background: 'none', color: C.muted,
  border: `1px solid ${C.border}`, borderRadius: 8, padding: '11px 0',
  cursor: 'pointer', fontSize: 13, fontFamily: "'Syne', sans-serif",
}

const miniSelectStyle = {
  background: C.bg, border: `1px solid #1a2e4a`, borderRadius: 4,
  color: '#e8f0fe', padding: '4px 6px', fontSize: 11,
  fontFamily: "'Syne', sans-serif", flexShrink: 0,
}

const miniInputStyle = {
  background: C.bg, border: `1px solid #1a2e4a`, borderRadius: 4,
  color: '#e8f0fe', padding: '4px 6px', fontSize: 11,
  textAlign: 'center', flexShrink: 0,
}
