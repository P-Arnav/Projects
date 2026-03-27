import { useEffect, useRef, useState } from 'react'
import { C } from '../constants.js'
import { api } from '../api.js'

// Modes: 'detect' = Grounding DINO full-scene, 'barcode' = BarcodeDetector live scan
export default function ScanModal({ onClose }) {
  const videoRef  = useRef(null)
  const canvasRef = useRef(null)
  const streamRef = useRef(null)
  const barcodeLoopRef = useRef(null)   // rAF handle for barcode scan loop

  const [mode,        setMode]        = useState('detect')   // 'detect' | 'barcode'
  const [status,      setStatus]      = useState('starting') // starting | ready | scanning | adding | done | error
  const [message,     setMessage]     = useState('')
  const [addedCount,  setAddedCount]  = useState(0)
  const [spoiledItems, setSpoiledItems] = useState([])
  const [lastBarcode, setLastBarcode]  = useState(null)    // {code, name, category}
  const [continuous,  setContinuous]  = useState(false)
  const continuousRef = useRef(false)
  const statusRef     = useRef('starting')
  const runningRef    = useRef(false)

  // ── Camera startup ─────────────────────────────────────────────────────────
  useEffect(() => {
    async function startCamera() {
      try {
        const tempStream = await navigator.mediaDevices.getUserMedia({ video: true })
        tempStream.getTracks().forEach(t => t.stop())

        const devices = await navigator.mediaDevices.enumerateDevices()
        const cameras = devices.filter(d => d.kind === 'videoinput')
        console.log('Cameras found:', cameras.map((c, i) => `[${i}] ${c.label}`))
        const deviceId = cameras[0]?.deviceId  // USB Camera (0c45:6366)

        const stream = await navigator.mediaDevices.getUserMedia({
          video: deviceId ? { deviceId: { exact: deviceId }, width: { ideal: 1280 }, height: { ideal: 720 } } : true,
        })
        streamRef.current = stream
        if (videoRef.current) videoRef.current.srcObject = stream
        setStatus('ready')
      } catch (err) {
        setStatus('error')
        setMessage(`Camera error: ${err.message}`)
      }
    }
    startCamera()
    return () => {
      streamRef.current?.getTracks().forEach(t => t.stop())
      if (barcodeLoopRef.current) cancelAnimationFrame(barcodeLoopRef.current)
    }
  }, [])

  // ── Barcode scanning loop (runs while mode === 'barcode' and status === 'ready') ──
  useEffect(() => {
    if (mode !== 'barcode' || status !== 'ready') {
      if (barcodeLoopRef.current) cancelAnimationFrame(barcodeLoopRef.current)
      return
    }

    let cancelled = false
    let lastScanned = ''

    if ('BarcodeDetector' in window) {
      // Native path — Chrome / Edge
      const detector = new window.BarcodeDetector({
        formats: ['ean_13', 'ean_8', 'upc_a', 'upc_e', 'code_128', 'qr_code', 'data_matrix'],
      })

      async function scan() {
        if (cancelled) return
        const video = videoRef.current
        if (video && video.readyState >= 2) {
          try {
            const barcodes = await detector.detect(video)
            if (barcodes.length > 0) {
              const code = barcodes[0].rawValue
              if (code !== lastScanned) { lastScanned = code; await handleBarcode(code) }
            }
          } catch { /* frame not ready */ }
        }
        barcodeLoopRef.current = requestAnimationFrame(scan)
      }
      barcodeLoopRef.current = requestAnimationFrame(scan)

    } else {
      // Fallback path — QuaggaJS canvas-based continuous scan
      import('@ericblade/quagga2').then(({ default: Quagga }) => {
        const canvas = document.createElement('canvas')

        async function decodeFrame() {
          const video = videoRef.current
          if (!video || video.readyState < 2 || video.videoWidth === 0) return null
          canvas.width  = video.videoWidth
          canvas.height = video.videoHeight
          canvas.getContext('2d').drawImage(video, 0, 0)
          return new Promise(resolve => {
            Quagga.decodeSingle({
              src: canvas.toDataURL('image/jpeg'),
              numOfWorkers: 0,
              locate: true,
              inputStream: { size: 640 },
              decoder: { readers: ['ean_reader', 'ean_8_reader', 'upc_reader', 'code_128_reader'] },
            }, result => resolve(result?.codeResult?.code || null))
          })
        }

        async function scanLoop() {
          while (!cancelled) {
            const code = await decodeFrame()
            if (code && code !== lastScanned) {
              lastScanned = code
              await handleBarcode(code)
              setTimeout(() => { lastScanned = '' }, 3000)
            }
            await new Promise(r => setTimeout(r, 300))
          }
        }
        scanLoop()
      }).catch(e => {
        setMessage(`Barcode scanning unavailable: ${e.message}`)
      })
    }

    return () => {
      cancelled = true
      if (barcodeLoopRef.current) cancelAnimationFrame(barcodeLoopRef.current)
    }
  }, [mode, status])   // eslint-disable-line react-hooks/exhaustive-deps

  async function handleBarcode(code) {
    setStatus('scanning')
    setMessage(`Found barcode ${code} — looking up…`)
    try {
      const result = await api.lookupBarcode(code)
      setLastBarcode({ code, ...result })

      await api.postItem({
        name:           result.name || code,
        category:       result.category,
        quantity:       1,
        shelf_life:     result.shelf_life,
        location:       '',
        estimated_cost: result.estimated_cost ?? 0,
        storage_temp:   4.0,
        humidity:       50.0,
      })

      setAddedCount(n => n + 1)
      setMessage(result.off_found
        ? `Added "${result.name}" (${result.category})`
        : `Added barcode ${code} with default values`)
      setStatus('ready')
    } catch {
      setMessage(`Lookup failed for ${code}. Continuing scan…`)
      setStatus('ready')
    }
  }

  // Keep statusRef in sync so the continuous loop can read it without stale closure
  useEffect(() => { statusRef.current = status }, [status])

  // ── Continuous detection loop ───────────────────────────────────────────────
  useEffect(() => {
    continuousRef.current = continuous
    if (!continuous) return
    let cancelled = false

    async function loop() {
      while (!cancelled && continuousRef.current) {
        if (statusRef.current === 'ready') {
          await captureAndDetectInner(true)
        }
        await new Promise(r => setTimeout(r, 2000))
      }
    }
    loop()
    return () => { cancelled = true }
  }, [continuous])  // eslint-disable-line react-hooks/exhaustive-deps

  // ── Grounding DINO capture ─────────────────────────────────────────────────
  async function captureAndDetect() {
    await captureAndDetectInner(false)
  }

  async function captureAndDetectInner(isContinuous) {
    if (runningRef.current) return
    runningRef.current = true
    const video  = videoRef.current
    const canvas = canvasRef.current
    if (!video || !canvas) { runningRef.current = false; return }

    canvas.width  = video.videoWidth
    canvas.height = video.videoHeight
    canvas.getContext('2d').drawImage(video, 0, 0)

    setStatus('scanning')
    setMessage('Running detection…')

    let result
    try {
      const blob = await new Promise(resolve => canvas.toBlob(resolve, 'image/jpeg', 0.92))
      result = await api.scanFridge(blob)
    } catch {
      setStatus('error')
      setMessage('Detection failed. Is the backend running with vision deps installed?')
      runningRef.current = false
      return
    }

    if (!result.items || result.items.length === 0) {
      setStatus('ready')
      setMessage('No items detected. Try capturing again.')
      runningRef.current = false
      return
    }

    // Warn about spoiled items in console; skip adding them
    const spoiled = result.items.filter(i => i.spoilage_detected)
    const fresh   = result.items.filter(i => !i.spoilage_detected)

    if (spoiled.length > 0) {
      console.warn('Spoiled items detected (not added):', spoiled.map(i => i.name))
    }

    setStatus('adding')
    setMessage(`Adding ${fresh.length} fresh item(s)…${spoiled.length > 0 ? ` Skipping ${spoiled.length} spoiled.` : ''}`)

    let added = 0
    for (const item of fresh) {
      try {
        await api.postItem({
          name: item.name, category: item.category, quantity: item.count,
          shelf_life: item.shelf_life, location: '', estimated_cost: item.estimated_cost ?? 0,
          storage_temp: 4.0, humidity: 50.0,
        })
        added++
      } catch { /* continue */ }
    }

    setAddedCount(n => n + added)
    setSpoiledItems(spoiled)
    runningRef.current = false
    if (isContinuous) {
      setStatus('ready')
      setMessage(added > 0
        ? `Added ${added} item(s) — scanning again…`
        : 'No new items — scanning again…')
    } else {
      setStatus('done')
      setMessage('')
      setTimeout(onClose, spoiled.length > 0 ? 4000 : 2000)
    }
  }

  // ── Render ─────────────────────────────────────────────────────────────────
  const busy = status === 'scanning' || status === 'adding'

  return (
    <div style={{
      position: 'fixed', inset: 0, background: '#00000099', zIndex: 100,
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }} onClick={e => e.target === e.currentTarget && onClose()}>
      <div style={{
        background: C.surface, border: `1px solid ${C.border2}`, borderRadius: 14,
        padding: 24, width: 520, maxWidth: '95vw',
      }}>
        {/* Header */}
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 14 }}>
          <div style={{ fontWeight: 700, fontSize: 17, color: C.text }}>Scan Fridge</div>
          <button onClick={onClose} style={{ background: 'none', border: 'none', color: C.muted, cursor: 'pointer', fontSize: 18 }}>✕</button>
        </div>

        {/* Mode tabs */}
        <div style={{ display: 'flex', gap: 8, marginBottom: 14 }}>
          {[['detect', 'Detect Items'], ['barcode', 'Scan Barcode']].map(([m, label]) => (
            <button key={m} onClick={() => { setMode(m); setMessage('') }} style={{
              background: mode === m ? C.teal + '22' : 'none',
              border: `1px solid ${mode === m ? C.teal : C.border2}`,
              color: mode === m ? C.teal : C.muted,
              borderRadius: 20, padding: '5px 14px', cursor: 'pointer', fontSize: 12,
              fontFamily: "'Syne', sans-serif", fontWeight: mode === m ? 700 : 400,
            }}>{label}</button>
          ))}
        </div>

        {/* Video feed */}
        <div style={{ position: 'relative', borderRadius: 10, overflow: 'hidden', background: C.surface2, marginBottom: 14 }}>
          <video ref={videoRef} autoPlay playsInline muted
            style={{ width: '100%', display: 'block', maxHeight: mode === 'detect' ? 420 : 220, objectFit: 'cover' }} />
          {status === 'starting' && (
            <div style={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', color: C.muted, fontSize: 13 }}>
              Starting camera…
            </div>
          )}
          {/* Continuous detect indicator */}
          {mode === 'detect' && continuous && (
            <div style={{ position: 'absolute', top: 10, left: 10, background: '#00000088', borderRadius: 6, padding: '4px 10px', color: C.teal, fontSize: 11, fontWeight: 700 }}>
              {status === 'scanning' || status === 'adding' ? '⏳ DETECTING…' : '● LIVE DETECT'}
            </div>
          )}
          {/* Barcode mode: scanning indicator */}
          {mode === 'barcode' && status === 'ready' && (
            <div style={{ position: 'absolute', top: 10, left: 10, background: '#00000088', borderRadius: 6, padding: '4px 10px', color: C.teal, fontSize: 11, fontWeight: 700 }}>
              SCANNING FOR BARCODES
            </div>
          )}
        </div>

        <canvas ref={canvasRef} style={{ display: 'none' }} />

        {/* Status messages */}
        {status === 'done' && (
          <div style={{ marginBottom: 12 }}>
            <div style={{ color: C.safe, fontSize: 14, textAlign: 'center', fontWeight: 600 }}>
              Added {addedCount} fresh item{addedCount !== 1 ? 's' : ''} to inventory.
            </div>
            {spoiledItems.length > 0 && (
              <div style={{ marginTop: 8, background: C.critical + '18', border: `1px solid ${C.critical}44`, borderRadius: 8, padding: '8px 12px' }}>
                <div style={{ color: C.critical, fontWeight: 700, fontSize: 12, marginBottom: 4 }}>SPOILAGE DETECTED — not added:</div>
                {spoiledItems.map(i => (
                  <div key={i.name} style={{ color: C.critical, fontSize: 12 }}>
                    {i.name} — {Math.round(i.spoilage_confidence * 100)}% spoiled
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
        {status === 'error' && (
          <div style={{ color: C.critical, fontSize: 13, marginBottom: 12 }}>{message}</div>
        )}
        {busy && (
          <div style={{ color: C.teal, fontSize: 13, marginBottom: 12 }}>{message}</div>
        )}
        {status === 'ready' && message && (
          <div style={{ color: C.safe, fontSize: 13, marginBottom: 12 }}>{message}</div>
        )}
        {/* Barcode session counter */}
        {mode === 'barcode' && addedCount > 0 && status !== 'done' && (
          <div style={{ color: C.muted, fontSize: 12, marginBottom: 8 }}>
            Items added this session: <span style={{ color: C.teal, fontWeight: 700 }}>{addedCount}</span>
          </div>
        )}

        {/* Actions */}
        <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 10, alignItems: 'center' }}>
          <button onClick={onClose} style={btnSecondary}>
            {addedCount > 0 ? 'Done' : 'Cancel'}
          </button>
          {mode === 'detect' && (
            <>
              <button
                onClick={() => setContinuous(v => !v)}
                disabled={status === 'done' || status === 'error'}
                style={{
                  ...btnSecondary,
                  borderColor: continuous ? C.teal : C.border2,
                  color: continuous ? C.teal : C.muted,
                }}>
                {continuous ? '⏹ Stop Live' : '▶ Live Detect'}
              </button>
              <button onClick={captureAndDetect} disabled={status !== 'ready' || continuous}
                style={{ ...btnPrimary, opacity: (status !== 'ready' || continuous) ? 0.5 : 1 }}>
                {status === 'scanning' ? 'Detecting…' : status === 'adding' ? 'Adding…' : 'Capture & Detect'}
              </button>
            </>
          )}
          {mode === 'barcode' && (
            <div style={{ color: C.muted, fontSize: 12, alignSelf: 'center' }}>
              Point camera at a barcode — auto-adds on detection
            </div>
          )}
        </div>
      </div>
    </div>
  )
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
