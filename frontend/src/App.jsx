import { useState, useMemo, useRef, useCallback } from 'react'
import './App.css'

function App() {
  const [file, setFile] = useState(null)
  const [preview, setPreview] = useState(null)
  const [age, setAge] = useState(50)
  const [gender, setGender] = useState('M')
  const [view, setView] = useState('PA')
  const [followup, setFollowup] = useState(0)
  const [results, setResults] = useState(null)
  const [loading, setLoading] = useState(false)
  const [dragOver, setDragOver] = useState(false)

  const fileInputRef = useRef(null)
  const resultsRef = useRef(null)

  const history = useMemo(() => {
    const g = gender === 'M' ? 'male' : 'female'
    let h = `Patient is a ${age}-year-old ${g}. Chest X-ray taken in ${view} view. `
    h += followup === 0 ? 'This is the initial presentation.' : `Follow-up visit number ${followup}.`
    return h
  }, [age, gender, view, followup])

  const handleFile = useCallback((f) => {
    if (!f || !f.type.startsWith('image/')) return
    setFile(f)
    const reader = new FileReader()
    reader.onload = (e) => setPreview(e.target.result)
    reader.readAsDataURL(f)
  }, [])

  const clearFile = useCallback((e) => {
    e.stopPropagation()
    setFile(null)
    setPreview(null)
    if (fileInputRef.current) fileInputRef.current.value = ''
  }, [])

  const handlePredict = useCallback(async () => {
    if (!file) return
    setLoading(true)
    setResults(null)

    const formData = new FormData()
    formData.append('file', file)
    formData.append('age', age)
    formData.append('gender', gender)
    formData.append('view', view)
    formData.append('followup', followup)

    try {
      const res = await fetch('/predict', { method: 'POST', body: formData })
      if (!res.ok) throw new Error(`Server error: ${res.status}`)
      const data = await res.json()
      setResults(data)
      setTimeout(() => resultsRef.current?.scrollIntoView({ behavior: 'smooth', block: 'start' }), 100)
    } catch (err) {
      alert(`Error: ${err.message}`)
    } finally {
      setLoading(false)
    }
  }, [file, age, gender, view, followup])

  return (
    <>
      {/* Navbar */}
      <nav className="navbar">
        <div className="nav-brand">
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="url(#grad)" strokeWidth="2">
            <defs>
              <linearGradient id="grad" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#4f8ef7" />
                <stop offset="100%" stopColor="#7c3aed" />
              </linearGradient>
            </defs>
            <path d="M22 12h-4l-3 9L9 3l-3 9H2" />
          </svg>
          <span>LungScan AI</span>
        </div>
        <div className="nav-links">
          <a href="#how-it-works">How It Works</a>
          <a href="#diagnose" className="nav-cta">Start Diagnosis</a>
        </div>
      </nav>

      {/* Hero */}
      <section className="hero">
        <div className="hero-content">
          <div className="hero-badge">Multimodal AI Diagnosis</div>
          <h1>Intelligent Chest X-ray<br />Disease Detection</h1>
          <p className="hero-desc">
            Upload a chest X-ray and enter patient details. Our AI model combines
            visual analysis with clinical history to detect <strong>14 lung diseases</strong> with
            attention-based explainability.
          </p>
          <a href="#diagnose" className="hero-btn">Start Diagnosis</a>
        </div>
      </section>

      {/* How It Works */}
      <section className="how-section" id="how-it-works">
        <h2>How It Works</h2>
        <div className="steps-grid">
          <div className="step-card">
            <div className="step-num">1</div>
            <div className="step-icon">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
                <polyline points="17 8 12 3 7 8" />
                <line x1="12" y1="3" x2="12" y2="15" />
              </svg>
            </div>
            <h3>Upload X-ray</h3>
            <p>Upload a chest X-ray image (PNG or JPG format)</p>
          </div>
          <div className="step-card">
            <div className="step-num">2</div>
            <div className="step-icon">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                <path d="M16 21v-2a4 4 0 0 0-4-4H6a4 4 0 0 0-4 4v2" />
                <circle cx="9" cy="7" r="4" />
                <path d="M22 21v-2a4 4 0 0 0-3-3.87" />
                <path d="M16 3.13a4 4 0 0 1 0 7.75" />
              </svg>
            </div>
            <h3>Enter Patient Info</h3>
            <p>Provide age, gender, X-ray view, and visit details</p>
          </div>
          <div className="step-card">
            <div className="step-num">3</div>
            <div className="step-icon">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" />
                <polyline points="14 2 14 8 20 8" />
                <line x1="16" y1="13" x2="8" y2="13" />
                <line x1="16" y1="17" x2="8" y2="17" />
              </svg>
            </div>
            <h3>Get Results</h3>
            <p>View disease predictions, attention maps, and compatibility scores</p>
          </div>
        </div>
      </section>

      {/* Diagnosis */}
      <section className="diagnose-section" id="diagnose">
        <h2>Diagnosis</h2>
        <div className="diagnose-grid">
          {/* Input Panel */}
          <div className="card">
            <div className="card-header">
              <h3>Patient Input</h3>
              <span className="card-badge">Step 1 & 2</span>
            </div>

            {/* Upload */}
            <div
              className={`upload-area${preview ? ' has-image' : ''}${dragOver ? ' drag-over' : ''}`}
              onClick={() => !preview && fileInputRef.current?.click()}
              onDragOver={(e) => { e.preventDefault(); setDragOver(true) }}
              onDragLeave={() => setDragOver(false)}
              onDrop={(e) => { e.preventDefault(); setDragOver(false); handleFile(e.dataTransfer.files[0]) }}
            >
              <input
                ref={fileInputRef}
                type="file"
                accept="image/*"
                hidden
                onChange={(e) => handleFile(e.target.files[0])}
              />
              {preview ? (
                <>
                  <img src={preview} alt="X-ray preview" className="preview-img" />
                  <button className="clear-btn" onClick={clearFile}>&times;</button>
                </>
              ) : (
                <div className="upload-placeholder">
                  <svg width="44" height="44" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                    <rect x="3" y="3" width="18" height="18" rx="2" ry="2" />
                    <circle cx="8.5" cy="8.5" r="1.5" />
                    <polyline points="21 15 16 10 5 21" />
                  </svg>
                  <p>Click or drag & drop a chest X-ray</p>
                  <span>PNG, JPG supported</span>
                </div>
              )}
            </div>

            {/* Patient form */}
            <div className="form-grid">
              <div className="field">
                <label>Age</label>
                <input type="number" min="0" max="120" value={age} onChange={(e) => setAge(Number(e.target.value) || 0)} />
              </div>
              <div className="field">
                <label>Gender</label>
                <select value={gender} onChange={(e) => setGender(e.target.value)}>
                  <option value="M">Male</option>
                  <option value="F">Female</option>
                </select>
              </div>
              <div className="field">
                <label>X-ray View</label>
                <select value={view} onChange={(e) => setView(e.target.value)}>
                  <option value="PA">PA (Posterior-Anterior)</option>
                  <option value="AP">AP (Anterior-Posterior)</option>
                </select>
              </div>
              <div className="field">
                <label>Follow-up Visit #</label>
                <input type="number" min="0" max="50" value={followup} onChange={(e) => setFollowup(Number(e.target.value) || 0)} />
                <span className="field-hint">0 = initial presentation</span>
              </div>
            </div>

            {/* History preview */}
            <div className="history-preview">
              <label>Generated Clinical History</label>
              <p>{history}</p>
            </div>

            <button className="predict-btn" disabled={!file || loading} onClick={handlePredict}>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <circle cx="11" cy="11" r="8" /><line x1="21" y1="21" x2="16.65" y2="16.65" />
              </svg>
              Analyse X-ray
            </button>
          </div>

          {/* Results Panel */}
          <div className="card results-card" ref={resultsRef}>
            <div className="card-header">
              <h3>Diagnosis Results</h3>
              <span className="card-badge">Step 3</span>
            </div>

            {!results ? (
              <div className="empty-state">
                <svg width="56" height="56" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1">
                  <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" />
                  <polyline points="14 2 14 8 20 8" />
                </svg>
                <p>Upload an X-ray and click <strong>Analyse</strong> to see results</p>
              </div>
            ) : (
              <div className="fade-in">
                {/* Compatibility */}
                <div className="compat-section">
                  <div className="compat-label">
                    <span>Image-History Compatibility</span>
                    <span className="compat-value">{results.compatibility.toFixed(3)}</span>
                  </div>
                  <div className="bar-track">
                    <div className="bar-fill" style={{ width: `${Math.round(results.compatibility * 100)}%` }} />
                  </div>
                </div>

                {/* Heatmap */}
                <div className="heatmap-section">
                  <h4 className="section-title">Attention Heatmap</h4>
                  <div className="heatmap-wrapper">
                    <img src={`data:image/png;base64,${results.heatmap_base64}`} alt="Attention heatmap" />
                  </div>
                </div>

                {/* Predictions */}
                <div className="predictions-section">
                  <h4 className="section-title">Disease Predictions</h4>
                  <div className="predictions-header">
                    <span>Disease</span>
                    <span>Probability</span>
                    <span>Status</span>
                  </div>
                  <div className="predictions-list">
                    {results.predictions.map((pred) => (
                      <div key={pred.disease} className={`pred-row${pred.predicted ? ' is-positive' : ''}`}>
                        <span className="pred-name">{pred.disease.replace(/_/g, ' ')}</span>
                        <div className="pred-prob-cell">
                          <div className="pred-bar-track">
                            <div
                              className={`pred-bar-fill ${pred.predicted ? 'positive' : 'negative'}`}
                              style={{ width: `${Math.round(pred.prob * 100)}%` }}
                            />
                          </div>
                          <span className="pred-prob">{(pred.prob * 100).toFixed(1)}%</span>
                        </div>
                        <span className={`pred-badge ${pred.predicted ? 'yes' : 'no'}`}>
                          {pred.predicted ? 'Detected' : 'Normal'}
                        </span>
                      </div>
                    ))}
                  </div>
                </div>

                {/* Tokens */}
                <div>
                  <h4 className="section-title">Key Clinical Tokens</h4>
                  <div className="tokens-list">
                    {results.top_history_tokens?.length > 0 ? (
                      results.top_history_tokens.map((t, i) => (
                        <span key={i} className={`token-chip${i < 5 ? ' top' : ''}`} title={`attention: ${t.weight.toFixed(6)}`}>
                          {t.token}
                        </span>
                      ))
                    ) : (
                      <span style={{ color: 'var(--muted)', fontSize: '0.85rem' }}>No history tokens</span>
                    )}
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer>
        <p>LungScan AI &mdash; History-Conditioned Cross-Modal Attention Framework</p>
        <p className="footer-sub">Built on NIH ChestXray14 &middot; DenseNet-121 + Bio_ClinicalBERT &middot; For research use only</p>
      </footer>

      {/* Loading */}
      {loading && (
        <div className="loading-overlay">
          <div className="loading-card">
            <div className="spinner" />
            <p className="loading-title">Analysing X-ray</p>
            <p className="loading-sub">Running multimodal inference...</p>
          </div>
        </div>
      )}
    </>
  )
}

export default App
