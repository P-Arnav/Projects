import { useState, useEffect } from 'react'
import { C, riskColor, riskLabel } from '../constants.js'
import { api } from '../api.js'

const fmt = (n, d = 2) => n == null ? '—' : Number(n).toFixed(d)
const fmtPct = (n) => n == null ? '—' : `${(n * 100).toFixed(0)}%`

// 7-day spoilage forecast: count items expiring on day d (not cumulative)
function buildForecast(items) {
  return Array.from({ length: 7 }, (_, d) => ({
    day: d,
    label: d === 0 ? 'Today' : `Day ${d}`,
    count: items.filter(i =>
      i.RSL != null &&
      i.RSL <= d + 0.5 &&
      i.RSL > d - 0.5
    ).length,
  }))
}

export default function Analytics({ items, dispatch, groceryItems }) {
  const scored = [...items].sort((a, b) => (b.fapf_score ?? -Infinity) - (a.fapf_score ?? -Infinity))
  const forecast = buildForecast(items)
  const maxCount = Math.max(...forecast.map(f => f.count), 1)
  const barColors = ['#34d399', '#6ee7b7', '#fbbf24', '#f97316', '#ff4d6d', '#ff4d6d', '#ff4d6d']

  const [restock, setRestock] = useState([])
  const [restockLoading, setRestockLoading] = useState(false)
  const [addedIds, setAddedIds] = useState(new Set())
  const [predictions, setPredictions] = useState([])
  const [predsLoading, setPredsLoading] = useState(false)

  useEffect(() => {
    setRestockLoading(true)
    api.getRestock()
      .then(setRestock)
      .catch(() => {})
      .finally(() => setRestockLoading(false))
  }, [items])

  useEffect(() => {
    setPredsLoading(true)
    api.getConsumptionPredictions()
      .then(data => setPredictions(Array.isArray(data) ? data : []))
      .catch(() => {})
      .finally(() => setPredsLoading(false))
  }, [])

  const handleAddToGrocery = async (suggestion) => {
    try {
      await api.addGrocery({
        name: suggestion.name,
        category: suggestion.category,
        quantity: 1,
        source: 'restock',
      })
      setAddedIds(prev => new Set([...prev, suggestion.name]))
    } catch {}
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 32 }}>

      {/* Restock Suggestions */}
      <section>
        <SectionTitle>Restock Suggestions</SectionTitle>
        {restockLoading ? (
          <div style={{ color: C.muted, fontSize: 13, padding: '16px 0' }}>Analysing pantry…</div>
        ) : restock.length === 0 ? (
          <div style={{
            background: C.surface, border: `1px solid ${C.border}`,
            borderRadius: 12, padding: '20px 24px',
            color: C.muted, fontSize: 13,
          }}>
            No restock needed — your pantry looks good.
          </div>
        ) : (
          <div style={{
            background: C.surface, border: `1px solid ${C.border}`,
            borderRadius: 12, overflow: 'hidden',
          }}>
            {restock.map((s, i) => {
              const alreadyIn = groceryItems.some(g => g.name.toLowerCase() === s.name.toLowerCase() && !g.checked)
              const justAdded = addedIds.has(s.name)
              const priorityColor = s.priority === 'urgent' ? C.critical : C.warn
              return (
                <div key={i} style={{
                  display: 'flex', alignItems: 'center', gap: 14,
                  padding: '12px 20px',
                  borderTop: i === 0 ? 'none' : `1px solid ${C.border}`,
                  background: i % 2 === 0 ? 'transparent' : C.surface2 + '44',
                }}>
                  <span style={{
                    fontSize: 10, fontWeight: 700, color: priorityColor,
                    background: priorityColor + '22', borderRadius: 4,
                    padding: '2px 8px', letterSpacing: '0.05em', flexShrink: 0,
                  }}>
                    {s.priority === 'urgent' ? 'URGENT' : 'LOW STOCK'}
                  </span>
                  <div style={{ flex: 1, minWidth: 0 }}>
                    <span style={{ color: C.text, fontWeight: 600, fontSize: 14 }}>{s.name}</span>
                    <span style={{ color: C.muted, fontSize: 12, marginLeft: 8 }}>{s.category}</span>
                    <div style={{ color: C.muted, fontSize: 12, marginTop: 2 }}>{s.reason}</div>
                  </div>
                  {s.p_spoil != null && (
                    <span style={{ color: riskColor(s.p_spoil), fontSize: 12, fontFamily: "'JetBrains Mono', monospace", flexShrink: 0 }}>
                      {fmtPct(s.p_spoil)}
                    </span>
                  )}
                  <button
                    onClick={() => handleAddToGrocery(s)}
                    disabled={alreadyIn || justAdded}
                    style={{
                      background: (alreadyIn || justAdded) ? C.surface2 : C.teal + '18',
                      border: `1px solid ${(alreadyIn || justAdded) ? C.border : C.teal}`,
                      color: (alreadyIn || justAdded) ? C.muted : C.teal,
                      borderRadius: 6, padding: '5px 12px',
                      cursor: (alreadyIn || justAdded) ? 'default' : 'pointer',
                      fontSize: 12, fontFamily: "'Syne', sans-serif",
                      fontWeight: 600, flexShrink: 0,
                    }}
                  >
                    {alreadyIn || justAdded ? '✓ In list' : '+ Grocery'}
                  </button>
                </div>
              )
            })}
          </div>
        )}
      </section>

      {/* 7-Day Forecast Chart */}
      <section>
        <SectionTitle>7-Day Spoilage Forecast</SectionTitle>
        <div style={{
          background: C.surface, border: `1px solid ${C.border}`,
          borderRadius: 12, padding: '24px 28px',
        }}>
          <div style={{ display: 'flex', alignItems: 'flex-end', gap: 12, height: 140 }}>
            {forecast.map(({ day, label, count }) => {
              const h = count === 0 ? 4 : Math.max(4, (count / maxCount) * 120)
              return (
                <div key={day} style={{ flex: 1, display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 6 }}>
                  <div style={{
                    fontSize: 11, color: C.muted,
                    fontFamily: "'JetBrains Mono', monospace",
                  }}>{count || ''}</div>
                  <div style={{
                    width: '100%', height: h,
                    background: barColors[day],
                    borderRadius: '4px 4px 2px 2px',
                    opacity: count === 0 ? 0.2 : 1,
                    transition: 'height 0.5s ease',
                  }} />
                  <div style={{ fontSize: 11, color: C.muted, whiteSpace: 'nowrap' }}>{label}</div>
                </div>
              )
            })}
          </div>
          <div style={{ fontSize: 12, color: C.muted, marginTop: 12 }}>
            Items expiring on each day
          </div>
        </div>
      </section>

      {/* Consumption Predictions */}
      <section>
        <SectionTitle>Consumption Behaviour & Predictions</SectionTitle>
        {predsLoading ? (
          <div style={{ color: C.muted, fontSize: 13, padding: '16px 0' }}>Analysing consumption history…</div>
        ) : predictions.length === 0 ? (
          <div style={{
            background: C.surface, border: `1px solid ${C.border}`,
            borderRadius: 12, padding: '20px 24px', color: C.muted, fontSize: 13,
          }}>
            No consumption history yet — predictions appear after items are consumed.
          </div>
        ) : (
          <div style={{ background: C.surface, border: `1px solid ${C.border}`, borderRadius: 12, overflow: 'hidden' }}>
            <table style={{ width: '100%', borderCollapse: 'collapse' }}>
              <thead>
                <tr style={{ background: C.surface2 }}>
                  {['Item', 'Category', 'Times', 'Avg Interval', 'Weekly Rate', 'Next In', 'Confidence'].map(h => (
                    <Th key={h}>{h}</Th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {predictions.map((p, i) => {
                  const nextColor = p.predicted_next_days == null ? C.muted
                    : p.predicted_next_days < 0 ? C.critical
                    : p.predicted_next_days < 3 ? C.warn
                    : C.safe
                  const confColor = p.confidence === 'HIGH' ? C.safe
                    : p.confidence === 'MEDIUM' ? C.warn
                    : C.muted
                  return (
                    <tr key={p.name} style={{
                      borderTop: `1px solid ${C.border}`,
                      background: i % 2 === 0 ? 'transparent' : C.surface2 + '55',
                    }}>
                      <Td bold>{p.name}</Td>
                      <Td muted>{p.category}</Td>
                      <Td mono>{p.times_consumed}×</Td>
                      <Td mono muted>
                        {p.avg_interval_days != null ? `${p.avg_interval_days}d` : '—'}
                      </Td>
                      <Td mono muted>
                        {p.weekly_rate != null ? `${p.weekly_rate}/wk` : '—'}
                      </Td>
                      <Td mono color={nextColor}>
                        {p.predicted_next_days == null ? '—'
                          : p.predicted_next_days < 0 ? `${Math.abs(p.predicted_next_days)}d ago`
                          : `${p.predicted_next_days}d`}
                      </Td>
                      <Td>
                        <span style={{
                          background: confColor + '22', color: confColor,
                          borderRadius: 4, padding: '2px 7px',
                          fontSize: 10, fontWeight: 700,
                          fontFamily: "'JetBrains Mono', monospace",
                          letterSpacing: '0.05em',
                        }}>
                          {p.confidence}
                        </span>
                      </Td>
                    </tr>
                  )
                })}
              </tbody>
            </table>
          </div>
        )}
      </section>

      {/* FAPF Priority Table */}
      <section>
        <SectionTitle>FAPF Priority Ranking</SectionTitle>
        {scored.length === 0 ? (
          <div style={{ color: C.muted, textAlign: 'center', padding: 40 }}>
            No items scored yet — add items and wait for the settle timer.
          </div>
        ) : (
          <div style={{
            background: C.surface, border: `1px solid ${C.border}`,
            borderRadius: 12, overflow: 'hidden',
          }}>
            <table style={{ width: '100%', borderCollapse: 'collapse' }}>
              <thead>
                <tr style={{ background: C.surface2 }}>
                  {['#', 'Name', 'Category', 'P_spoil', 'RSL', 'FAPF Score', 'Risk'].map(h => (
                    <Th key={h}>{h}</Th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {scored.map((item, i) => {
                  const risk = riskColor(item.P_spoil)
                  return (
                    <tr key={item.item_id} style={{
                      borderTop: `1px solid ${C.border}`,
                      background: i % 2 === 0 ? 'transparent' : C.surface2 + '55',
                    }}>
                      <Td mono muted>{i + 1}</Td>
                      <Td bold>{item.name}</Td>
                      <Td muted>{item.category}</Td>
                      <Td mono color={risk}>{fmtPct(item.P_spoil)}</Td>
                      <Td mono color={item.RSL != null && item.RSL < 1 ? C.critical : undefined}>
                        {item.RSL == null ? '—' : `${fmt(item.RSL, 1)}d`}
                      </Td>
                      <Td mono>{fmt(item.fapf_score, 3)}</Td>
                      <Td>
                        <span style={{
                          background: risk + '22', color: risk,
                          borderRadius: 4, padding: '2px 7px',
                          fontSize: 10, fontWeight: 700,
                          fontFamily: "'JetBrains Mono', monospace",
                          letterSpacing: '0.05em',
                        }}>
                          {riskLabel(item.P_spoil)}
                        </span>
                      </Td>
                    </tr>
                  )
                })}
              </tbody>
            </table>
          </div>
        )}
      </section>
    </div>
  )
}

function SectionTitle({ children }) {
  return (
    <div style={{ fontSize: 16, fontWeight: 700, color: C.text, marginBottom: 14 }}>
      {children}
    </div>
  )
}

function Th({ children }) {
  return (
    <th style={{
      padding: '10px 16px', textAlign: 'left', fontSize: 11, fontWeight: 600,
      color: C.muted, textTransform: 'uppercase', letterSpacing: '0.06em',
      fontFamily: "'Syne', sans-serif",
    }}>
      {children}
    </th>
  )
}

function Td({ children, mono, bold, muted, color }) {
  return (
    <td style={{
      padding: '10px 16px', fontSize: 13,
      fontFamily: mono ? "'JetBrains Mono', monospace" : "'Syne', sans-serif",
      fontWeight: bold ? 600 : 400,
      color: color ?? (muted ? C.muted : C.text),
    }}>
      {children}
    </td>
  )
}
