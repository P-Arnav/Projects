import { C, riskColor, riskLabel } from '../constants.js'

const fmt = (n, d = 2) => n == null ? '—' : Number(n).toFixed(d)
const fmtPct = (n) => n == null ? '—' : `${(n * 100).toFixed(0)}%`

// 7-day spoilage forecast: count items expiring by day d
function buildForecast(items) {
  return Array.from({ length: 7 }, (_, d) => ({
    day: d,
    label: d === 0 ? 'Today' : `Day ${d}`,
    count: items.filter(i => i.RSL != null && i.RSL <= d + 0.5).length,
  }))
}

export default function Analytics({ items }) {
  const scored = [...items].sort((a, b) => (b.fapf_score ?? -Infinity) - (a.fapf_score ?? -Infinity))
  const forecast = buildForecast(items)
  const maxCount = Math.max(...forecast.map(f => f.count), 1)

  const barColors = ['#34d399', '#6ee7b7', '#fbbf24', '#f97316', '#ff4d6d', '#ff4d6d', '#ff4d6d']

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 32 }}>

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
            Items with RSL expiring by each day
          </div>
        </div>
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
