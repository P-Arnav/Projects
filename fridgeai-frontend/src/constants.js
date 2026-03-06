export const C = {
  bg:       '#070d1a',
  surface:  '#0c1628',
  surface2: '#101e33',
  border:   '#1a2e4a',
  border2:  '#243d5c',
  teal:     '#00d4aa',
  blue:     '#3b9eff',
  text:     '#e8f0fe',
  muted:    '#4a6080',
  critical: '#ff4d6d',
  warn:     '#fbbf24',
  safe:     '#34d399',
}

export const riskColor = (p) =>
  p == null ? C.muted : p > 0.80 ? C.critical : p > 0.50 ? C.warn : C.safe

export const riskLabel = (p) =>
  p == null ? 'PENDING' : p > 0.80 ? 'CRITICAL' : p > 0.50 ? 'USE SOON' : 'SAFE'

export const CATEGORIES = [
  'dairy', 'protein', 'meat', 'vegetable', 'fruit', 'fish', 'cooked', 'beverage',
]

export const ALERT_COLOR = {
  CRITICAL_ALERT:  C.critical,
  WARNING_ALERT:   C.warn,
  USE_TODAY_ALERT: C.teal,
}

export const ALERT_LABEL = {
  CRITICAL_ALERT:  'CRITICAL',
  WARNING_ALERT:   'WARNING',
  USE_TODAY_ALERT: 'USE TODAY',
}
