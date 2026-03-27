export const C = {
  bg:       '#09090e',
  surface:  '#0f0f17',
  surface2: '#141420',
  border:   '#1c1c2a',
  border2:  '#242436',
  orange:   '#f97316',
  teal:     '#00d4aa',
  blue:     '#6366f1',
  text:     '#eeeef5',
  muted:    '#484868',
  critical: '#ef4444',
  warn:     '#f59e0b',
  safe:     '#22c55e',
}

export const riskColor = (p) =>
  p == null ? C.muted : p > 0.80 ? C.critical : p > 0.50 ? C.warn : C.safe

export const riskLabel = (p) =>
  p == null ? 'PENDING' : p > 0.80 ? 'CRITICAL' : p > 0.50 ? 'USE SOON' : 'SAFE'

export const CATEGORIES = [
  'dairy', 'protein', 'meat', 'vegetable', 'fruit', 'fish', 'cooked', 'beverage',
]

export const CAT_COLOR = {
  dairy:     '#6366f1',
  protein:   '#f97316',
  meat:      '#ef4444',
  vegetable: '#22c55e',
  fruit:     '#f59e0b',
  fish:      '#3b82f6',
  cooked:    '#a855f7',
  beverage:  '#00d4aa',
}

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
