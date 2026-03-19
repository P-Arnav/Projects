import { useState, useEffect } from 'react'
import { C } from '../constants.js'
import { api } from '../api.js'

export default function Recipes({ items }) {
  const [recipes, setRecipes] = useState([])
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState(null)
  const [expanded, setExpanded] = useState(null)
  const [cooking, setCooking] = useState(null)
  const [cooked, setCooked] = useState(new Set())

  const load = async () => {
    setLoading(true)
    setError(null)
    try {
      const data = await api.getRecipes()
      setRecipes(data)
    } catch {
      setError('Could not load recipes. Check internet connection.')
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => { load() }, [])

  const handleCook = async (recipe) => {
    setCooking(recipe.meal_id)
    // Match pantry items to this recipe's ingredient list
    const matchingIds = items
      .filter(item =>
        recipe.ingredients.some(ing => ing.toLowerCase().includes(item.name.toLowerCase()))
      )
      .map(i => i.item_id)

    try {
      await api.cookRecipe(recipe.meal_id, matchingIds)
      setCooked(prev => new Set([...prev, recipe.meal_id]))
    } catch {
      // silent — WS will reflect item changes
    } finally {
      setCooking(null)
    }
  }

  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 20 }}>
        <div style={{ fontSize: 18, fontWeight: 700, color: C.text }}>Recipe Suggestions</div>
        <button onClick={load} disabled={loading} style={{
          background: 'none', border: `1px solid ${C.border2}`, color: C.muted,
          borderRadius: 8, padding: '6px 14px', cursor: 'pointer', fontSize: 12,
          fontFamily: "'Syne', sans-serif",
        }}>
          {loading ? 'Loading…' : 'Refresh'}
        </button>
      </div>

      <div style={{ fontSize: 13, color: C.muted, marginBottom: 20 }}>
        Recipes suggested based on items currently in your pantry, via TheMealDB.
      </div>

      {loading && (
        <div style={{ textAlign: 'center', color: C.muted, padding: '60px 0', fontSize: 14 }}>
          Finding recipes based on your pantry…
        </div>
      )}

      {error && (
        <div style={{ textAlign: 'center', color: C.critical, padding: '40px 0', fontSize: 14 }}>{error}</div>
      )}

      {!loading && !error && recipes.length === 0 && (
        <div style={{ textAlign: 'center', color: C.muted, padding: '60px 0', fontSize: 14 }}>
          <div style={{ fontSize: 36, marginBottom: 12 }}>🍳</div>
          No recipes found. Add items to your pantry first.
        </div>
      )}

      <div style={{
        display: 'grid',
        gridTemplateColumns: 'repeat(auto-fill, minmax(300px, 1fr))',
        gap: 16,
      }}>
        {recipes.map(recipe => (
          <RecipeCard
            key={recipe.meal_id}
            recipe={recipe}
            expanded={expanded === recipe.meal_id}
            onToggle={() => setExpanded(expanded === recipe.meal_id ? null : recipe.meal_id)}
            onCook={() => handleCook(recipe)}
            cooking={cooking === recipe.meal_id}
            cooked={cooked.has(recipe.meal_id)}
          />
        ))}
      </div>
    </div>
  )
}

function RecipeCard({ recipe, expanded, onToggle, onCook, cooking, cooked }) {
  return (
    <div style={{
      background: C.surface, border: `1px solid ${cooked ? C.safe : C.border}`,
      borderRadius: 12, overflow: 'hidden',
      transition: 'border-color 0.3s',
    }}>
      {recipe.thumbnail && (
        <img
          src={recipe.thumbnail} alt={recipe.name}
          style={{ width: '100%', height: 160, objectFit: 'cover', display: 'block' }}
        />
      )}
      <div style={{ padding: '16px' }}>
        <div style={{ fontWeight: 700, color: C.text, fontSize: 15, marginBottom: 8 }}>
          {recipe.name}
        </div>
        <div style={{ display: 'flex', gap: 6, marginBottom: 12, flexWrap: 'wrap' }}>
          {recipe.category && <Badge color={C.blue}>{recipe.category}</Badge>}
          {recipe.area && <Badge color={C.muted}>{recipe.area}</Badge>}
        </div>

        <button onClick={onToggle} style={{
          background: 'none', border: 'none', color: C.teal,
          cursor: 'pointer', fontSize: 12, padding: 0,
          fontFamily: "'Syne', sans-serif", marginBottom: expanded ? 12 : 0,
        }}>
          {expanded ? 'Hide details ▲' : 'Show details ▼'}
        </button>

        {expanded && (
          <div>
            {recipe.ingredients.length > 0 && (
              <>
                <div style={{ fontSize: 11, color: C.muted, fontWeight: 600, marginBottom: 6, letterSpacing: '0.06em' }}>
                  INGREDIENTS
                </div>
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: 4, marginBottom: 12 }}>
                  {recipe.ingredients.map((ing, i) => (
                    <span key={i} style={{
                      background: C.surface2, color: C.text,
                      borderRadius: 4, padding: '2px 8px', fontSize: 11,
                      border: `1px solid ${C.border}`,
                    }}>
                      {ing}
                    </span>
                  ))}
                </div>
              </>
            )}
            {recipe.instructions && (
              <>
                <div style={{ fontSize: 11, color: C.muted, fontWeight: 600, marginBottom: 6, letterSpacing: '0.06em' }}>
                  INSTRUCTIONS (preview)
                </div>
                <div style={{ fontSize: 12, color: C.text, lineHeight: 1.65, opacity: 0.8 }}>
                  {recipe.instructions}…
                </div>
              </>
            )}
          </div>
        )}

        <button onClick={onCook} disabled={cooking || cooked} style={{
          marginTop: 14, width: '100%',
          background: cooked ? C.safe + '22' : cooking ? C.surface2 : C.teal + '18',
          border: `1px solid ${cooked ? C.safe : C.teal}`,
          color: cooked ? C.safe : C.teal,
          borderRadius: 8, padding: '9px', fontWeight: 700,
          cursor: cooking || cooked ? 'default' : 'pointer', fontSize: 13,
          fontFamily: "'Syne', sans-serif", transition: 'all 0.2s',
        }}>
          {cooked ? '✓ Cooked' : cooking ? 'Recording…' : 'Cook This Recipe'}
        </button>
      </div>
    </div>
  )
}

function Badge({ children, color }) {
  return (
    <span style={{
      background: color + '22', color,
      borderRadius: 4, padding: '2px 8px',
      fontSize: 10, fontWeight: 600, letterSpacing: '0.04em',
    }}>
      {children}
    </span>
  )
}
