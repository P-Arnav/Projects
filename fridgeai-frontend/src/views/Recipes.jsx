import { useState, useEffect } from 'react'
import { C } from '../constants.js'
import { api } from '../api.js'

export default function Recipes({ items }) {
  const [recipes, setRecipes] = useState([])
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState(null)
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
    // Match pantry items to this recipe's used ingredients
    const matchingIds = items
      .filter(item =>
        recipe.used_ingredients.some(ing => ing.toLowerCase().includes(item.name.toLowerCase()))
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
        Recipes suggested based on items currently in your pantry, via Spoonacular.
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
            onCook={() => handleCook(recipe)}
            cooking={cooking === recipe.meal_id}
            cooked={cooked.has(recipe.meal_id)}
          />
        ))}
      </div>
    </div>
  )
}

function RecipeCard({ recipe, onCook, cooking, cooked }) {
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
        <div style={{ fontWeight: 700, color: C.text, fontSize: 15, marginBottom: 12 }}>
          {recipe.name}
        </div>

        {recipe.used_ingredients.length > 0 && (
          <div style={{ marginBottom: 10 }}>
            <div style={{ fontSize: 10, color: C.safe, fontWeight: 700, letterSpacing: '0.06em', marginBottom: 5 }}>
              IN YOUR FRIDGE
            </div>
            <div style={{ display: 'flex', flexWrap: 'wrap', gap: 4 }}>
              {recipe.used_ingredients.map((ing, i) => (
                <span key={i} style={{
                  background: C.safe + '18', color: C.safe,
                  borderRadius: 4, padding: '2px 8px', fontSize: 11,
                  border: `1px solid ${C.safe}33`,
                }}>
                  {ing}
                </span>
              ))}
            </div>
          </div>
        )}

        {recipe.missed_ingredients.length > 0 && (
          <div style={{ marginBottom: 14 }}>
            <div style={{ fontSize: 10, color: C.warn, fontWeight: 700, letterSpacing: '0.06em', marginBottom: 5 }}>
              STILL NEED
            </div>
            <div style={{ display: 'flex', flexWrap: 'wrap', gap: 4 }}>
              {recipe.missed_ingredients.map((ing, i) => (
                <span key={i} style={{
                  background: C.warn + '18', color: C.warn,
                  borderRadius: 4, padding: '2px 8px', fontSize: 11,
                  border: `1px solid ${C.warn}33`,
                }}>
                  {ing}
                </span>
              ))}
            </div>
          </div>
        )}

        <button onClick={onCook} disabled={cooking || cooked} style={{
          marginTop: 4, width: '100%',
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
