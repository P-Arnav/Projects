-- Run this once in your Supabase project: Dashboard > SQL Editor > New query

-- 1. Households
CREATE TABLE IF NOT EXISTS public.households (
    household_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name         TEXT NOT NULL,
    created_at   TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- 2. User preferences (keyed to Supabase Auth user id)
CREATE TABLE IF NOT EXISTS public.user_prefs (
    user_id               UUID PRIMARY KEY REFERENCES auth.users(id) ON DELETE CASCADE,
    household_id          UUID REFERENCES public.households(household_id) ON DELETE SET NULL,
    auto_restock_enabled  BOOLEAN NOT NULL DEFAULT FALSE
);

-- 3. Row Level Security
--    The backend uses the service role key, which bypasses RLS automatically.
--    These policies protect direct client access if you ever expose the anon key.
ALTER TABLE public.households ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.user_prefs  ENABLE ROW LEVEL SECURITY;

DROP POLICY IF EXISTS "users_own_prefs_select" ON public.user_prefs;
CREATE POLICY "users_own_prefs_select" ON public.user_prefs
    FOR SELECT USING (auth.uid() = user_id);

DROP POLICY IF EXISTS "users_own_prefs_update" ON public.user_prefs;
CREATE POLICY "users_own_prefs_update" ON public.user_prefs
    FOR UPDATE USING (auth.uid() = user_id);

-- 4. (Optional) Disable email confirmation for local dev so register works immediately
--    Dashboard > Authentication > Settings > "Enable email confirmations" → OFF

-- App tables
CREATE TABLE IF NOT EXISTS public.items (
    item_id         TEXT PRIMARY KEY,
    name            TEXT NOT NULL,
    category        TEXT NOT NULL,
    quantity        INTEGER NOT NULL DEFAULT 1,
    entry_time      TEXT NOT NULL,
    shelf_life      INTEGER NOT NULL,
    location        TEXT NOT NULL DEFAULT '',
    estimated_cost  DOUBLE PRECISION NOT NULL DEFAULT 0.0,
    storage_temp    DOUBLE PRECISION NOT NULL DEFAULT 4.0,
    humidity        DOUBLE PRECISION NOT NULL DEFAULT 50.0,
    p_spoil         DOUBLE PRECISION,
    rsl             DOUBLE PRECISION,
    fapf_score      DOUBLE PRECISION,
    paif_action     TEXT,
    confidence_tier TEXT NOT NULL DEFAULT 'LOW',
    updated_at      TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS public.alerts (
    alert_id   TEXT PRIMARY KEY,
    item_id    TEXT NOT NULL,
    item_name  TEXT NOT NULL,
    alert_type TEXT NOT NULL,
    p_spoil    DOUBLE PRECISION,
    rsl        DOUBLE PRECISION,
    message    TEXT NOT NULL,
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS public.feedback (
    feedback_id         TEXT PRIMARY KEY,
    item_id             TEXT NOT NULL,
    category            TEXT NOT NULL,
    shelf_life_declared INTEGER NOT NULL,
    shelf_life_actual   DOUBLE PRECISION NOT NULL,
    correction          DOUBLE PRECISION NOT NULL,
    created_at          TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS public.grocery_items (
    grocery_id TEXT PRIMARY KEY,
    name       TEXT NOT NULL,
    category   TEXT NOT NULL DEFAULT 'vegetable',
    quantity   INTEGER NOT NULL DEFAULT 1,
    checked    INTEGER NOT NULL DEFAULT 0,
    source     TEXT NOT NULL DEFAULT 'manual',
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS public.consumption_history (
    id                  TEXT PRIMARY KEY,
    item_id             TEXT NOT NULL,
    item_name           TEXT NOT NULL,
    category            TEXT NOT NULL,
    quantity_consumed   INTEGER NOT NULL DEFAULT 1,
    reason              TEXT NOT NULL DEFAULT 'consumed',
    p_spoil_at_removal  DOUBLE PRECISION,
    consumed_at         TEXT NOT NULL
);
