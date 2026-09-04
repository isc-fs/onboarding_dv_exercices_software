/**
 * Mission Control API client helpers.
 *
 * The backend can run in one of two modes:
 *   1. Unauthenticated (no IFSSIM_MC_API_KEY env var). Every call
 *      works without a key; the modal never appears.
 *   2. Authenticated (IFSSIM_MC_API_KEY set). Every mutating call
 *      needs the `X-API-Key` header and /ws/telemetry needs
 *      `?api_key=…` on the handshake.
 *
 * The frontend stores the key in localStorage. If a fetch comes back
 * 401, the App component prompts via `promptForApiKey()` (see App.tsx)
 * and retries.
 */

const STORAGE_KEY = 'mc_api_key'

export function getApiKey(): string {
  try {
    return localStorage.getItem(STORAGE_KEY) ?? ''
  } catch {
    return ''
  }
}

export function setApiKey(key: string): void {
  try {
    if (key) localStorage.setItem(STORAGE_KEY, key)
    else localStorage.removeItem(STORAGE_KEY)
  } catch {
    // localStorage can throw on private-mode Safari, full disk, or
    // when storage is disabled by enterprise policy. We can survive
    // any of those — the user just has to re-enter the API key on
    // every page load. Silently ignored to avoid throwing from a
    // setter call site.
  }
}

/**
 * Wrapped fetch that attaches `X-API-Key` when we have one. On 401,
 * re-resolves the key via the `onUnauthorized` callback (if provided)
 * and retries once so the user doesn't have to trigger the call again
 * after entering the key.
 */
export async function apiFetch(
  input: RequestInfo | URL,
  init: RequestInit = {},
  onUnauthorized?: () => Promise<string | null>
): Promise<Response> {
  const attempt = async (key: string): Promise<Response> => {
    const headers = new Headers(init.headers ?? {})
    if (key) headers.set('X-API-Key', key)
    return fetch(input, { ...init, headers })
  }

  let res = await attempt(getApiKey())
  if (res.status === 401 && onUnauthorized) {
    const newKey = await onUnauthorized()
    if (newKey) {
      setApiKey(newKey)
      res = await attempt(newKey)
    }
  }
  return res
}

/**
 * Compose a WebSocket URL with the `api_key` query parameter. Safe to
 * call with no key — the backend only enforces when configured.
 */
export function apiWsUrl(base: string): string {
  const key = getApiKey()
  if (!key) return base
  const sep = base.includes('?') ? '&' : '?'
  return `${base}${sep}api_key=${encodeURIComponent(key)}`
}

/**
 * Parse a fetch {@link Response} as JSON. If the body is HTML (e.g. nginx
 * 502 page), empty, or non‑JSON, throws an error that includes HTTP status
 * and a short text snippet — avoids opaque `JSON.parse` errors in the console.
 */
export async function readJsonResponse<T = unknown>(res: Response): Promise<T> {
  const text = await res.text()
  const trimmed = text.trim()
  if (!trimmed) {
    if (!res.ok) {
      throw new Error(`HTTP ${res.status} ${res.statusText} (empty body)`)
    }
    throw new Error('Empty response body')
  }
  try {
    return JSON.parse(trimmed) as T
  } catch {
    throw new Error(
      `HTTP ${res.status}: response was not JSON (${trimmed.slice(0, 160)}${trimmed.length > 160 ? '…' : ''})`,
    )
  }
}

/**
 * Default `onUnauthorized` handler: prompt the user for the key via
 * `window.prompt`, persist it, and trigger a reload so the WebSocket
 * reconnects with the new credentials. Components can pass their own
 * to apiFetch for a richer UX.
 */
export function promptForApiKey(): Promise<string | null> {
  return new Promise((resolve) => {
    const current = getApiKey()
    const entered = window.prompt(
      'Mission Control requires an API key. Paste the value of IFSSIM_MC_API_KEY from the backend .env:',
      current
    )
    if (entered === null) {
      resolve(null)
      return
    }
    const trimmed = entered.trim()
    if (!trimmed) {
      resolve(null)
      return
    }
    setApiKey(trimmed)
    resolve(trimmed)
  })
}
