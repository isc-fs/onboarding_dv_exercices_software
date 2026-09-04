import { useState, useEffect, useRef } from 'react'
import { apiFetch, promptForApiKey, readJsonResponse } from '../lib/api'

interface LogEntry {
  timestamp: string;
  type: string;
  message: string;
}

// Pre-#326 the response was assigned to `setLog` without checking it
// was an array; an `{error: "..."}` envelope (which other endpoints
// return — see Scoring.tsx's `if (!d.error)` check) would write a
// non-array, then `log.length > 0 && log.map(...)` threw and unmounted
// the tab (finding F6).
function isLogArray(p: unknown): p is LogEntry[] {
  return Array.isArray(p)
}

export default function SessionLog() {
  const [log, setLog] = useState<LogEntry[]>([])
  const [error, setError] = useState<string | null>(null)
  // `inFlightRef` debounces overlapping polls — pre-#326 the 3 s
  // interval fired regardless of whether the previous fetch had
  // returned, so a slow response could land out of order behind a
  // fresh one and the UI flickered between old and new state
  // (finding F5). Pair with an `AbortController` so a fetch
  // outstanding at unmount doesn't try to setState afterwards.
  const inFlightRef = useRef(false)

  useEffect(() => {
    let cancelled = false
    let controller: AbortController | null = null

    const refresh = async () => {
      if (cancelled || inFlightRef.current) return
      inFlightRef.current = true
      controller = new AbortController()
      try {
        const r = await apiFetch('/api/session/log', { signal: controller.signal }, promptForApiKey)
        if (cancelled) return
        if (!r.ok) {
          setError(`HTTP ${r.status}`)
          return
        }
        const parsed: unknown = await readJsonResponse(r)
        if (cancelled) return
        if (isLogArray(parsed)) {
          setLog(parsed)
          setError(null)
        } else {
          // Non-array response (likely an `{error: "..."}` envelope) —
          // surface it instead of poisoning state with something the
          // table can't render.
          const msg = parsed && typeof parsed === 'object' && 'error' in parsed
            ? String((parsed as Record<string, unknown>).error)
            : 'malformed response'
          setError(msg)
        }
      } catch (e: unknown) {
        // AbortError is expected on cleanup; any other error gets
        // surfaced.
        if (e instanceof DOMException && e.name === 'AbortError') return
        if (cancelled) return
        setError(e instanceof Error ? e.message : String(e))
      } finally {
        inFlightRef.current = false
      }
    }

    refresh()
    const interval = setInterval(refresh, 3000)
    return () => {
      cancelled = true
      clearInterval(interval)
      controller?.abort()
    }
  }, [])

  const exportLog = () => {
    window.open('/api/session/export', '_blank')
  }

  const typeColors: Record<string, string> = {
    event_set: 'text-[#ffb81c]',
    event_start: 'text-green-400',
    res: 'text-red-400',
    reset: 'text-blue-400',
    track_load: 'text-purple-400',
    track_generate: 'text-cyan-400',
  }

  return (
    <div className="space-y-4">
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <div className="flex justify-between items-center mb-4">
          <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold">Session Log</h2>
          <div className="flex gap-2">
            <button onClick={exportLog}
              className="px-3 py-1.5 bg-[#ffb81c] text-black font-medium rounded text-xs hover:bg-[#e6a619]">
              Export JSON
            </button>
          </div>
        </div>

        {error && (
          <p className="mb-3 text-xs text-orange-400">Log fetch error: {error}</p>
        )}

        {log.length > 0 ? (
          <div className="space-y-1 max-h-[500px] overflow-y-auto">
            {[...log].reverse().map((entry, i) => (
              <div key={i} className="flex items-start gap-3 px-3 py-2 rounded hover:bg-[#111] text-sm">
                <span className="text-gray-600 font-mono text-xs whitespace-nowrap">
                  {new Date(entry.timestamp).toLocaleTimeString()}
                </span>
                <span className={`font-medium text-xs uppercase w-24 ${typeColors[entry.type] || 'text-gray-400'}`}>
                  {entry.type}
                </span>
                <span className="text-gray-300">{entry.message}</span>
              </div>
            ))}
          </div>
        ) : (
          <p className="text-gray-500 text-sm">No events logged yet. Actions will appear here.</p>
        )}
      </div>
    </div>
  )
}
