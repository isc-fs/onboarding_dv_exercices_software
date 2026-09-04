import { useState, useEffect, useRef } from 'react'
import { apiFetch, promptForApiKey, readJsonResponse } from '../lib/api'
import type { TelemetryData } from '../hooks/useWebSocket'
import { useConfirm } from './ConfirmDialog'

const EVENTS = ['trackdrive', 'autocross', 'acceleration', 'skidpad'] as const
type EventName = typeof EVENTS[number]

// #465 — localStorage key for the "Record bag" checkbox. Persisted
// across page reloads so a tester who always wants recordings doesn't
// have to re-tick every session.
const LS_RECORD_BAG = 'ifssim.mc.record_bag'

function loadRecordBagPref(): boolean {
  try {
    return localStorage.getItem(LS_RECORD_BAG) === '1'
  } catch {
    return false
  }
}

function saveRecordBagPref(v: boolean) {
  try {
    localStorage.setItem(LS_RECORD_BAG, v ? '1' : '0')
  } catch {
    /* private-mode / quota — best-effort persistence */
  }
}

export default function EventSetup({ telemetry }: { telemetry: TelemetryData }) {
  const [event, setEvent] = useState<EventName>('trackdrive')
  const [laps, setLaps] = useState(10)
  const [msg, setMsg] = useState('')
  // `busy` gates Start/Stop while a request is in flight so a panicked
  // double-click can't re-fire the 4.5 s event_start sequence (#327 B7) or
  // queue a duplicate pipeline-stop. Cleared in the finally block.
  const [busy, setBusy] = useState(false)
  // #465 — optional bag-record. Default to the localStorage-persisted
  // preference. Disabled while busy (same gating as other inputs).
  const [recordBag, setRecordBag] = useState<boolean>(loadRecordBagPref)
  const confirm = useConfirm()
  // `initialSyncDoneRef` ensures we mirror the sim's reported event ONCE
  // when the first non-unknown frame lands, then leave the local
  // selection alone. Pre-#326 the effect ran on every WS frame, so a
  // user picking "skidpad" locally got silently overridden the moment
  // telemetry reported "trackdrive" again (finding F9). With this
  // ref the local selection is a "draft" the operator owns until they
  // actually click Start.
  const initialSyncDoneRef = useRef(false)

  useEffect(() => {
    if (initialSyncDoneRef.current) return
    if (telemetry.event && telemetry.event !== 'unknown' && (EVENTS as readonly string[]).includes(telemetry.event)) {
      setEvent(telemetry.event as EventName)
      initialSyncDoneRef.current = true
    }
  }, [telemetry.event])

  // Union of the response shapes the backend returns for this component's
  // endpoints. Defining one type for all call sites is enough — the keys
  // are all optional and TS lets callers read whichever ones their
  // endpoint actually produces. Callers that want stricter typing can
  // pass an explicit type argument (`await api<MyShape>(...)`).
  type ApiResponse = {
    ok?: boolean
    error?: string
    // /api/event/start echoes:
    event?: string
    laps?: number
    bag?: { error?: string; name?: string }
  }
  const api = async <T = ApiResponse>(
    url: string,
    body?: Record<string, unknown>,
  ): Promise<T> => {
    const res = await apiFetch(url, {
      method: 'POST',
      headers: body ? { 'Content-Type': 'application/json' } : {},
      body: body ? JSON.stringify(body) : undefined,
    }, promptForApiKey)
    return readJsonResponse<T>(res)
  }

  return (
    <div className="space-y-6">
      {/* Event Selector */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Event Configuration</h2>
        <div className="flex flex-wrap gap-3 mb-4">
          {EVENTS.map(e => (
            <button
              key={e}
              onClick={() => setEvent(e)}
              className={`px-4 py-2 rounded-lg text-sm font-medium capitalize transition-all ${
                event === e
                  ? 'bg-[#ffb81c] text-black'
                  : 'bg-[#222] text-gray-400 hover:bg-[#333] border border-[#444]'
              }`}
            >
              {e}
            </button>
          ))}
        </div>

        {event === 'trackdrive' && (
          <div className="flex items-center gap-3 mb-4">
            <label className="text-sm text-gray-400">Laps:</label>
            <input
              type="number" value={laps} onChange={e => setLaps(+e.target.value)}
              className="w-20 px-3 py-1.5 bg-[#111] border border-[#444] rounded text-white text-sm"
              min={1} max={100}
            />
          </div>
        )}

        {/* #465 — bag-record toggle. Sits ABOVE the Start/Stop row
            so the operator sees it as part of "what this Start will
            do" rather than a stray control. Persisted preference. */}
        <label className="flex items-center gap-2 mb-3 text-sm text-gray-300 select-none cursor-pointer">
          <input
            type="checkbox"
            checked={recordBag}
            disabled={busy}
            onChange={e => { setRecordBag(e.target.checked); saveRecordBagPref(e.target.checked) }}
            className="w-4 h-4 accent-[#ffb81c] disabled:opacity-50"
          />
          <span>Record bag (mcap)</span>
          <span className="text-gray-500 text-xs">
            {/* Wording updated post-#498 auto-pull: bags now land on host
                automatically at Stop Session, no manual pull-bag.sh
                step needed in the happy path. The post-stop badge
                surfaces the host path (or falls back to the manual
                hint when auto-pull is disabled or fails). */}
            — full topic dump; auto-pulled onto host{' '}
            <code className="text-gray-400">bags/&lt;name&gt;/</code> when the session stops.
          </span>
        </label>

        <div className="flex gap-3 flex-wrap items-center">
          <button
            disabled={busy}
            onClick={async () => {
              setBusy(true)
              setMsg('Starting…')
              try {
                const r = await api('/api/event/start', {
                  event_type: event,
                  num_laps: laps,
                  record_bag: recordBag,
                })
                if (r.ok) {
                  let m = `Session started: ${r.event} (${r.laps} laps)`
                  // The backend echoes bag info on the response if
                  // record_bag was true. Surface failures clearly —
                  // the session is up but recording isn't, so the
                  // operator knows not to wait for an mcap.
                  if (r.bag?.error) {
                    m += ` — recording failed: ${r.bag.error}`
                  } else if (r.bag?.name) {
                    m += ` — recording: ${r.bag.name}`
                  }
                  setMsg(m)
                } else {
                  setMsg(`Error: ${r.error ?? 'unknown'}`)
                }
              } catch (e) {
                setMsg(`Error: ${e instanceof Error ? e.message : String(e)}`)
              } finally {
                setBusy(false)
              }
            }}
            className="px-6 py-2.5 bg-green-700 text-white font-bold rounded-lg text-sm hover:bg-green-600 transition-all
                       shadow-[0_0_10px_rgba(34,197,94,0.3)] disabled:opacity-50 disabled:cursor-not-allowed
                       disabled:hover:bg-green-700"
          >
            {busy ? 'Working…' : 'Start Session'}
          </button>
          {/* Stop Session disables the autonomy pipeline (removes the
              control file the launcher polls). It deliberately does NOT
              activate RES — that's the dedicated emergency-stop button
              in the header. Earlier this button fired
              /api/res/activate with a misleading "Session stopped"
              label; see #326 finding F1. */}
          <button
            disabled={busy}
            onClick={async () => {
              const ok = await confirm({
                title: 'Stop autonomy pipeline?',
                message:
                  'This disables the autonomous driver. The sim and vehicle keep running.\n\n' +
                  'For an emergency stop, use the RES button in the header instead.',
                confirmLabel: 'Stop pipeline',
                destructive: true,
              })
              if (!ok) return
              setBusy(true)
              setMsg('Stopping pipeline…')
              try {
                const r = await api('/api/pipeline/stop')
                setMsg(r.ok ? 'Autonomy pipeline stopped' : `Error: ${r.error ?? 'unknown'}`)
              } catch (e) {
                setMsg(`Error: ${e instanceof Error ? e.message : String(e)}`)
              } finally {
                setBusy(false)
              }
            }}
            className="px-6 py-2.5 bg-red-700 text-white font-bold rounded-lg text-sm hover:bg-red-600 transition-all
                       shadow-[0_0_10px_rgba(239,68,68,0.3)] disabled:opacity-50 disabled:cursor-not-allowed
                       disabled:hover:bg-red-700"
          >
            {busy ? 'Working…' : 'Stop Session'}
          </button>
          {telemetry.pipeline_enabled && (
            <span className="text-green-400 text-xs font-medium animate-pulse">● PIPELINE RUNNING</span>
          )}
          {/* #465 — bag-record live badge. Visible whenever the
              recorder is alive (starting/recording) and stays visible
              one terminal-state tick after stop so the operator sees
              the "stopped" or "failed" outcome without missing it. */}
          {telemetry.bag_state === 'recording' && (
            <span className="text-red-400 text-xs font-medium animate-pulse">
              ● RECORDING{telemetry.bag_name ? ` — ${telemetry.bag_name}` : ''}
            </span>
          )}
          {telemetry.bag_state === 'starting' && (
            <span className="text-amber-400 text-xs font-medium">
              ◌ starting recorder…
            </span>
          )}
          {telemetry.bag_state === 'stopped' && telemetry.bag_name && telemetry.bag_host_path && (
            // #498 — auto-pull succeeded; bag is on host at the
            // bind-mounted path. The leading "✓" + green tint signals
            // operator can find the bag at `./bags/<name>/` without
            // running tools/pull-bag.sh manually.
            <span className="text-green-400 text-xs" title={`Bag on host:\nbags/${telemetry.bag_name}/`}>
              ✓ bag saved to host: <code className="font-mono">bags/{telemetry.bag_name}/</code>
            </span>
          )}
          {telemetry.bag_state === 'stopped' && telemetry.bag_name && !telemetry.bag_host_path && (
            // Auto-pull disabled or failed — bag is only in the
            // container's named volume. Operator can recover with the
            // manual pull script.
            <span className="text-gray-400 text-xs" title={`Pull onto host:\ntools/pull-bag.sh ${telemetry.bag_name}`}>
              ◍ bag saved (in container): {telemetry.bag_name}
              <span className="ml-1 text-gray-500">
                — pull with <code className="font-mono">tools/pull-bag.sh</code>
              </span>
            </span>
          )}
          {telemetry.bag_state === 'failed' && (
            <span className="text-red-500 text-xs font-medium">
              ✗ recording failed (see logs)
            </span>
          )}
        </div>

        {msg && <p className="mt-3 text-xs text-gray-400">{msg}</p>}
      </div>

      {/* Sim Controls */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Simulation Control</h2>
        <div className="flex gap-3">
          <button onClick={() => api('/api/sim/pause')}
            className="px-4 py-2 bg-yellow-700 text-white rounded-lg text-sm font-medium hover:bg-yellow-600">
            Pause
          </button>
          <button onClick={() => api('/api/sim/resume')}
            className="px-4 py-2 bg-green-700 text-white rounded-lg text-sm font-medium hover:bg-green-600">
            Resume
          </button>
          <button
            onClick={async () => {
              const ok = await confirm({
                title: 'Reset simulation?',
                message:
                  'Teleports the vehicle back to the start, clears DOO/OC counters, and resets the lap counter.\n\n' +
                  'The autonomy pipeline keeps its current state — disable it separately if you want a clean restart.',
                confirmLabel: 'Reset',
                destructive: true,
              })
              if (ok) api('/api/sim/reset')
            }}
            className="px-4 py-2 bg-red-800 text-white rounded-lg text-sm font-medium hover:bg-red-700">
            Reset
          </button>
        </div>
      </div>

      {/* Live State */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Live Event State</h2>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
          <Stat label="Event" value={telemetry.event} />
          <Stat label="Laps" value={`${telemetry.laps}/${telemetry.required_laps}`} />
          <Stat label="DOO" value={telemetry.doo} warn={telemetry.doo > 0} />
          <Stat label="OC" value={telemetry.oc} warn={telemetry.oc > 0} />
          <Stat label="Speed" value={`${telemetry.speed.toFixed(1)} m/s`} />
          <Stat label="Finished" value={telemetry.finished ? 'YES' : 'No'} good={telemetry.finished} />
          <Stat label="Paused" value={telemetry.paused ? 'YES' : 'No'} />
          <Stat label="FPS" value={telemetry.fps.toFixed(0)} />
        </div>
      </div>
    </div>
  )
}

function Stat({ label, value, warn, good }: { label: string; value: string | number | boolean; warn?: boolean; good?: boolean }) {
  return (
    <div className="bg-[#111] rounded-lg px-4 py-3">
      <div className="text-gray-500 text-xs uppercase mb-1">{label}</div>
      <div className={`text-lg font-semibold ${warn ? 'text-orange-400' : good ? 'text-green-400' : 'text-white'}`}>
        {String(value)}
      </div>
    </div>
  )
}
