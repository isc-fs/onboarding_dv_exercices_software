import { useState, useEffect } from 'react';
import { apiWsUrl } from '../lib/api';

export interface TelemetryData {
  speed: number;
  rpm: number;
  gear: number;
  x: number;
  y: number;
  z: number;
  throttle: number;
  steering: number;
  brake: number;
  // Regen telemetry (motor-side, pre-gearbox)
  regen_torque: number;         // Nm currently absorbed
  regen_power: number;          // W currently absorbed
  regen_avail_torque: number;   // Nm cap at the current motor ω
  regen_max_torque: number;     // Hardware motor-torque ceiling (from settings)
  regen_max_power: number;      // Hardware cell-input power ceiling (from settings)
  doo: number;
  oc: number;
  laps: number;
  required_laps: number;
  finished: boolean;
  event: string;
  fps: number;
  paused: boolean;
  res_active: boolean;
  pipeline_enabled: boolean;
  // #465 — optional bag-record status pushed on every WS tick when
  // recording is wired through Mission Control's session UX.
  //
  //   bag_state:
  //     "none"        — no recording active (default)
  //     "starting"    — `ros2 bag record` spawned, awaiting first scan
  //     "recording"   — recorder PID confirmed alive
  //     "stopped"     — clean SIGINT shutdown, mcap closed
  //     "failed"      — start refused (disk-full / mcap plugin missing)
  //                     or stop errored (see logs)
  //
  // bag_name is the directory name under the host `bags/` landing
  // zone (bind-mounted into mc_backend as /bags); null while recording
  // / unset on legacy backends.
  //
  // bag_host_path is the absolute path the bag landed at on the host
  // after the #498 auto-pull (e.g. `/host_bags/<bag_name>`). Empty
  // until auto-pull completes; absent on legacy backends.
  bag_state?: 'none' | 'starting' | 'recording' | 'stopped' | 'failed';
  bag_name?: string | null;
  bag_host_path?: string | null;
  error?: string;
}

const defaultTelemetry: TelemetryData = {
  speed: 0, rpm: 0, gear: 0,
  x: 0, y: 0, z: 0,
  throttle: 0, steering: 0, brake: 0,
  regen_torque: 0, regen_power: 0, regen_avail_torque: 0,
  regen_max_torque: 0, regen_max_power: 0,
  doo: 0, oc: 0, laps: 0, required_laps: 0,
  finished: false, event: 'unknown',
  fps: 0, paused: false, res_active: false, pipeline_enabled: false,
  bag_state: 'none', bag_name: null, bag_host_path: null,
};

// Reconnect tuning. Capped at 30 s — long enough to spare a slow-restart
// backend, short enough that an operator-triggered reload reconnects
// before they get impatient. ±20% jitter avoids reconnect storms when
// multiple tabs lose the connection at the same instant. Pre-#326 this
// hook reconnected on a flat 2 s with no cap and no jitter (finding F3).
const RECONNECT_BASE_MS = 1_000;
const RECONNECT_CAP_MS = 30_000;
const RECONNECT_JITTER = 0.2;

function nextBackoff(attempt: number): number {
  const exp = Math.min(RECONNECT_CAP_MS, RECONNECT_BASE_MS * 2 ** attempt);
  const jitter = exp * RECONNECT_JITTER * (Math.random() * 2 - 1);
  return Math.max(0, exp + jitter);
}

// Shape gate for incoming WS frames. Pre-#326 every parsed message
// replaced `data` wholesale (finding F4) — a partial frame, an event
// envelope, or a ping/pong from a different protocol would blank
// fields like `speed`/`fps`/`x` and crash the downstream `.toFixed`
// calls. Now we (a) require it to be a plain object, (b) require at
// least one of the numeric scalars consumers actually depend on, and
// (c) merge into the previous state instead of overwriting.
function isTelemetryFrame(p: unknown): p is Partial<TelemetryData> {
  if (!p || typeof p !== 'object' || Array.isArray(p)) return false;
  const o = p as Record<string, unknown>;
  if (typeof o.error === 'string') return false;
  return typeof o.speed === 'number' || typeof o.fps === 'number';
}

export function useWebSocket(url: string) {
  const [data, setData] = useState<TelemetryData>(defaultTelemetry);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // All connection state lives in this effect's closure. A new effect
    // run (url change, StrictMode dev double-mount) gets a fresh set of
    // bindings; cleanup of the previous run sets `cancelled = true` on
    // its OWN bindings, breaking the onclose-triggers-reconnect loop
    // that pre-#326 leaked timers across remounts (finding F2).
    let cancelled = false;
    let ws: WebSocket | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let attempt = 0;

    function scheduleRetry() {
      if (cancelled || reconnectTimer) return;
      const delay = nextBackoff(attempt++);
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        if (!cancelled) connect();
      }, delay);
    }

    function connect() {
      if (cancelled) return;
      try {
        const sock = new WebSocket(apiWsUrl(url));
        ws = sock;

        sock.onopen = () => {
          if (cancelled) {
            sock.close();
            return;
          }
          attempt = 0;
          setConnected(true);
        };
        sock.onclose = () => {
          setConnected(false);
          scheduleRetry();
        };
        sock.onerror = () => sock.close();
        sock.onmessage = (e) => {
          try {
            const parsed: unknown = JSON.parse(e.data);
            if (isTelemetryFrame(parsed)) {
              setData(prev => ({ ...prev, ...parsed }));
            }
          } catch {
            // Malformed JSON — silently ignored. Logging at WS rate is
            // a footgun (one bad server frame would spam the console
            // 100 Hz); a separate finding would add a rate-limited
            // error beacon if this ever proves to matter in practice.
          }
        };
      } catch {
        scheduleRetry();
      }
    }

    connect();

    return () => {
      // Unmount or url change. Mark cancelled FIRST so any in-flight
      // callbacks bail out, then null the WS handlers so close() can't
      // re-trigger the reconnect loop, then clear the pending timer.
      cancelled = true;
      if (ws) {
        ws.onopen = null;
        ws.onclose = null;
        ws.onerror = null;
        ws.onmessage = null;
        ws.close();
        ws = null;
      }
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
      setConnected(false);
    };
  }, [url]);

  return { data, connected };
}
