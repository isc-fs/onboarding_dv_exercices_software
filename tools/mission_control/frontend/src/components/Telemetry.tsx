import type { TelemetryData } from '../hooks/useWebSocket'

export default function Telemetry({ telemetry }: { telemetry: TelemetryData }) {
  return (
    <div className="space-y-4">
      {/* Gauges */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
        <Gauge label="Speed" value={telemetry.speed} unit="m/s" max={30} color="#ffb81c" />
        <Gauge label="Motor RPM" value={telemetry.rpm} unit="" max={6000} color="#ef4444" />
        <Gauge label="Throttle" value={telemetry.throttle * 100} unit="%" max={100} color="#22c55e" />
        <Gauge label="Brake" value={telemetry.brake * 100} unit="%" max={100} color="#ef4444" />
      </div>

      {/* Controls Detail */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Vehicle Controls</h2>
        <div className="grid grid-cols-3 gap-6">
          <VerticalBar label="Throttle" value={telemetry.throttle} color="bg-green-500" />
          <div className="text-center">
            <div className="text-xs text-gray-500 uppercase mb-2">Steering</div>
            <div className="relative h-6 bg-[#333] rounded-full overflow-hidden">
              <div className="absolute top-0 left-1/2 h-full w-0.5 bg-gray-600" />
              <div
                className="absolute top-0 h-full bg-blue-500 rounded-full transition-all duration-100"
                style={{
                  left: `${50 + telemetry.steering * 50}%`,
                  width: '8px',
                  transform: 'translateX(-4px)',
                }}
              />
            </div>
            <div className="text-sm text-white mt-1">{(telemetry.steering * 100).toFixed(0)}%</div>
          </div>
          <VerticalBar label="Brake" value={telemetry.brake} color="bg-red-500" />
        </div>
      </div>

      {/* Regen Brake (motor-side, real-car IFS-08 has regen-only drive-wheel braking) */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Regen Brake</h2>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <Gauge
            label="Regen Power"
            value={telemetry.regen_power / 1000}
            unit="kW"
            max={Math.max(1, telemetry.regen_max_power / 1000)}
            color="#10b981"
          />
          <Gauge
            label="Regen Torque"
            value={telemetry.regen_torque}
            unit="Nm (motor)"
            max={Math.max(1, telemetry.regen_max_torque)}
            color="#10b981"
          />
          <Gauge
            label="Avail. Torque"
            value={telemetry.regen_avail_torque}
            unit="Nm cap @ ω"
            max={Math.max(1, telemetry.regen_max_torque)}
            color="#3b82f6"
          />
          <Gauge
            label="Power Cap Binding"
            value={
              telemetry.regen_max_torque > 0
                ? (1 - telemetry.regen_avail_torque / telemetry.regen_max_torque) * 100
                : 0
            }
            unit="%"
            max={100}
            color="#f59e0b"
          />
        </div>
      </div>

      {/* Position */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Vehicle Position (ENU)</h2>
        <div className="grid grid-cols-3 gap-4 text-center">
          <div>
            <div className="text-xs text-gray-500">X (East)</div>
            <div className="text-xl font-mono text-white">{telemetry.x.toFixed(3)}</div>
          </div>
          <div>
            <div className="text-xs text-gray-500">Y (North)</div>
            <div className="text-xl font-mono text-white">{telemetry.y.toFixed(3)}</div>
          </div>
          <div>
            <div className="text-xs text-gray-500">Z (Up)</div>
            <div className="text-xl font-mono text-white">{telemetry.z.toFixed(3)}</div>
          </div>
        </div>
      </div>

      {/* Referee Live */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Referee</h2>
        <div className="grid grid-cols-2 md:grid-cols-5 gap-4 text-center">
          <div>
            <div className="text-xs text-gray-500">Event</div>
            <div className="text-lg font-semibold text-white capitalize">{telemetry.event}</div>
          </div>
          <div>
            <div className="text-xs text-gray-500">Laps</div>
            <div className="text-lg font-semibold text-white">{telemetry.laps}/{telemetry.required_laps}</div>
          </div>
          <div>
            <div className="text-xs text-gray-500">DOO</div>
            <div className={`text-lg font-semibold ${telemetry.doo > 0 ? 'text-orange-400' : 'text-green-400'}`}>{telemetry.doo}</div>
          </div>
          <div>
            <div className="text-xs text-gray-500">OC</div>
            <div className={`text-lg font-semibold ${telemetry.oc > 0 ? 'text-red-400' : 'text-green-400'}`}>{telemetry.oc}</div>
          </div>
          <div>
            <div className="text-xs text-gray-500">Status</div>
            <div className={`text-lg font-semibold ${telemetry.finished ? 'text-green-400' : 'text-[#ffb81c]'}`}>
              {telemetry.finished ? 'FINISHED' : 'RUNNING'}
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

function Gauge({ label, value, unit, max, color }: { label: string; value: number; unit: string; max: number; color: string }) {
  const pct = Math.min(100, (Math.abs(value) / max) * 100)
  return (
    <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-4 text-center">
      <div className="text-xs text-gray-500 uppercase tracking-wider mb-2">{label}</div>
      <div className="text-3xl font-bold" style={{ color }}>{value.toFixed(value > 100 ? 0 : 1)}</div>
      <div className="text-xs text-gray-500 mb-2">{unit}</div>
      <div className="h-2 bg-[#333] rounded-full overflow-hidden">
        <div className="h-full rounded-full transition-all duration-100" style={{ width: `${pct}%`, backgroundColor: color }} />
      </div>
    </div>
  )
}

function VerticalBar({ label, value, color }: { label: string; value: number; color: string }) {
  const pct = Math.abs(value) * 100
  return (
    <div className="text-center">
      <div className="text-xs text-gray-500 uppercase mb-2">{label}</div>
      <div className="h-32 w-8 mx-auto bg-[#333] rounded-full overflow-hidden relative">
        <div
          className={`absolute bottom-0 w-full ${color} rounded-full transition-all duration-100`}
          style={{ height: `${pct}%` }}
        />
      </div>
      <div className="text-sm text-white mt-1">{pct.toFixed(0)}%</div>
    </div>
  )
}
