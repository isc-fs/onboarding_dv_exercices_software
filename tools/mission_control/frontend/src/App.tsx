import { useState } from 'react'
import { useWebSocket, type TelemetryData } from './hooks/useWebSocket'
import { apiFetch, promptForApiKey } from './lib/api'
import StatusBar from './components/StatusBar'
import EventSetup from './components/EventSetup'
import TrackManager from './components/TrackManager'
import Telemetry from './components/Telemetry'
import Scoring from './components/Scoring'
import SessionLog from './components/SessionLog'
import './index.css'

const TABS = ['Dashboard', 'Event', 'Tracks', 'Telemetry', 'Scoring', 'Log'] as const
type Tab = typeof TABS[number]

function App() {
  const [tab, setTab] = useState<Tab>('Dashboard')
  const { data: telemetry, connected } = useWebSocket(
    `ws://${window.location.hostname}:${window.location.port}/ws/telemetry`
  )

  return (
    <div className="min-h-screen bg-[#0a0a0a] text-gray-200">
      {/* Header */}
      <header className="bg-black border-b-[3px] border-[#ffb81c] px-6 py-4 flex items-center justify-between">
        <div className="flex items-center gap-4">
          <h1 className="text-[#ffb81c] text-xl font-bold tracking-wide">
            IFSSIM <span className="text-gray-300 font-normal">Mission Control</span>
          </h1>
          <StatusBar connected={connected} fps={telemetry.fps} paused={telemetry.paused} resActive={telemetry.res_active} />
        </div>

        {/* RES Button — most safety-critical control in the app, so
            the toggled "armed" state is conveyed by THREE redundant
            cues so a colourblind operator can still tell what's
            happening: text label flips ("RES" ↔ "RES ACTIVE"), the
            border/glow change colour, and the button gets
            `aria-pressed="true"` for screen readers. The pulse
            animation is the fourth, motion-based cue. Pre-#326 this
            was colour-only (finding F12). */}
        <button
          aria-pressed={telemetry.res_active}
          aria-label={telemetry.res_active ? 'Release emergency stop' : 'Activate emergency stop'}
          onClick={async () => {
            if (telemetry.res_active) {
              await apiFetch('/api/res/release', { method: 'POST' }, promptForApiKey)
            } else {
              await apiFetch('/api/res/activate', { method: 'POST' }, promptForApiKey)
            }
          }}
          className={`font-bold px-6 py-2 rounded-lg text-sm uppercase tracking-wider border-2
                     transition-all active:scale-95 ${
                       telemetry.res_active
                         ? 'bg-red-600 text-white border-red-400 shadow-[0_0_25px_rgba(239,68,68,0.6)] animate-pulse'
                         : 'bg-red-900 hover:bg-red-700 text-white border-red-700 shadow-[0_0_10px_rgba(239,68,68,0.2)] hover:shadow-[0_0_20px_rgba(239,68,68,0.4)]'
                     }`}
        >
          {telemetry.res_active ? '■ RES ACTIVE' : 'RES'}
        </button>
      </header>

      {/* Tab Navigation */}
      <nav className="bg-[#111] border-b border-[#333] px-6 flex gap-1">
        {TABS.map(t => (
          <button
            key={t}
            onClick={() => setTab(t)}
            className={`px-4 py-2.5 text-sm font-medium transition-all border-b-2 ${
              tab === t
                ? 'text-[#ffb81c] border-[#ffb81c]'
                : 'text-gray-500 border-transparent hover:text-gray-300 hover:border-gray-600'
            }`}
          >
            {t}
          </button>
        ))}
      </nav>

      {/* Content */}
      <main className="p-6 max-w-[1400px] mx-auto">
        {tab === 'Dashboard' && <DashboardView telemetry={telemetry} />}
        {tab === 'Event' && <EventSetup telemetry={telemetry} />}
        {tab === 'Tracks' && <TrackManager />}
        {tab === 'Telemetry' && <Telemetry telemetry={telemetry} />}
        {tab === 'Scoring' && <Scoring />}
        {tab === 'Log' && <SessionLog />}
      </main>

      {/* Footer */}
      <footer className="text-center py-4 text-gray-600 text-xs border-t border-[#222]">
        <a href="https://iscracingteam.com/formulastudent/" target="_blank" rel="noopener noreferrer" className="text-[#ffb81c] hover:underline">
          ISC Racing Team
        </a> | IFSSIM Formula Student Driverless Simulator
      </footer>
    </div>
  )
}

function DashboardView({ telemetry }: { telemetry: TelemetryData }) {
  return (
    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
      {/* Speed */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-2">Speed</div>
        <div className="text-5xl font-bold text-[#ffb81c]">{telemetry.speed.toFixed(1)}</div>
        <div className="text-gray-500 text-sm">m/s</div>
      </div>

      {/* Motor */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-2">Motor RPM</div>
        <div className="text-5xl font-bold text-white">{telemetry.rpm.toFixed(0)}</div>
        <div className="text-gray-500 text-sm">Electric Drive</div>
      </div>

      {/* Event */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-2">Event</div>
        <div className="text-2xl font-bold text-white capitalize">{telemetry.event}</div>
        <div className="text-gray-500 text-sm">
          Lap {telemetry.laps}/{telemetry.required_laps}
          {telemetry.finished && <span className="text-green-400 ml-2">FINISHED</span>}
        </div>
      </div>

      {/* DOO */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-2">DOO (Cone Hits)</div>
        <div className={`text-4xl font-bold ${telemetry.doo > 0 ? 'text-orange-400' : 'text-green-400'}`}>
          {telemetry.doo}
        </div>
        <div className="text-gray-500 text-sm">+{(telemetry.doo * 2).toFixed(0)}s penalty</div>
      </div>

      {/* OC */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-2">OC (Off Track)</div>
        <div className={`text-4xl font-bold ${telemetry.oc > 0 ? 'text-red-400' : 'text-green-400'}`}>
          {telemetry.oc}
        </div>
        <div className="text-gray-500 text-sm">+{(telemetry.oc * 10).toFixed(0)}s penalty</div>
      </div>

      {/* Position */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 text-center">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-2">Position (ENU)</div>
        <div className="text-lg font-mono text-white">
          {telemetry.x.toFixed(2)}, {telemetry.y.toFixed(2)}
        </div>
        <div className="text-gray-500 text-sm">z: {telemetry.z.toFixed(2)}m</div>
      </div>

      {/* Controls */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 col-span-1 md:col-span-3">
        <div className="text-gray-500 text-xs uppercase tracking-wider mb-3">Controls</div>
        <div className="flex gap-6 justify-center">
          <ControlBar label="Throttle" value={telemetry.throttle} color="bg-green-500" />
          <ControlBar label="Brake" value={telemetry.brake} color="bg-red-500" />
          <ControlBar label="Steering" value={(telemetry.steering + 1) / 2} color="bg-blue-500" />
        </div>
      </div>
    </div>
  )
}

function ControlBar({ label, value, color }: { label: string; value: number; color: string }) {
  const pct = Math.abs(value) * 100
  return (
    <div className="flex-1 max-w-[200px]">
      <div className="flex justify-between text-xs text-gray-500 mb-1">
        <span>{label}</span>
        <span>{(value * 100).toFixed(0)}%</span>
      </div>
      <div className="h-3 bg-[#333] rounded-full overflow-hidden">
        <div className={`h-full ${color} rounded-full transition-all duration-100`} style={{ width: `${pct}%` }} />
      </div>
    </div>
  )
}

export default App
