export default function StatusBar({ connected, fps, paused, resActive }: { connected: boolean; fps: number; paused: boolean; resActive: boolean }) {
  return (
    <div className="flex items-center gap-3">
      <span className={`px-3 py-1 rounded-full text-xs font-semibold ${
        connected ? 'bg-green-900/50 text-green-400 border border-green-800' : 'bg-red-900/50 text-red-400 border border-red-800'
      }`}>
        {connected ? 'Connected' : 'Disconnected'}
      </span>
      {connected && (
        <>
          <span className="text-xs text-gray-500">{fps.toFixed(0)} FPS</span>
          {paused && <span className="px-2 py-0.5 rounded text-xs bg-yellow-900/50 text-yellow-400 border border-yellow-800">PAUSED</span>}
          {resActive && <span className="px-2 py-0.5 rounded text-xs bg-red-900/50 text-red-400 border border-red-800 animate-pulse">RES ACTIVE</span>}
        </>
      )}
    </div>
  )
}
