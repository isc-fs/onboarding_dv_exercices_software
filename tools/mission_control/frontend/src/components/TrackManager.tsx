import { useState, useEffect } from 'react'
import { apiFetch, promptForApiKey, readJsonResponse } from '../lib/api'
import { useConfirm } from './ConfirmDialog'

interface Track {
  name: string; path: string; cones: number;
  blue: number; yellow: number; orange: number;
  builtin: boolean; event_type: string | null;
}

// Response shapes — kept narrow per endpoint so TS catches typos at
// call sites. All fields optional because the backend skips error
// envelopes on success and vice versa.
interface PreviewResponse { image?: string }
interface LoadResponse {
  result?: { error?: string }
  event_type?: string
}
interface GenerateResponse {
  name?: string
  cones?: number
  error?: string
}

export default function TrackManager() {
  const [tracks, setTracks] = useState<Track[]>([])
  const [selected, setSelected] = useState<string | null>(null)
  const [preview, setPreview] = useState<string | null>(null)
  const [msg, setMsg] = useState('')
  const [genParams, setGenParams] = useState({ n_points: 50, n_regions: 30, max_bound: 150, name: '' })
  const [generating, setGenerating] = useState(false)
  // `loadingName` holds the name of the track whose Load button is
  // currently in flight. Pre-#326 the Load buttons fired immediately
  // with no `disabled` state, so a panicked double-click queued two
  // sim-state-mutating loads and the UI showed whichever response
  // landed second (finding F7). All Load buttons disable while ANY
  // load is running — there is no useful concurrent-load workflow.
  const [loadingName, setLoadingName] = useState<string | null>(null)
  // `deletingName` is the same idea for Delete: prevents the user
  // from queueing a delete on top of an in-flight delete. Visible
  // separately from `loadingName` so the affected row's button
  // shows the correct label.
  const [deletingName, setDeletingName] = useState<string | null>(null)
  const confirm = useConfirm()

  const refresh = async () => {
    const r = await apiFetch('/api/track/list', {}, promptForApiKey)
    setTracks(await readJsonResponse<Track[]>(r))
  }

  useEffect(() => { refresh() }, [])

  const selectTrack = async (name: string) => {
    setSelected(name)
    setPreview(null)
    const r = await apiFetch(`/api/track/${encodeURIComponent(name)}/preview`, {}, promptForApiKey)
    const d = await readJsonResponse<PreviewResponse>(r)
    if (d.image) setPreview(d.image)
  }

  const loadTrack = async (track: Track) => {
    if (loadingName) return
    setLoadingName(track.name)
    setMsg(`Loading ${track.name}...`)
    try {
      const r = await apiFetch(`/api/track/${encodeURIComponent(track.name)}/load`, { method: 'POST' }, promptForApiKey)
      const d = await readJsonResponse<LoadResponse>(r)
      if (d.result?.error) {
        setMsg(`Error: ${d.result.error}`)
      } else {
        setMsg(d.event_type
          ? `Loaded ${track.name} — event set to ${d.event_type}`
          : `Loaded ${track.name}`)
      }
    } catch (e) {
      setMsg(`Error: ${e instanceof Error ? e.message : String(e)}`)
    } finally {
      setLoadingName(null)
    }
  }

  const deleteTrack = async (name: string) => {
    if (deletingName) return
    const ok = await confirm({
      title: 'Delete track?',
      message: `Permanently delete ${name}? This cannot be undone.`,
      confirmLabel: 'Delete',
      destructive: true,
    })
    if (!ok) return
    setDeletingName(name)
    try {
      await apiFetch(`/api/track/${encodeURIComponent(name)}`, { method: 'DELETE' }, promptForApiKey)
      setMsg(`Deleted ${name}`)
      if (selected === name) { setSelected(null); setPreview(null) }
      refresh()
    } catch (e) {
      setMsg(`Error: ${e instanceof Error ? e.message : String(e)}`)
    } finally {
      setDeletingName(null)
    }
  }

  const generate = async () => {
    setGenerating(true)
    const r = await apiFetch('/api/track/generate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(genParams),
    }, promptForApiKey)
    const d = await readJsonResponse<GenerateResponse>(r)
    setGenerating(false)
    if (d.name) {
      setMsg(`Generated: ${d.name} (${d.cones} cones)`)
      refresh()
      selectTrack(d.name)
    } else {
      setMsg(`Error: ${d.error}`)
    }
  }

  const builtinTracks = tracks.filter(t => t.builtin)
  const generatedTracks = tracks.filter(t => !t.builtin)

  return (
    <div className="space-y-4">
      {/* Standard Tracks */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Standard Tracks</h2>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
          {builtinTracks.map(t => (
            <div
              key={t.name}
              role="button"
              tabIndex={0}
              aria-pressed={selected === t.name}
              aria-label={`Select track ${t.name}`}
              onClick={() => selectTrack(t.name)}
              onKeyDown={e => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault()
                  selectTrack(t.name)
                }
              }}
              className={`flex justify-between items-center px-4 py-3 rounded-lg cursor-pointer border transition-all
                         focus:outline-none focus:ring-2 focus:ring-[#ffb81c] focus:ring-offset-2 focus:ring-offset-[#1a1a1a] ${
                selected === t.name ? 'border-[#ffb81c] bg-[#1a1a0a]' : 'border-[#333] bg-[#111] hover:border-[#555]'
              }`}
            >
              <div>
                <div className="flex items-center gap-2">
                  <span className="font-medium text-sm capitalize">{t.name.replace('.csv', '')}</span>
                  <span className="text-[10px] px-2 py-0.5 rounded-full bg-[#ffb81c]/10 text-[#ffb81c] border border-[#ffb81c]/30 uppercase tracking-wide">
                    {t.event_type}
                  </span>
                </div>
                <div className="text-xs text-gray-500 mt-0.5">
                  <span className="text-blue-400">{t.blue}</span> blue{' '}
                  <span className="text-[#ffb81c]">{t.yellow}</span> yellow{' '}
                  <span className="text-orange-500">{t.orange}</span> orange
                </div>
              </div>
              <button
                onClick={e => { e.stopPropagation(); loadTrack(t) }}
                disabled={loadingName !== null}
                className="px-3 py-1 bg-green-900/50 text-green-400 border border-green-800 rounded text-xs font-medium hover:bg-green-800/50 whitespace-nowrap
                           disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:bg-green-900/50"
              >
                {loadingName === t.name ? 'Loading…' : 'Load'}
              </button>
            </div>
          ))}
        </div>
        {msg && <p className="mt-3 text-xs text-gray-400">{msg}</p>}
      </div>

      {/* Generator */}
      <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
        <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Generate Track</h2>
        <div className="flex flex-wrap gap-3 items-end">
          <Field label="Points" value={genParams.n_points} onChange={(v: string) => setGenParams({...genParams, n_points: +v})} type="number" w="w-20" />
          <Field label="Regions" value={genParams.n_regions} onChange={(v: string) => setGenParams({...genParams, n_regions: +v})} type="number" w="w-20" />
          <Field label="Max Size (m)" value={genParams.max_bound} onChange={(v: string) => setGenParams({...genParams, max_bound: +v})} type="number" w="w-24" />
          <Field label="Name" value={genParams.name} onChange={(v: string) => setGenParams({...genParams, name: v})} placeholder="auto" w="w-36" />
          <button onClick={generate} disabled={generating}
            className="px-4 py-2 bg-[#ffb81c] text-black font-bold rounded-lg text-sm hover:bg-[#e6a619] disabled:opacity-50">
            {generating ? 'Generating...' : 'Generate'}
          </button>
          <button onClick={refresh} className="px-4 py-2 bg-[#333] text-gray-300 rounded-lg text-sm hover:bg-[#444]">
            Refresh
          </button>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Generated Track List */}
        <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6">
          <h2 className="text-[#ffb81c] text-sm uppercase tracking-wider font-semibold mb-4">Generated Tracks</h2>
          <div className="space-y-2 max-h-[400px] overflow-y-auto">
            {generatedTracks.map(t => (
              <div
                key={t.name}
                role="button"
                tabIndex={0}
                aria-pressed={selected === t.name}
                aria-label={`Select track ${t.name}`}
                className={`flex justify-between items-center px-4 py-3 rounded-lg cursor-pointer border transition-all
                           focus:outline-none focus:ring-2 focus:ring-[#ffb81c] focus:ring-offset-2 focus:ring-offset-[#1a1a1a] ${
                  selected === t.name ? 'border-[#ffb81c] bg-[#1a1a0a]' : 'border-[#333] bg-[#111] hover:border-[#555]'
                }`}
                onClick={() => selectTrack(t.name)}
                onKeyDown={e => {
                  if (e.key === 'Enter' || e.key === ' ') {
                    e.preventDefault()
                    selectTrack(t.name)
                  }
                }}
              >
                <div>
                  <div className="font-medium text-sm">{t.name}</div>
                  <div className="text-xs text-gray-500">
                    <span className="text-blue-400">{t.blue}</span> blue{' '}
                    <span className="text-[#ffb81c]">{t.yellow}</span> yellow{' '}
                    <span className="text-orange-500">{t.orange}</span> orange{' '}
                    <span className="text-gray-500">({t.cones} total)</span>
                  </div>
                </div>
                <div className="flex gap-2">
                  <button
                    onClick={e => { e.stopPropagation(); loadTrack(t) }}
                    disabled={loadingName !== null || deletingName === t.name}
                    className="px-3 py-1 bg-green-900/50 text-green-400 border border-green-800 rounded text-xs font-medium hover:bg-green-800/50
                               disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:bg-green-900/50"
                  >
                    {loadingName === t.name ? 'Loading…' : 'Load'}
                  </button>
                  <button
                    onClick={e => { e.stopPropagation(); deleteTrack(t.name) }}
                    disabled={deletingName !== null || loadingName === t.name}
                    className="px-2 py-1 text-red-400 border border-red-900 rounded text-xs hover:bg-red-900/30
                               disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:bg-transparent"
                  >
                    {deletingName === t.name ? '…' : 'Del'}
                  </button>
                </div>
              </div>
            ))}
            {generatedTracks.length === 0 && <p className="text-gray-500 text-sm">No generated tracks yet</p>}
          </div>
        </div>

        {/* Preview */}
        <div className="bg-[#1a1a1a] border border-[#333] rounded-xl p-6 flex items-center justify-center min-h-[300px]">
          {preview ? (
            <img src={`data:image/png;base64,${preview}`} alt="Track preview" className="max-w-full rounded-lg" />
          ) : (
            <p className="text-gray-600 text-sm">Select a track to preview</p>
          )}
        </div>
      </div>
    </div>
  )
}

interface FieldProps {
  label: string
  value: string | number
  onChange: (v: string) => void
  type?: string
  placeholder?: string
  w?: string
}

function Field({ label, value, onChange, type, placeholder, w }: FieldProps) {
  return (
    <label className="flex flex-col gap-1 text-xs text-gray-500 uppercase tracking-wide">
      {label}
      <input
        type={type || 'text'} value={value} onChange={e => onChange(e.target.value)}
        placeholder={placeholder}
        className={`${w || 'w-24'} px-3 py-1.5 bg-[#111] border border-[#444] rounded text-white text-sm`}
      />
    </label>
  )
}
