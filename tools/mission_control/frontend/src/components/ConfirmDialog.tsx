/**
 * Non-blocking, themed replacement for `window.confirm`. Pre-#326 the
 * app used native `confirm` / `alert` / `prompt` calls (finding F10),
 * which freeze the entire renderer thread while open — WS messages
 * queue, render is blocked, and on a 100 Hz telemetry feed that
 * shows up as visible jank when the operator dismisses. Replaced
 * here for everything except the api-key bootstrap prompt (auth
 * flow that benefits from native focus behaviour).
 *
 * Usage:
 *
 *   <ConfirmProvider>
 *     <App />
 *   </ConfirmProvider>
 *
 *   const confirm = useConfirm()
 *   if (!(await confirm({ title: "Stop?", message: "..." }))) return
 */
import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useRef,
  useState,
  type ReactNode,
} from 'react'

export interface ConfirmOptions {
  title: string
  message: string
  confirmLabel?: string
  cancelLabel?: string
  /** Style the confirm button as destructive (red). Default false. */
  destructive?: boolean
}

type ConfirmFn = (opts: ConfirmOptions) => Promise<boolean>

const ConfirmCtx = createContext<ConfirmFn | null>(null)

interface PendingConfirm {
  opts: ConfirmOptions
  resolve: (ok: boolean) => void
}

export function ConfirmProvider({ children }: { children: ReactNode }) {
  const [pending, setPending] = useState<PendingConfirm | null>(null)
  const confirmBtnRef = useRef<HTMLButtonElement>(null)

  const ask = useCallback<ConfirmFn>((opts) => {
    return new Promise<boolean>((resolve) => {
      setPending({ opts, resolve })
    })
  }, [])

  // Focus the confirm button when the dialog opens — keyboard users
  // get an immediate target without having to tab through the page.
  useEffect(() => {
    if (pending) confirmBtnRef.current?.focus()
  }, [pending])

  // Esc cancels, Enter confirms.
  useEffect(() => {
    if (!pending) return
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.preventDefault()
        pending.resolve(false)
        setPending(null)
      } else if (e.key === 'Enter') {
        e.preventDefault()
        pending.resolve(true)
        setPending(null)
      }
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [pending])

  return (
    <ConfirmCtx.Provider value={ask}>
      {children}
      {pending && (
        <div
          role="dialog"
          aria-modal="true"
          aria-labelledby="confirm-title"
          aria-describedby="confirm-message"
          className="fixed inset-0 z-50 flex items-center justify-center bg-black/70 backdrop-blur-sm"
          onClick={() => {
            pending.resolve(false)
            setPending(null)
          }}
        >
          <div
            className="bg-[#1a1a1a] border border-[#444] rounded-xl p-6 max-w-md w-full mx-4 shadow-2xl"
            onClick={e => e.stopPropagation()}
          >
            <h3 id="confirm-title" className="text-[#ffb81c] text-base font-bold uppercase tracking-wider mb-3">
              {pending.opts.title}
            </h3>
            <p id="confirm-message" className="text-gray-300 text-sm mb-6 whitespace-pre-line">
              {pending.opts.message}
            </p>
            <div className="flex justify-end gap-3">
              <button
                onClick={() => {
                  pending.resolve(false)
                  setPending(null)
                }}
                className="px-4 py-2 bg-[#333] text-gray-300 rounded-lg text-sm font-medium hover:bg-[#444]"
              >
                {pending.opts.cancelLabel ?? 'Cancel'}
              </button>
              <button
                ref={confirmBtnRef}
                onClick={() => {
                  pending.resolve(true)
                  setPending(null)
                }}
                className={`px-4 py-2 text-white rounded-lg text-sm font-bold ${
                  pending.opts.destructive
                    ? 'bg-red-700 hover:bg-red-600'
                    : 'bg-green-700 hover:bg-green-600'
                }`}
              >
                {pending.opts.confirmLabel ?? 'OK'}
              </button>
            </div>
          </div>
        </div>
      )}
    </ConfirmCtx.Provider>
  )
}

/**
 * Returns the `confirm` function for the nearest `ConfirmProvider`.
 * Throws if used outside a provider so the failure is loud rather
 * than silently never resolving.
 *
 * Co-located with the provider deliberately — they share the
 * private context object. Splitting into separate files for the
 * sake of fast-refresh granularity costs more than it's worth at
 * this app's scale.
 */
// eslint-disable-next-line react-refresh/only-export-components
export function useConfirm(): ConfirmFn {
  const ctx = useContext(ConfirmCtx)
  if (!ctx) {
    throw new Error('useConfirm() must be used inside <ConfirmProvider>')
  }
  return ctx
}
