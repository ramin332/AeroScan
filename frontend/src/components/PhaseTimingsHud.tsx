import { useStore } from '../store';

export function PhaseTimingsHud() {
  const timings = useStore((s) => s.lastPhaseTimings);
  if (import.meta.env.VITE_DEBUG_TIMINGS !== '1') return null;
  if (!timings || timings.length === 0) return null;

  const total = timings.reduce((acc, t) => acc + t.seconds, 0);

  return (
    <div
      style={{
        position: 'fixed',
        right: 12,
        bottom: 12,
        zIndex: 9999,
        background: 'rgba(10, 10, 15, 0.82)',
        color: '#d0e8ff',
        font: '11px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace',
        padding: '8px 10px',
        borderRadius: 6,
        border: '1px solid rgba(120, 160, 220, 0.3)',
        boxShadow: '0 4px 14px rgba(0,0,0,0.4)',
        pointerEvents: 'none',
        maxWidth: 360,
      }}
    >
      <div style={{ color: '#8fb4d9', marginBottom: 4 }}>phase timings</div>
      {timings.map((t, i) => (
        <div key={i} style={{ display: 'flex', justifyContent: 'space-between', gap: 12 }}>
          <span>{t.label}</span>
          <span>{t.seconds.toFixed(3)}s</span>
        </div>
      ))}
      <div
        style={{
          borderTop: '1px solid rgba(120, 160, 220, 0.25)',
          marginTop: 4,
          paddingTop: 3,
          display: 'flex',
          justifyContent: 'space-between',
          color: '#8fb4d9',
        }}
      >
        <span>total</span>
        <span>{total.toFixed(3)}s</span>
      </div>
    </div>
  );
}
