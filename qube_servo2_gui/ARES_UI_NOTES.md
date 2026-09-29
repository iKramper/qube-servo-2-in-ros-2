# QUBE-Servo 2 HMI — Ares UI v1.6

## Reference & Control layout

The lower-left `REFERENCE COMMAND` deck is now a single adaptive workstation.

### AUTOMATIC
The panel contains three integrated tabs:

1. **PARAMETERS** — waveform configuration and automatic run/safe/emergency actions.
2. **REFERENCE PREVIEW** — eight compact configuration chips plus the waveform scope.
3. **CONTROL TOPOLOGY** — the controller-dependent block diagram and control law.

No second preview/topology panel is placed below `REFERENCE COMMAND`.

### MANUAL
The automatic tabs are replaced by the manual actuator instrument. The position dial and velocity throttle get the full lower-left deck. `SAFE STOP / HOLD` and `EMERGENCY ZERO / DEACTIVATE` are immediately below the instrument so the manual command and its safety actions remain visually grouped.

The velocity instrument is centered and width-limited so it behaves visually like a vertical aircraft throttle rather than stretching into a horizontal slider.

## Performance

- Manual drag events remain coalesced to about 50 Hz before publication.
- Hidden automatic preview charts are not rebuilt while MANUAL mode is active.
- The original `ScopePlot` hover cursor, interpolation markers, tooltip, wheel time zoom, and Y/FIT controls are preserved.
- The System statistics deck from v1.5 is retained.
