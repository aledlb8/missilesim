# Engine exhaust

Engine fire is an attached emission/absorption volume, rendered in the HDR effects
pass before bloom and tone mapping. It uses 40 stable integration samples, a tapered
envelope, animated flow and axial shock cells. Jet and rocket presets use different
lengths, expansion, colors and throttle response. Smoke and refractive heat haze
remain separate particles downstream. This is an artistic real-time approximation,
not a combustion or compressible-flow solver.

`export_aircraft_obj.py` derives `# exhaust x y z dx dy dz radius` comments from the
actual nozzle outlet rings in `aircraft.glb`. The OBJ loader applies the mesh's
pre-transform and normalization to those sockets. Rendering and attachments then
share the same object matrix, including jet bank and scale. The exporter also
adds recessed dark fighter nozzle baffles to hide the open tube and intersecting
tailplane geometry in the source asset. Regenerate both models with:

```powershell
python tools/export_aircraft_obj.py assets/models/aircraft.glb
```

Replacement OBJ models need their own socket comments; missing metadata emits no
attached flame. Do not restore estimated offsets in the application. Engine volumes
are submitted anew each rendered frame, capped at 256, and disappear immediately
on cutoff, zero fuel or inactive aircraft. Their animation uses the effects clock,
so pausing does not accumulate fire. Both HDR and legacy rendering use detached
scene depth for volume clipping.

After building the app with MSVC/Ninja, run the GPU regression:

```powershell
python tools/verify_exhaust.py
python tools/verify_exhaust.py --motion
```

The second command also writes 180 orbit/bank review frames. Results are under
`build/exhaust-review`: eight model views, `verification.txt`, and optional
`motion/frame-*.ppm`. Checks cover geometry-derived sockets, banking, shutdown,
fuel, paused rendering, equal-time images at 30/60/144 FPS, inside/end-on views,
depth occlusion, invalid inputs, and OpenGL errors. GPU query measurements include
the isolated particle pass and its legacy depth copy, not total application cost;
cost depends strongly on the number of visible pixels and overlapping plumes.
