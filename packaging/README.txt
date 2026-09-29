MissileSim
==========

A real-time 3D missile engagement sandbox: guided missiles, AI fighter jets
that dodge and drop flares, and physically simulated sound.


Running it
----------
1. Extract the whole zip somewhere (Desktop, Documents, ...). Keep the
   "assets" folder next to MissileSim.exe.
2. Double-click MissileSim.exe.
3. Choose START ENGAGEMENT, then press F to launch.

Windows SmartScreen may warn that the app is from an unknown publisher,
because it is not code-signed. Click "More info" -> "Run anyway".

Needs 64-bit Windows 10 or 11 and a graphics card with OpenGL 4.5 support
(any NVIDIA/AMD card from the last ~10 years, or a recent Intel iGPU).
Nothing else needs to be installed.


Controls
--------
  F                     Launch the missile
  R                     Seeker cue: lock a target before launch
  Enter                 Pause / resume
  V                     Cycle camera: free, missile, fighter
  C                     Frame the whole engagement
  W A S D               Move the free camera
  Space / Ctrl          Camera up / down
  Shift                 Move the camera faster
  Right mouse + drag    Look around
  Tab                   Control panel (tune the missile, targets, world)
  H                     Hide / show the HUD
  F11                   Fullscreen
  Esc                   Menu (settings, restart, quit)

Settings > Display has window mode, V-sync and interface size;
Settings > Graphics and Audio have the rest.


Files it creates
----------------
Next to MissileSim.exe:
  config\user_settings.ini   your settings (delete it to reset to defaults)
  imgui.ini                  interface state
  missilesim.log             log of the last run - send this along if
                             something goes wrong
