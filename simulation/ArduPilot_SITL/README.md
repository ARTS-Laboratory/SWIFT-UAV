# ArduPilot_SITL - `swift_6dof_sitl`

The SWIFT UAV model flown by the real ArduPlane autopilot in software-in-the-loop (SITL) simulation. Simulink and ArduPlane exchange one frame per time step over ArduPilot's JSON interface (UDP port 9002): the model sends the aircraft state, ArduPlane sends back servo outputs. How to fly it: [main guide, section 5](../README.md#5-fly-an-arduplane-sitl-session).

```matlab
swift_setup        % in this folder, every MATLAB session
swift_run E        % fly Session E end to end
```

| Path | What it is |
|---|---|
| `swift_setup.m` | Makes this folder the active setup: turns Standalone off, puts these folders and every `sitl/` subfolder on the path, sends build files to `cache/`. |
| **models/** | |
| `swift_6dof_sitl.slx` | The Standalone airframe plus: ArduPilot bridge, servo-PWM decoding into surface angles (with V-tail un-mixing), three-wheel ground contact with rolling friction, accelerometer output, battery voltage and current sent to ArduPlane. |
| **params/** | |
| `swift_placeholder.m`, `swift_aero_matrix.m`, `swift_vtail.m`, `swift_vtail_add.m`, `swift_liftdrag_add.m` | As in [Standalone](../Standalone/README.md) - this folder's own copies. `swift_liftdrag_add` has not been run on this model yet: run it once to log lift and drag (`out_aero`). |
| `swift_sitl_params.m` | SITL-only settings added to `P`: bridge rate, ground gear (stiffness, damping, friction, wheel positions), start on the ground or in the air, parked heading, logging rate, the optional virtual tilt table. |
| **battery/** | Same battery model as Standalone; `swift_battery_add.m` here also writes `sitl/ardupilot/swift_sitl_batt.param` (ArduPlane's battery settings). |
| **sitl/** | Everything on the ArduPlane side - see [sitl/README.md](sitl/README.md). |
| **docs/** | |
| `swift_run_README.txt` | Full manual for `swift_run`: steps, commands, options, platforms. |
| `SessionC/D/E_CUB_README.txt` | What each session flies, its estimates and its manual run order. |
| `SWIFT_SITL_Session*_KCUB_Guide.html` | Archived step-by-step guides for flying each session by hand. Open in a browser. |
| **data/** | |
| `UAV Specs.xlsx` | The airframe specification sheet. |
| `runs/<session>/` | Results of each flight: plots, summaries and event lists (committed); `.BIN` logs, `.mat` data and video (kept locally only). |
