# Standalone - `swift_6dof`

The SWIFT UAV six-degree-of-freedom (6-DOF) model on its own, with no autopilot. It starts in trimmed level flight at 100 m and flies 60 s on fixed control inputs. How to run it: [main guide, section 4](../README.md#4-run-the-standalone-model).

```matlab
swift_setup                    % run in this folder first, every MATLAB session
open_system('swift_6dof')      % press Run
```

| Path | What it is |
|---|---|
| `swift_setup.m` | Makes this folder the active setup: turns ArduPilot_SITL off, puts these folders on the path, sends build files to `cache/`. |
| **models/** | |
| `swift_6dof.slx` | The Simulink model: 6-DOF equations of motion, COESA atmosphere, linear aerodynamic matrix plus drag polar, inverted-V-tail mixing, thrust table, battery subsystem, signal logging. |
| **params/** | |
| `swift_placeholder.m` | **Every number the model reads**, each tagged with its source: mass budget, inertia, geometry, aerodynamic derivatives, motor/propeller/battery data, trim, limits. Run it to rebuild `P`; `P_with_payload = false` first for an empty-weight run. |
| `swift_aero_matrix.m` | Packs the derivatives into the 5 x 9 matrix the model multiplies by; also works out aspect ratio, induced-drag factor and stall speed. Re-run after editing a derivative: `P = swift_aero_matrix(P);` |
| `swift_vtail.m` | Inverted-V-tail geometry (from the CAD) and the tail's share of the stability and control derivatives; ruddervator mixing. Called by `swift_placeholder`. |
| `swift_vtail_add.m` | Installer: adds the V-tail mixing block to the model. Already applied; re-run only if the model is rebuilt. |
| `swift_liftdrag_add.m` | Installer for the Lift & Drag logger (`out_aero`), and `swift_liftdrag_add('plot', out)` to plot lift, drag, CL and CD. |
| `swift_mission.m` | Flight profile for a mission sequencer from an earlier version of the model. **Not used** by the current `swift_6dof.slx`. |
| **battery/** | |
| `swift_battery_fcn.m` | The battery model the Battery block runs every step: momentum-theory propeller power, efficiency chain, LiPo open-circuit voltage curve and internal resistance. Callable by hand for one operating point. |
| `swift_battery_plot.m` | Battery plots and summary for a run: state of charge, voltage, current, power, warnings. |
| `swift_battery_add.m` | Installer for the Battery subsystem. Already applied. |
| **data/** | |
| `UAV Specs.xlsx` | The airframe specification sheet the `[SHEET]` values come from. |
| `runs/` | Saved plots (created when you save from a plot function). |

Shares file names with `ArduPilot_SITL/` but not file contents. Copy a change across by hand if both models should get it.
