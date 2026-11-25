# SWIFT-UAV Simulation

Six-degree-of-freedom (6-DOF) flight simulation of the **SWIFT UAV** (Pelican V0.3): a 2.6 m span, twin-boom, inverted-V-tail pusher with a MAD X7215 motor and a 12S lithium-polymer (LiPo) battery. Built in MATLAB/Simulink, it can fly two ways:

- **Standalone** - the airframe on its own, on fixed control inputs. Good for checking aerodynamics, trim, battery draw and control response.
- **ArduPilot SITL** - the same airframe flown by the real ArduPlane autopilot software in software-in-the-loop (SITL) simulation, through ArduPilot's JSON interface. Good for full missions: takeoff, waypoints, landing, battery use and range, all with the real autopilot code.

One command flies a whole SITL mission and produces the plots. Pick a session by its letter:

```matlab
swift_run list      % the sessions you can fly (C, D, E, and any you add)
swift_run <letter>  % e.g. swift_run C, swift_run D, swift_run E
```

Prefer buttons? Open `swift_app.m` in MATLAB and press **Run** - see [Click-to-run window](#click-to-run-window).

---

## Contents

- [Click-to-run window](#click-to-run-window)
1. [What is in this folder](#1-what-is-in-this-folder)
2. [Requirements](#2-requirements)
3. [First-time setup](#3-first-time-setup)
4. [Run the Standalone model](#4-run-the-standalone-model)
5. [Fly an ArduPlane SITL session](#5-fly-an-arduplane-sitl-session)
6. [Change the aircraft](#6-change-the-aircraft)
7. [What a run produces](#7-what-a-run-produces)
8. [Conventions](#8-conventions)
9. [Troubleshooting](#9-troubleshooting)
10. [Status and limitations](#10-status-and-limitations)

---

## Click-to-run window

Everything below can be done with buttons instead of commands.

1. In MATLAB, open `swift_app.m` (in this folder) and press **Run** (or type `swift_app` here).
2. Pick a tab:

| Tab | Buttons |
|---|---|
| **Standalone** | *Run the model* (tick or untick *With 4.5 kg payload*), *Open the model*, then *Battery plots*, *Lift & drag plots*, *V-tail check* |
| **ArduPilot SITL** | choose a session from the list, then *Check (plan only)*, *Fly*, *Return & land*, *Stop now*, *Re-analyse last run*, *Flight status*, *Open results folder* |

- Each button makes the right setup active first, so there is no need to run `swift_setup` yourself.
- Tick boxes on the SITL tab turn the MAVProxy map, the analysis and the flight video on or off.
- The line under the tabs shows the live flight status; the box at the bottom keeps a log of what each button did. Full detail still prints in the Command Window.
- New sessions you add (section 5.6) appear in the list automatically.

The window only presses the same commands described below, so the two ways can be mixed freely. Needs MATLAB R2020b or newer.

---

## 1. What is in this folder

| Folder | Model | Use it to |
|---|---|---|
| [`Standalone/`](Standalone) | `swift_6dof.slx` | study the airframe on its own (no autopilot) |
| [`ArduPilot_SITL/`](ArduPilot_SITL) | `swift_6dof_sitl.slx` | fly missions with ArduPlane in the loop |
| [`docs/`](docs) | - | the V-tail design note |
| `swift_app.m` | - | click-to-run window for both setups |

**The two setups never share files.** Each folder holds its own copy of everything its model needs, plus its own `swift_setup.m` that turns the other setup off. Some files exist in both folders under the same name (for example `swift_placeholder.m`). Editing one copy does **not** change the other; copy a change across by hand if both models should get it.

Every folder has its own README with a file-by-file list:
[Standalone](Standalone/README.md) · [ArduPilot_SITL](ArduPilot_SITL/README.md) · [ArduPilot_SITL/sitl](ArduPilot_SITL/sitl/README.md)

```
simulation/
├── README.md                 this guide
├── swift_app.m               click-to-run window (both setups)
├── docs/                     V-tail design note
├── Standalone/               swift_6dof.slx + its parameters, battery model, data
└── ArduPilot_SITL/           swift_6dof_sitl.slx + parameters, battery, data
    ├── sitl/                 everything on the ArduPlane side
    │   ├── run/              swift_run (one command per flight), autofly, session settings
    │   ├── missions/         mission planners and the .waypoints/.param files they write
    │   ├── ardupilot/        shared ArduPlane parameter files
    │   ├── bridge/           the Simulink <-> ArduPlane JSON bridge
    │   ├── checks/           preflight and sign checks
    │   └── analysis/         post-flight plots, range/V-tail/motor checks, flight video
    ├── docs/                 swift_run manual, per-session notes and archived guides
    └── data/runs/            results, one folder per session
```

---

## 2. Requirements

| | Standalone | ArduPilot SITL |
|---|---|---|
| MATLAB + Simulink | R2026a or newer | R2026a or newer |
| Aerospace Blockset | required | required |
| ArduPilot (ArduPlane 4.5+) with `sim_vehicle.py` | - | required |
| MAVProxy and Python 3 | - | required (MAVProxy installs `pymavlink`) |
| Operating system | Windows, macOS, Linux | Windows (ArduPilot in WSL), macOS, Linux |

- The `.slx` models are saved in **R2026a** and do not open in older releases. Someone with R2026a can export a copy - see [Older MATLAB](#older-matlab).
- WSL is the Windows Subsystem for Linux: Ubuntu running inside Windows, which is how ArduPilot is normally built on a Windows PC.
- No toolbox beyond Simulink and Aerospace Blockset is used. `udpport`, the plotting and the video writer are all part of base MATLAB.

---

## 3. First-time setup

### 3.1 MATLAB side (everyone)

Nothing to install. Clone or download the SWIFT-UAV repository anywhere; every script finds its files relative to its own location.

```matlab
cd(fullfile('<repo>', 'simulation', 'Standalone'))   % <repo> = where you put SWIFT-UAV
swift_setup
```

`swift_setup` puts the folder's code on the MATLAB path, sends Simulink's build files to `cache/` and prints `STANDALONE is ACTIVE`. Run it once per MATLAB session, and again whenever you switch between Standalone and ArduPilot_SITL.

### 3.2 ArduPilot side (SITL only)

Follow ArduPilot's own guides to build ArduPlane SITL. Use the default location `~/ardupilot` if you can; otherwise pass `'ardupilot', '<path>'` to `swift_run`.

- **Windows:** [Set up the build environment in WSL](https://ardupilot.org/dev/docs/building-setup-windows11.html), then [SITL on Windows using WSL](https://ardupilot.org/dev/docs/sitl-on-windows-wsl.html)
- **Linux:** [Setting up SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- **macOS:** the macOS build-environment page of the [ArduPilot developer docs](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

On Ubuntu (including Ubuntu in WSL) that comes down to:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf plane
```

Check it works once on its own: `cd ~/ardupilot/ArduPlane && ../Tools/autotest/sim_vehicle.py -v ArduPlane --console --map` should open a map with a plane on it. Close it with Ctrl+C.

**Windows firewall:** ArduPlane in WSL talks to MATLAB on Windows over UDP (User Datagram Protocol) port 9002. The first time, Windows may ask whether MATLAB may accept connections - allow it on private networks. If ArduPlane keeps waiting for the simulator, see [Troubleshooting](#9-troubleshooting).

---

## 4. Run the Standalone model

```matlab
cd(fullfile('<repo>', 'simulation', 'Standalone')); swift_setup
open_system('swift_6dof')      % then press Run
```

**What it does:** starts at 100 m in trimmed level flight at 1.3 x stall speed and flies 60 s on fixed control inputs - the trim aileron, elevator, rudder and throttle that `swift_placeholder.m` works out. With no autopilot, any drift away from straight and level shows the airframe's own stability.

**Look at the results:**

```matlab
swift_battery_plot(out)                     % state of charge, voltage, current, power
swift_liftdrag_add('plot', out)             % lift, drag, CL and CD over the run
swift_vtail_check(out)                      % ruddervator travel and mixing checks
```

**Try something different:**

| To | Do this before pressing Run |
|---|---|
| fly empty (no 4.5 kg payload) | `P_with_payload = false; swift_placeholder` |
| run longer | `P.sim.tfinal = 300;` |
| apply a control input | `P.init.delta_e = P.init.delta_e - deg2rad(2);` (2 deg more nose-up elevator) |
| add a steady wind | `P.env.wind_ned = [0; 5; 0];` (5 m/s from the west, blowing east) |
| go back to the defaults | `clear P` (the model rebuilds `P` from `swift_placeholder` on the next run) |

The model only rebuilds `P` when it is missing, so a change you make at the command line stays in force until you `clear P` or re-run `swift_placeholder`.

---

## 5. Fly an ArduPlane SITL session

### 5.1 The one-command way

```matlab
cd(fullfile('<repo>', 'simulation', 'ArduPilot_SITL')); swift_setup
swift_run list                  % the sessions you can fly
swift_run('E', 'dry', true)     % optional: plan it and show the ArduPlane script, launch nothing
swift_run E                     % fly it
```

`swift_run E` then does everything:

1. **Preflight** - rebuilds the parameters, checks the model, the control-surface signs and that UDP port 9002 is free. It stops on any `[FAIL]`.
2. **Plan** - the session's planner writes the mission (`.waypoints`) and its ArduPlane settings (`.param`).
3. **Launch** - opens ArduPlane (`sim_vehicle.py`) with the right parameter files and runway location: a console window in WSL on Windows, Terminal on macOS, a terminal (or the background) on Linux.
4. **Run** - presses Run on `swift_6dof_sitl`.
5. **Fly** - `swift_autofly.py` uploads the mission, reads back **every** parameter and refuses to fly if one is wrong, waits for a GPS fix, then switches FBWA -> arm -> AUTO. Progress is echoed in the MATLAB Command Window every 2 s.
6. **Finish** - after touchdown and the automatic disarm: Simulink stops, ArduPlane closes, the ArduPilot `.BIN` log is copied, `out` is kept as `outE`, and the session's analysis runs (plots, range check, V-tail check, video).

Results go to `ArduPilot_SITL/data/runs/<session>/`.

**While it flies** the Command Window stays free:

| Command | Does |
|---|---|
| `swift_run status` | where the flight is |
| `swift_run rtl` | ArduPlane returns and lands on the runway |
| `swift_run stop` | stop now (Simulink first, then ArduPlane); the partial flight is still analysed |

**Sessions included** (all from Owens Downtown Airport, KCUB, Columbia SC, runway 13):

| Session | Flight | About |
|---|---|---|
| C | full circuit with climbs, turns, loiter, spiral and autoland | about 12 km, 20 min logged |
| D | staircase climbs, figure-8, spirals and slalom + motor check against the bench table | about 17 min logged |
| E | out-and-back range test at the best-range airspeed + full V-tail check | 34 km, 27 min |

These missions exist only for simulation. Session E's route runs beyond visual line of sight under controlled airspace; do not fly them for real.

### 5.2 Options

`swift_run('E', name, value, ...)`

| Option | Default | Meaning |
|---|---|---|
| `'plan'` | `{}` | options for the session's planner, e.g. `{'land_mAh', 4000}` or `{'alt', 120}` |
| `'skip'` | `{}` | analysis steps to leave out, e.g. `{'video'}` (the video takes a few minutes) |
| `'analyze'` | `true` | `false` = fly only |
| `'map'` | `true` | `false` = no MAVProxy map and console windows |
| `'auto'` | `true` | `false` = launch only; you fly it from MAVProxy or a ground station |
| `'addr'` | automatic | address ArduPlane sends to (Windows: found from WSL and remembered; macOS/Linux: 127.0.0.1) |
| `'ardupilot'` | `'~/ardupilot'` | where ArduPilot is (inside WSL on Windows) |
| `'distro'` | default | Windows only: which WSL distribution |
| `'wipe'` | `true` | start ArduPlane with fresh parameters (keep it on) |
| `'force'` | `false` | launch even if preflight reports a problem |
| `'dry'` | `false` | plan and write the script, launch nothing |

### 5.3 Watch it in a ground station

ArduPlane also sends MAVLink telemetry to UDP port 14550. Connect Mission Planner or QGroundControl to UDP 14550 to watch the flight on a map. Watching needs no extra settings; `swift_run` does the flying.

### 5.4 Fly it yourself

`swift_run('E', 'auto', false)` launches ArduPlane and the model and then leaves the flying to you: load the waypoints, arm and change modes in MAVProxy or a ground station. The manual steps for each session are in `ArduPilot_SITL/docs/`.

### 5.5 Analyse a run again

```matlab
out = outE;                      % or load a saved run
swift_run analyze E              % the whole analysis set again
swift_plot_flight(out, 'tag', 'sessionE_cub')     % or any single step
```

### 5.6 Make a new session

1. Copy `ArduPilot_SITL/sitl/run/sessions/swift_session_template.m` to `swift_session_F.m` in the same folder and rename the function inside to `swift_session_F`.
2. Fill in the fields: title, results folder, which planner writes the mission, which `.param` files ArduPlane loads, which analysis to run.
3. `swift_run('F', 'dry', true)` to check it, then `swift_run F`.

A variant of an existing flight needs no new code, only planner options. The template's own example is Session E flown at 120 m with a 4000 mAh landing reserve. A new mission needs a planner (see the three in `sitl/missions/`) that writes a `.waypoints` file and returns its path and start point.

What `swift_run` expects of a mission:
- it takes off from the ground: FBWA, arm, then AUTO from a takeoff waypoint
- it ends with a landing; `swift_run` treats the automatic disarm after touchdown as the end of the flight
- for `swift_run rtl` to land the aircraft, the session's parameters set `RTL_AUTOLAND = 2`

---

## 6. Change the aircraft

**Every number the models use lives in one file:** `params/swift_placeholder.m`. No block in either model contains a number; every block dialog points into the `P` structure that file builds. Change a value there, run `swift_placeholder`, and the model picks it up.

Each value carries a source tag so you can tell measured numbers from guesses:

| Tag | Source |
|---|---|
| `[SHEET]` | `data/UAV Specs.xlsx` |
| `[SPEC]` | manufacturer data (motor bench table, servo datasheet) |
| `[CAD]` | measured from the CAD model |
| `[STD]` | physical constant or published standard |
| `[EST]`, `[EST->CAD]`, `[EST->AERO]`, `[EST->PROP]` | estimate, with what should replace it |

Related files in `params/`:

| File | Role |
|---|---|
| `swift_aero_matrix.m` | packs the aerodynamic derivatives into the single 5 x 9 matrix the model multiplies by. After changing a derivative at the command line: `P = swift_aero_matrix(P);` |
| `swift_vtail.m` | tail geometry from the CAD and the derivatives the inverted-V tail contributes; called from `swift_placeholder` |
| `swift_sitl_params.m` | (SITL only) bridge rate, ground gear, start mode and SITL-only overrides |

**Installers.** `swift_battery_add`, `swift_liftdrag_add` and `swift_vtail_add` add or rebuild a part of a model (battery subsystem, lift and drag logging, V-tail mixing). `swift_6dof` has all three; `swift_6dof_sitl` has the battery and V-tail but **not yet** the Lift & Drag logger - run `swift_liftdrag_add` once in `ArduPilot_SITL/` to add `out_aero` there. Otherwise run an installer again only if you change what it builds. Each backs the model up to `archive/` first, and each must be run in each folder separately.

**Keep both copies in step.** A change to `swift_placeholder.m`, `swift_aero_matrix.m`, `swift_vtail.m` or the battery model that both setups should get has to be made in `Standalone/` **and** `ArduPilot_SITL/`.

---

## 7. What a run produces

**Logged signals** (in `out`, as timeseries):

| Variable | Contents | Units |
|---|---|---|
| `out_pos` | position, north-east-down (NED) from the start point | m |
| `out_vb` | velocity in body axes [u v w] | m/s |
| `out_euler` | roll, pitch, yaw | rad |
| `out_om` | body rates [p q r] | rad/s |
| `out_V`, `out_alpha`, `out_beta` | airspeed, angle of attack, sideslip | m/s, rad, rad |
| `out_qbar` | dynamic pressure | Pa |
| `out_delta` | aileron, elevator, rudder, throttle | rad, rad, rad, 0-1 |
| `out_rv` | left and right ruddervator | rad |
| `out_batt` | state of charge, pack voltage, current, power, mAh and Wh used, cell voltage, shaft power, motor load, status | see `swift_battery_fcn.m` |
| `out_aero` | lift, drag, CL, CD (Standalone; SITL after `swift_liftdrag_add`) | N, N, -, - |
| `out_accel` | (SITL) specific force sent to ArduPlane | m/s^2 |

**Run folder** `ArduPilot_SITL/data/runs/<session>/`:

| File | From |
|---|---|
| `swift_sitl_*.png` | flight overview, ground track, attitude, aero and controls, takeoff, landing, battery on track, events |
| `swift_battery.png`, `battery_summary.txt` | battery |
| `swift_range_check.png`, `range_summary.txt`, `range_legs.csv` | range check |
| `swift_vtail_check.png`, `vtail_summary.txt` | V-tail check |
| `swift_motor_check_*.png`, `motor_summary.txt` | motor check |
| `summary.txt`, `events.csv` | headline numbers and the numbered event list |
| `signals.mat`, `<session>.BIN`, `swift_flight.mp4` | data, ArduPilot log, video - kept locally, **not** committed (too large for GitHub) |

---

## 8. Conventions

- **Units:** SI throughout (m, kg, s, N, rad inside the model). Plots show degrees and minutes where that reads better.
- **Axes:** position in north-east-down (NED); body axes x forward, y right, z down.
- **Control signs:** positive elevator, rudder and aileron follow the aerodynamic derivatives in `swift_placeholder.m`. `swift_sitl_signcheck` proves ArduPlane's servo outputs move each surface the right way.
- **The `P` structure** in the base workspace holds every parameter. `clear P` resets to the files.
- **Runway frame** in the mission planners: x along runway 13 from its threshold, y to the right of the centreline.

---

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `swift_6dof... not found` or the wrong copy of a function runs | run `swift_setup` in the folder you are working in |
| Model will not open: "created with a newer version" | the models need R2026a - see [Older MATLAB](#older-matlab) |
| Preflight `[FAIL] UDP 9002 is in use` | another MATLAB, or a crashed run, holds the port: `clear ap_json_step`, or restart MATLAB |
| ArduPlane keeps waiting for the simulator (Windows) | allow MATLAB through Windows Firewall for UDP; check the address with `swift_run('E', 'dry', true)` |
| Autofly `[FAIL] <param> is ..., wants ...` | ArduPlane did not load a parameter file; keep `'wipe'` on, and check the file is listed in the session |
| `pymavlink is missing` in the ArduPlane window | `pip3 install --user pymavlink MAVProxy` inside Ubuntu/WSL (or your macOS/Linux Python) |
| Landed but nothing happens in MATLAB | press Stop in Simulink; `swift_run` closes ArduPlane and runs the analysis on its own |
| Plots wrong after editing a shared file | the other setup still has the old copy - see [Keep both copies in step](#6-change-the-aircraft) |

### Older MATLAB

Someone with R2026a can save copies for an older release:

```matlab
Simulink.exportToVersion('swift_6dof',      'swift_6dof_R2024b.slx',      'R2024b')
Simulink.exportToVersion('swift_6dof_sitl', 'swift_6dof_sitl_R2024b.slx', 'R2024b')
```

Rename the exported file back to `swift_6dof.slx` / `swift_6dof_sitl.slx` in the copy that will run on the older release. The scripts themselves need R2020b or newer.

---

## 10. Status and limitations

- **Aerodynamics are mostly estimates.** Lift comes from NACA 4415 finite-wing theory and the tail derivatives from the CAD geometry. Most other stability derivatives, the inertia and the centre of gravity (CG) are still generic small-UAV estimates, tagged `[EST->AERO]` / `[EST->CAD]`. A vortex-lattice analysis (for example AVL or XFLR5) and a CAD mass-properties export would replace most of them.
- **Lift is linear in angle of attack.** The model does not stall. Sessions C-E stay inside the linear range (peak angle of attack under 10 deg), but the model cannot warn of a stall.
- **Thrust** comes from the MAD X7215 bench table, with an estimated drop-off with airspeed. It does not yet drop with battery voltage.
- **Actuators are ideal** (no servo rate limit or lag), and the air is smooth apart from an optional steady wind.
- **SITL timing:** the bridge runs in lockstep with ArduPlane at 1200 Hz (`P.sitl.rate`), so long missions take a while to simulate.

Part of [SWIFT-UAV](https://github.com/ARTS-Laboratory/SWIFT-UAV), ARTS Lab, University of South Carolina. Licensing and citation: see the repository root.
