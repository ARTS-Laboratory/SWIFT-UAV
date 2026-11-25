# sitl - the ArduPlane side

Every subfolder is on the MATLAB path after `swift_setup`. For everyday use you only need `swift_run`; the rest is what it calls.

## run/ - one command per flight
| File | What it does |
|---|---|
| `swift_run.m` | `swift_run E` plans, launches ArduPlane, runs the model, flies, stops and analyses. Also `list`, `status`, `rtl`, `stop`, `analyze`. Works on Windows (WSL), macOS and Linux. |
| `swift_autofly.py` | Runs next to ArduPlane: uploads the mission, checks every parameter against the `.param` files, waits for GPS/EKF, FBWA -> arm -> AUTO, follows the flight, reports the landing. Needs Python 3 and `pymavlink`. |
| `sessions/swift_session_C.m`, `_D.m`, `_E.m` | Settings for each session: planner, `.param` files in load order, results folder, analysis steps. |
| `sessions/swift_session_template.m` | Copy this to make a new session (instructions inside). |

## missions/ - what each session flies
| File | What it does |
|---|---|
| `swift_sitl_cub.m` | Session C planner: full circuit from KCUB runway 13 with climbs, turns, loiter, spiral and autoland; leg-by-leg battery estimate. |
| `swift_sitl_cub_d.m` | Session D planner: staircase climbs, figure-8, spirals and slalom to compare flight power with the MAD X7215 bench table. |
| `swift_sitl_cub_e.m` | Session E planner: sweeps airspeed for the best range, solves the turn-back point for a chosen landing reserve, out-and-back mission. |
| `swift_sitl_cub*.waypoints`, `swift_sitl_cub*.param` | The mission and session settings each planner writes (regenerated on every run). |

## ardupilot/ - ArduPlane settings shared by every session
| File | What it sets |
|---|---|
| `swift_sitl.param` | Airspeeds, bank and pitch limits, TECS climb/sink, takeoff, waypoint radius, return-to-launch (RTL) height. |
| `swift_sitl_batt.param` | Battery monitor, capacity, low/critical voltage and mAh failsafes (written by `swift_battery_add`). |
| `swift_sitl_vtail.param` | V-tail outputs: SERVO2/4 = left/right ruddervator, mixing gain (written by `swift_vtail_add`). |

ArduPlane loads them after its own `plane.parm`, then the session's own file; later files win.

## bridge/ - Simulink <-> ArduPlane
| File | What it does |
|---|---|
| `ap_json_step.m` | One lockstep frame of the JSON interface: sends gyro, accelerometer, position, velocity, attitude, airspeed and battery to ArduPlane, returns its 16 servo PWM values. Called by the model; never call it yourself. |
| `swift_sitl_patch.m` | Ground-start patch (initial state on the runway, ground contact, test propulsion). Already applied to the shipped model. |

## checks/ - before and after a flight
| File | What it does |
|---|---|
| `swift_sitl_preflight.m` | Run before every session (`swift_run` does): fresh parameters, key numbers, model patch, surface signs, tilt table off, UDP port free. |
| `swift_sitl_signcheck.m` | Proves each control surface moves the right way: `predict` the signs, a virtual tilt-table run (`jig`), then `report`. |
| `swift_plot_selftest.m` | Runs every plotting script on many kinds of flight (saved runs plus edited copies) to catch plotting bugs. |

## analysis/ - after a flight
| File | What it does |
|---|---|
| `swift_plot_flight.m` | Main flight plots and `summary.txt`: overview, ground track, attitude, aero and controls, takeoff roll, landing, battery on track, numbered events. |
| `swift_range_check.m` | Range and efficiency: cruise legs, mAh/km, Wh/km, plan vs flight, projected still-air range. |
| `swift_vtail_check.m` | Six PASS/WARN/FAIL checks on the V-tail: logging, mixing, travel, pitch and yaw sign, straight-flight trim. |
| `swift_motor_check.m` | Compares flight power with the MAD X7215 bench table at steady points. |
| `swift_flight_video.m` | MP4 of the whole flight: 3D chase view, instruments, mini-map, battery and V-tail panels. |
| `swift_flight_events.m` | Builds the numbered list of what happened when (mission items, liftoff, touchdown ...) used by every plot. |
| `swift_mark_events.m` | Draws those numbered markers on any axes. |
| `swift_events_figure.m` | Altitude profile with every event marked, plus the event key. |

Every analysis function takes `out` and `'tag', '<session>'`, and saves into `../data/runs/<tag>/`.
