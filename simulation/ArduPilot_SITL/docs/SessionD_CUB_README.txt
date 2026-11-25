SWIFT UAV - SITL Session D: motor-validation flight from KCUB (v0.1, 11 Nov 2026)
================================================================================
Purpose: one full ArduPlane AUTO flight from Owens Downtown (KCUB) runway 13 - takeoff roll,
climbs, a propulsion staircase, a figure-8, spirals, a slalom, the right-hand pattern, autoland
and rollout - flown with the REAL MAD X7215 IPE 220KV / 21x10 / AMPX 120A / 12S motor data
(swift_placeholder v0.6), so every swift_plot_flight figure, the battery figures and the new
swift_motor_check figures have data from one run. Session C files are not touched.

Guide (step by step, archived): docs\SWIFT_SITL_SessionD_KCUB_Guide.html - open in a browser

Folder: simulation\ArduPilot_SITL\sitl\missions (planner + mission files)   One command: swift_run D (docs\swift_run_README.txt)
  swift_sitl_cub_d.m           NEW  mission + param writer, leg-by-leg throttle/current/SOC estimate
  swift_motor_check.m          NEW  post-flight: steady throttle points vs the MAD bench table
  swift_sitl_cub_d.waypoints   written by swift_sitl_cub_d (37 items, ceiling 120 m)
  swift_sitl_cub_d.param       written by swift_sitl_cub_d (Session C landing set +
                                TRIM_THROTTLE 54, TECS_CLMB_MAX 2.0 for this flight only)

Flight (runway frame: x along RWY 13 from the threshold, y right of centreline), 120 m max AGL
  1 Takeoff roll at full throttle, climb to 30 m                    [~97 A peak]
  2 Max-rate climb to 90 m, right turns up to 120 m                  [~83 % throttle]
  3 Propulsion staircase on y = 1300 m, 700 m per step:
      22 m/s level 51 % | 22 m/s descent 39 % | 24 m/s 58 % | 26 m/s 69 % | 27.5 m/s 82 %
  4 Figure-8: one left + one right circle, 100 m radius, 22 m/s
  5 Left spiral down to 55 m (low throttle), right spiral up to 100 m (high throttle)
  6 Slalom: 7 gates alternating left/right, 450 m apart, 23 m/s
  7 Upwind, two left turns onto downwind, right base, right final, autoland at x = 200 m
Estimate [EST]: ~17.6 km, ~15.4 min arm-to-disarm, SOC 100 -> 57 %, ~4260 mAh / 201 Wh,
peak 97 A, avg 17 A. BATT_LOW_MAH failsafe (RTL at 8000 mAh used) has ~3700 mAh margin.

RUN ORDER - START
 1 MATLAB  cd('<repo>\simulation\ArduPilot_SITL'); swift_setup
 2 MATLAB  swift_sitl_preflight('<addr>')            (<addr> = Windows address seen from WSL)
 3 MATLAB  S = swift_sitl_cub_d('addr','<addr>');     (4x [ OK ] under BATTERY CHECK; the
           propulsion line must say MAD X7215, not SITL-TEST; read the est. thr % column)
 4 MATLAB  open_system('swift_6dof_sitl'); press Run - it waits for ArduPilot
 5 Ubuntu  D="/mnt/c/<repo>/simulation/ArduPilot_SITL/sitl"
           cp "$D"/swift_sitl.param "$D"/swift_sitl_cub_d.param \
              "$D"/swift_sitl_batt.param "$D"/swift_sitl_vtail.param "$D"/swift_sitl_cub_d.waypoints ~/
           cd ~/ardupilot/ArduPlane
           ../Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:<addr> -N -w \
             --custom-location=33.9744563,-81.0019926,58.9,125.5 \
             --add-param-file=../Tools/autotest/models/plane.parm \
             --add-param-file=$HOME/swift_sitl.param \
             --add-param-file=$HOME/swift_sitl_cub_d.param \
             --add-param-file=$HOME/swift_sitl_batt.param \
             --add-param-file=$HOME/swift_sitl_vtail.param \
             --out=udp:<addr>:14550 --console --map
           (swift_sitl_cub_d prints this block with your address filled in - copy it from there)
 6 MAVProxy wait for "EKF3 IMU0 is using GPS", then
           wp load ../../swift_sitl_cub_d.waypoints       (expect 38 items incl. home)
           param show TECS_CLMB_MAX                       (expect 2)
           param show BATT_MONITOR                        (expect 4; console Batt ~50 V)
           mode FBWA -> arm throttle -> mode AUTO
 7 Watch   Mission Planner (UDP 14550, connect, "allow control" not needed to watch) or the
           MAVProxy map. Item numbers to expect: 8-15 staircase, 17-18 figure-8, 19-20 spirals,
           22-28 slalom, 32 landing sequence, 37 LAND. Disarms itself 20 s after touchdown.

RUN ORDER - END
 8 After "Disarming" in MAVProxy: press Stop in Simulink FIRST (so 'out' is written),
   then Ctrl+C in Ubuntu.
 9 MATLAB  outD = out;
           R = swift_plot_flight(outD, 'tag', 'sessionD_cub', 'wp', S.wp_file);
           M = swift_motor_check(outD, 'tag', 'sessionD_cub');
   Everything lands in ArduPilot_SITL\data\runs\sessionD_cub\
   (7 flight figures + battery figure + 3 motor figures, signals.mat, summary.txt,
    motor_steady_points.csv, motor_summary.txt)
10 Optional: copy the ArduPilot .BIN log into the same run folder (as for Session C) and
   pass 'aplog' to swift_plot_flight for the desired-vs-actual attitude overlay.

ABORT / EARLY STOP
  - Something wrong in the air: MAVProxy "mode RTL" - RTL_AUTOLAND = 2 jumps to item 32 and
    lands on runway 13. Then follow steps 8-9 (the plots work on a partial flight).
  - Hard stop: Simulink Stop, then Ctrl+C in Ubuntu. Run "outD = out;" before anything else.

WHAT THE MOTOR CHECK TELLS YOU
  The MAD table is a static (0 m/s) bench test; in flight the airstream unloads the prop, so
  flight points sit BELOW the bench curve at the same throttle - that is expected.
  - Flight points on the model curve -> the Simulink build matches its own equations.
  - Static model vs bench (already known from the table): the single eta_mot = 0.82 is right
    near 50 % throttle but the bench efficiency runs 71-82 %, so the model reads about
    -4 % at 30 %, +8 % at 50 %, +4 % at 70 % and -13 % at 100 % throttle on input power
    (about 94 A vs the bench 108 A at full throttle). A throttle-dependent efficiency from the
    bench table would remove most of that.
  - True in-flight accuracy needs a logged flight of the real aircraft.

CHECKED
  - swift_sitl_cub_d run in Octave 8.4 with the real params: plan table, files written, waypoint
    file decoded back to the runway-frame points (all 37 items, max 120 m).
  - swift_motor_check run in Octave on a synthetic flight: 7 steady stretches found, static
    point found, figures built and saved. Not yet run in MATLAB or against a real SITL run.
  - The landing pattern (items 31-37) is identical to Session C, which landed at 21.1 m/s,
    0.83 m/s sink, 412 m rollout.

This is a simulation plan only. Real flight at CUB sits under the Columbia (CAE) Class C shelf
and needs airspace authorization; the far legs (2.7 km out) are beyond visual line of sight.
