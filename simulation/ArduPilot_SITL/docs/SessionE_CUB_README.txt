SWIFT UAV - SITL Session E: out-and-back range test from KCUB (v0.1, 15 Nov 2026)
================================================================================
Purpose: one long straight leg out along the runway 13 heading, one gentle right 180, and
the same leg back, at the best-range airspeed, flown on the CUB (Owens Downtown) setup from
Sessions C/D. The same flight is the first full SITL check of the inverted-V tail, and it
exercises every plot plus the flight video. Sessions C and D files are not touched.

Guide (step by step, archived): docs\SWIFT_SITL_SessionE_KCUB_Guide.html - open in a browser

Folder: simulation\ArduPilot_SITL\sitl\missions (planner + mission files)   One command: swift_run E (docs\swift_run_README.txt)
  swift_sitl_cub_e.m            NEW  best-range speed sweep, energy-solved turn-back point,
                                     mission + param writer, leg-by-leg estimate
  swift_range_check.m           NEW  post-flight: cruise legs, mAh/km, km/Ah, Wh/km, plan vs
                                     flight, projected still-air range to the reserve
  swift_vtail_check.m           NEW  post-flight: six PASS/WARN/FAIL checks on the V-tail
  swift_flight_video.m          v0.3 SWIFT shape (red pod, twin booms, inverted V with moving
                                     ruddervators, pusher prop) + V-TAIL inset
                                     (replaces v0.1)
  swift_sitl_cub_e.waypoints    written by swift_sitl_cub_e (16 items + home, 100 m cruise)
  swift_sitl_cub_e.param        written by swift_sitl_cub_e (Session D landing set +
                                TRIM_THROTTLE at the best-range value, TECS_CLMB_MAX 2.0)

BEST-RANGE SPEED (max payload, 21.38 kg)  [EST]
  Level flight T = D = q S (CD0 + k CL^2); pack current from swift_battery_fcn; pick the
  airspeed with the lowest mAh per km between AIRSPEED_MIN (22) and 27 m/s.
     V m/s   thr %   I A    P W   Wh/km  km/Ah
     22.0    51.4   16.65   759   9.59   4.76
     22.5    52.8   16.99   775   9.56   4.77   <- fly
     23.0    54.5   17.37   792   9.56   4.77
     24.0    57.9   18.24   830   9.61   4.74
     25.0    63.0   19.25   876   9.73   4.67
  Within 1 % of the best from 22.0 to 24.3 m/s - the curve is flat. ArduPlane flies
  airspeed in AUTO, so the mission commands 22.5 m/s and TECS finds ~53 % throttle;
  TRIM_THROTTLE is set to 53 so it starts there. Recomputed from P on every run.

FLIGHT (runway frame: x along RWY 13 from the threshold, y right of centreline)
  1     Takeoff roll at full throttle, climb to 30 m                       [~97 A peak]
  2-3   Straight climb at 23 m/s to 100 m by x = 1.5 km                    [~74 %]
  4-5   22.5 m/s, OUT leg to x = 14.7 km                                   [~53 %, 16.3 A]
  6-8   Right 180 on a 325 m half-circle (~10 deg bank)
  9     BACK leg on y = 650 m (= right downwind for RWY 13) to x = 2.5 km  [~53 %, 17.1 A]
  10    Descend to 45 m along the lane
  11-16 Session C/D pattern and autoland, touchdown at x = 200 m
Estimate [EST]: ~33.4 km, ~25.7 min arm-to-disarm, SOC 100 -> 30 %, ~7006 mAh / 325 Wh,
lands with ~3000 mAh. BATT_LOW_MAH failsafe (RTL at 8000 mAh used) has ~1000 mAh margin.
Change the turn-back with 'land_mAh' (default 3000) or fix it with 'x_turn' (m).

RUN ORDER - START
 1 MATLAB  cd('<repo>\simulation\ArduPilot_SITL'); swift_setup
 2 MATLAB  swift_sitl_preflight('<addr>')
 3 MATLAB  S = swift_sitl_cub_e('addr','<addr>');    ([ OK ] on every BATTERY AND V-TAIL
           CHECK line incl. "logs out_rv"; "<- fly" on 22.5 m/s; turn back at 14.7 km)
 4 MATLAB  open_system('swift_6dof_sitl'); press Run - it waits for ArduPilot
 5 Ubuntu  D="/mnt/c/<repo>/simulation/ArduPilot_SITL/sitl"
           cp "$D"/swift_sitl.param "$D"/swift_sitl_cub_e.param \
              "$D"/swift_sitl_batt.param "$D"/swift_sitl_vtail.param "$D"/swift_sitl_cub_e.waypoints ~/
           cd ~/ardupilot/ArduPlane
           ../Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:<addr> -N -w \
             --custom-location=33.9744563,-81.0019926,58.9,125.5 \
             --add-param-file=../Tools/autotest/models/plane.parm \
             --add-param-file=$HOME/swift_sitl.param \
             --add-param-file=$HOME/swift_sitl_cub_e.param \
             --add-param-file=$HOME/swift_sitl_batt.param \
             --add-param-file=$HOME/swift_sitl_vtail.param \
             --out=udp:<addr>:14550 --console --map
 6 MAVProxy wait for "EKF3 IMU0 is using GPS", then
           wp load ../../swift_sitl_cub_e.waypoints     (expect 17 items incl. home)
           param show TRIM_THROTTLE                     (expect 53)
           param show SERVO2_FUNCTION / SERVO4_FUNCTION (expect 79 / 80)
           param show BATT_MONITOR                      (expect 4)
           mode FBWA -> arm throttle -> mode AUTO
 7 Watch   item 5 = out leg end, 6-8 = turn back, 9 = back leg, 11 = landing sequence,
           16 = LAND. Disarms itself 20 s after touchdown. Optional live V-tail check in
           Mission Planner Quick tab: ch2out and ch4out move together at rotation, apart in
           the turn.

RUN ORDER - END
 8 After "Disarming": Simulink Stop FIRST, then Ctrl+C in Ubuntu.
 9 MATLAB  outE = out;
           R  = swift_plot_flight(outE, 'tag', 'sessionE_cub', 'wp', S.wp_file);
           RC = swift_range_check(outE, 'tag', 'sessionE_cub');
           VT = swift_vtail_check(outE, 'tag', 'sessionE_cub');
           swift_flight_video(outE, 'tag', 'sessionE_cub', 'speed', 20);   (~77 s of video)
   Everything lands in ArduPilot_SITL\data\runs\sessionE_cub\

WHAT THE V-TAIL CHECK GRADES
  Logged      out_rv present (swift_vtail_add's logger)
  Mixing      delta_e = (rv_L + rv_R)/2 and delta_r = (rv_L - rv_R)/2 match what the 6-DOF flew
  Travel      on a +/-25 deg stop < 1 % of airborne time
  Pitch sign  both surfaces trailing edge up at takeoff rotation
  Yaw sign    Cn_dr*delta_r has the sign of Cl_da*delta_a (rudder into the turn, from
              ArduPlane's aileron-to-rudder mix). FAIL -> swap SERVO2_FUNCTION 79 / SERVO4_FUNCTION 80
  Straight    |delta_r| < 1 deg on wings-level flight
  On Session D's real ArduPlane log (pre-V-tail, from out_delta) Yaw sign already passes:
  slope +0.16, same sign in 100 % of 28 383 samples.

WHAT THE RANGE CHECK REPORTS
  mAh/km creeps up as the pack voltage falls (same power, more current), so the back leg
  reads a few percent worse than the out leg in mAh/km. Wh/km is the fair comparison and
  should match on both legs (~9.56 Wh/km at 22.5 m/s in the model). A partial flight is
  compared with the plan at the same distance flown.

CHECKED (Octave 8.4, this session)
  - swift_sitl_cub_e with the real P (max payload): sweep, turn-back solve, files written.
  - A 25.5 min synthetic flight that follows this mission (kinematic, with the V-tail
    mixing and swift_battery_fcn): swift_plot_flight (all 7 figures + battery + events,
    18 numbered points on the long thin track), swift_range_check, swift_vtail_check (all
    PASS), swift_flight_video stills. It used 7151 mAh against the 7006 planned (+2.1 %).
  - Same scripts on: Session D's real log (no out_rv), a log cut at 8 min, a 5 s log, and
    a flight with left/right swapped (Yaw sign FAIL with the swap advice).
  - V-tail model geometry rendered on its own: inverted V apex up, + deflection = trailing
    edge down on both panels.
  Not yet run in MATLAB or against a real SITL run.

This is a simulation plan only. The out leg runs ~15 km SE of CUB, under the Columbia (CAE)
Class C shelf and toward the Fort Jackson restricted areas, far beyond visual line of sight.
