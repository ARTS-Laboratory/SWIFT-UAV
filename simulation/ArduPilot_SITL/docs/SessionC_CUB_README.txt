SWIFT UAV - SITL Session C: full flight from KCUB, with battery (v0.2, 10 Nov 2026)
================================================================================
Purpose: one complete ArduPlane AUTO flight from Owens Downtown (KCUB) runway 13 - takeoff roll,
climbs, speed changes, right and left turns, loiter, descending spiral, right-hand pattern,
autoland and rollout - with the Battery subsystem running, so every swift_plot_flight figure
AND the battery figures have data from one run.

Guide (step by step, archived): docs\SWIFT_SITL_SessionC_KCUB_Guide.html - open in a browser
Folder (since 10 Nov split): Simulink\ArduPilot_SITL  - see Simulink\README.md
One command (since 25 Nov): swift_run C  - see docs\swift_run_README.txt

Runway (FAA via AirNav/SkyVector): RWY 13 threshold 33.9744563, -81.0019926, 58.9 m AMSL;
125.55 deg true; 1528 m; displaced threshold 119 m. Model ground is flat.

Changes in v0.2
- sitl/swift_sitl_cub.m v0.2: BATTERY CHECK (P.batt, P.sitl.send_batt, swift_sitl_batt.param,
  out_batt logger in swift_6dof_sitl); leg-by-leg battery estimate through swift_battery_fcn;
  launch command loads swift_sitl_batt.param last; MAVProxy uses ../../swift_sitl_cub.waypoints.
- sitl/swift_plot_flight.m v0.3: Fig 1 gets a SOC + current panel; new Fig 6 Landing (height,
  airspeed, sink, pitch vs distance from touchdown; touchdown V/sink, rollout); new Fig 7 Battery on
  track (track coloured by current, altitude vs pack power); calls swift_battery_plot with the same
  tag so everything lands in data/runs/<tag>/; R.batt, R.touchdown_*, R.rollout_m.
- Checked in Octave (estimate run; plot logic on a synthetic flight with graphics stubbed). Not yet
  run in MATLAB.

Estimate (max payload, full pack, SITL-TEST prop): ~12.4 km, ~11.5 min arm-to-disarm,
SOC 100 -> 60 %, ~3990 mAh / 188 Wh, peak 59 A on the roll, avg ~21 A. BATT_LOW_MAH failsafe
(RTL at 8000 mAh used) has ~4000 mAh margin.

Flight (runway frame: x along RWY 13, y right of centreline), max 100 m AGL
 takeoff to 30 m -> climb-out 50 m -> climbing right turns to 100 m -> fast leg 27 m/s -> slow 22 m/s,
 2 left loiter circles (r 130 m) -> left spiral to 60 m -> 23 m/s, two left turns onto downwind 45 m ->
 DO_LAND_START -> right base, right final, 2 deg approach -> LAND at x = 200 m -> rollout, disarm +20 s.

Run order (manual; swift_run C does all of this)
 1 cd('<repo>\simulation\ArduPilot_SITL'); swift_setup; swift_sitl_preflight('<addr>')
 2 S = swift_sitl_cub('addr','<addr>');          (4x [ OK ] under BATTERY CHECK)
 3 open_system('swift_6dof_sitl'); Run
 4 Ubuntu: cp swift_sitl.param swift_sitl_cub.param swift_sitl_batt.param swift_sitl_vtail.param
           swift_sitl_cub.waypoints ~/
   sim_vehicle.py -v ArduPlane -f JSON:<addr> -N -w --custom-location=33.9744563,-81.0019926,58.9,125.5
     --add-param-file plane.parm, swift_sitl.param, swift_sitl_cub.param, swift_sitl_batt.param,
     swift_sitl_vtail.param --out=udp:<addr>:14550 --console --map
 5 MAVProxy: wp load ../../swift_sitl_cub.waypoints; param show BATT_MONITOR (4); mode FBWA; arm throttle; mode AUTO
 6 After "Disarming": Simulink Stop, then Ctrl+C Ubuntu; outC = out;
 7 R = swift_plot_flight(outC,'tag','sessionC_cub','wp',S.wp_file)
