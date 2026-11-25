SWIFT UAV - swift_run: one-command SITL sessions (v0.1, 25 Nov 2026)
=====================================================================
One MATLAB command replaces the whole RUN ORDER of the session READMEs - the
MATLAB steps, the Ubuntu cp + sim_vehicle.py block, the MAVProxy typing, the
stop sequence and the post-flight calls.

    cd('<repo>\simulation\ArduPilot_SITL'); swift_setup
    swift_run E

That's all. The Command Window stays free while it flies, and autofly's
progress is echoed there every 2 s.

FILES  (ArduPilot_SITL\sitl\..., nothing in Standalone is touched)
  run\swift_run.m                       the runner
  run\swift_autofly.py                  runs next to ArduPlane: mission upload, parameter check, arm, AUTO
  run\sessions\swift_session_C/D/E.m    session settings (planner, .param files, analysis)
  run\sessions\swift_session_template.m copy this for Session F, G ...
  missions\swift_sitl_cub*.m            planners; they write their .waypoints/.param next to themselves
  ardupilot\swift_sitl*.param           shared ArduPlane parameters (base, battery, V-tail)
  ..\docs\swift_run_README.txt          this file
  A session's C.params names are looked up in ardupilot\ first, then missions\.
  Written each run (safe to delete): ArduPilot_SITL\cache\swift_run\
      swift_run_sitl.sh  swift_run_close.sh  autofly.log  done.json

PLATFORMS (v0.2, 25 Nov 2026)
  Windows  ArduPilot inside WSL; opens a console window; address found from WSL.
  macOS    ArduPilot built natively; opens Terminal.app; address 127.0.0.1.
  Linux    ArduPilot built natively; opens gnome-terminal / x-terminal-emulator / konsole /
           xterm, or with no display runs in the background (cache/swift_run/sitl_console.log)
           with MAVProxy non-interactive; address 127.0.0.1.
  The scripts unset LD_LIBRARY_PATH first: MATLAB's own libraries otherwise break python and
  sim_vehicle.py when they are started from MATLAB on macOS/Linux.
  Wherever these notes say "Ubuntu window", read "the ArduPlane window" on macOS/Linux.

WHAT  swift_run E  DOES
  MATLAB   1  swift_setup + swift_sitl_preflight (stops on a red FAIL line)
           2  swift_sitl_cub_e writes the .waypoints and .param files
           3  opens an Ubuntu window: copies the files to ~/swift_run and starts
              sim_vehicle.py -v ArduPlane -f JSON:<addr> -N -w at the runway,
              with plane.parm + the session's .param files in order and two
              outputs: udp <addr>:14550 (Mission Planner) and 127.0.0.1:14551
           4  presses Run on swift_6dof_sitl
  Ubuntu   5  swift_autofly.py: waits for ArduPlane, uploads the mission, reads
              back EVERY parameter in the .param files (refuses to fly if one
              is wrong), waits for GPS 3D fix + EKF position, FBWA -> arm -> AUTO,
              then follows the flight (mission item, ArduPlane messages, a
              progress line every 30 s)
  MATLAB   6  after touchdown and auto-disarm: Simulink Stop, then ArduPlane
              closed, newest .BIN log copied to data\runs\<tag>\<tag>.BIN
           7  'out' kept as outE, then the session's analysis: flight plots,
              range check, V-tail check, video. One failing step does not stop
              the others.
  Results: workspace variable swiftRun (plan, autofly result, analysis outputs),
  files in data\runs\sessionE_cub\.

COMMANDS
  swift_run list                  sessions you can fly
  swift_run E                     fly Session E
  swift_run status                where the flight is (last autofly lines)
  swift_run rtl                   ArduPlane RTL -> lands (RTL_AUTOLAND = 2)
  swift_run stop                  stop now: Simulink first, then ArduPlane
                                  (the partial flight is still analysed)
  swift_run analyze E             re-run E's analysis on 'out' in the workspace
  swift_run('E', 'dry', true)     plan + write the Ubuntu script, launch nothing

OPTIONS  swift_run('E', name, value, ...)
  'addr'      ''      Windows address seen from WSL. Blank = automatic (mirrored
                      networking -> 127.0.0.1, otherwise the WSL default route).
                      Remembered after the first run.
  'plan'      {}      planner options, e.g.  {'land_mAh', 4000}  or  {'alt', 120}
  'skip'      {}      analysis steps to skip, e.g.  {'video'}
  'analyze'   true    false = fly only
  'map'       true    false = no MAVProxy map/console windows
  'auto'      true    false = launch only; type the MAVProxy steps yourself
  'wipe'      true    sim_vehicle.py -w (fresh parameters every run)
  'force'     false   launch even if preflight reports a problem
  'ardupilot' '~/ardupilot'    'distro' ''  (WSL distribution, blank = default)

NEW SESSION
  copy sessions\swift_session_template.m -> sessions\swift_session_F.m, rename
  the function inside to swift_session_F, fill in the fields, then
  swift_run('F', 'dry', true) to check and  swift_run F  to fly. The template's
  example is a Session E variant at 120 m with a 4000 mAh reserve - no new
  planner needed for variants, only plan_args.

FIRST RUN CHECKLIST
  - pymavlink in Ubuntu: it comes with MAVProxy, which sim_vehicle.py already
    uses. If the Ubuntu window prints "pymavlink is missing":
        pip3 install --user pymavlink MAVProxy
  - Mission Planner can still connect on UDP 14550 to watch.
  - Your Session C/D/E scripts and READMEs are unchanged; the manual RUN
    ORDER still works exactly as before.

TESTED (25 Nov 2026, cloud workspace)
  - swift_autofly.py against a stand-in ArduPlane (pymavlink): mission upload
    (17 items, coordinates checked), 42-parameter readback, prearm wait, arm,
    AUTO, item tracking, disarm -> done file. Also: one wrong parameter
    (refuses to fly), RTL sent from MATLAB, a parameter missing from the build.
  - The generated Ubuntu script with a stand-in sim_vehicle.py: files copied,
    every sim_vehicle.py argument correct, autofly flew it end to end.
  - swift_run.m in Octave 8.4 with Simulink stubbed: dry run, session list,
    bad session/option errors, the timer's landed -> stop -> close -> analysis
    path, the Simulink-stopped-early path, an analysis step that errors.
  - The close script kills only the processes it started and copies the log.
  - v0.2 on Linux (Octave): automatic 127.0.0.1, headless background launch of the
    stand-in ArduPlane, full autofly flight, close script with log copy.
  NOT YET RUN in MATLAB/Simulink or against real ArduPlane; the macOS Terminal launch
  is untested. The one
  thing only a real run can show: that MATLAB's timer fires while
  swift_6dof_sitl is running. If the flight lands and nothing happens in
  MATLAB within ~5 s, press Stop in Simulink - swift_run then closes
  ArduPlane and runs the analysis on its own - and tell me.
