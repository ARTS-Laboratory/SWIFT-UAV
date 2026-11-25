SWIFT UAV - V-tail in the Simulink models (14 Nov 2026)
========================================================
The models treated SWIFT as a conventional tail. These two files make both models
an inverted-V tail (apex up, twin booms): tail derivatives from the CAD geometry,
and two ruddervators that share their travel between pitch and yaw.

FILES (same two files, one copy in EACH folder - nothing is shared)
  Standalone\params\       swift_vtail.m   swift_vtail_add.m
  ArduPilot_SITL\params\   swift_vtail.m   swift_vtail_add.m

INSTALL (once per folder)
  cd('...\Simulink\Standalone');       swift_setup;   swift_vtail_add
  cd('...\Simulink\ArduPilot_SITL');   swift_setup;   swift_vtail_add
  Running it again changes nothing. Backups: swift_placeholder_<date>.bak and
  <model>_backup_<date>.slx next to the originals.

WHAT CHANGES
  swift_placeholder.m   one new line after section 4:  P = swift_vtail(P);
                        P.act.de_max and P.act.dr_max now equal P.tail.drv_max
  Controls              new "V-tail limit" block before delta_e / delta_r:
                          rv_L = de + dr,  rv_R = de - dr,  each held to +/- 25 deg
                        logged as out_rv = [left; right] (rad)
  SITL PWM decode       channel 2 = left ruddervator, channel 4 = right, un-mixed
                        to delta_e and delta_r. P.sitl.sgn keeps its meaning.
  sitl\swift_sitl_vtail.param   SERVO2_FUNCTION 79, SERVO4_FUNCTION 80, MIXING_GAIN 1
                        Load it AFTER your other parameter files.
  Nothing else in the model changes: it still flies on delta_e and delta_r, and
  out_delta, the plots and the flight video work as before.

TAIL GEOMETRY (swift_vtail.m section 1)
  From the STEP files in ARTS-Laboratory/SWIFT-UAV, Pelican structural_model:
  panel angle        25 deg from horizontal   v0.3.1 root mitre face, v0.3.0 Tail.stp  [CAD]
  panel length       0.4535 m (root to tip)   v0.3.1 tail parts                        [CAD]
  panel chord        0.2045 m                 v0.3.1                                   [CAD]
  ruddervator        0.296 x 0.0843 m         v0.3.1 Male/Female_Elevon                [CAD]
  boom spacing       0.765 m                  v0.2.4 MainAssembly.stp                  [CAD]
  tail arm           0.98 m                   v0.2.4 layout, CG assumed at wing 1/4 c  [EST->CAD]
  apex above CG      0.24 m                   0.19 above booms (CAD) + 0.05 estimate   [EST->CAD]
  ruddervator travel +/- 25 deg               not in the CAD                           [EST]
  There is no v0.3.1 assembly STEP in the repository, so the arm and boom spacing
  come from the v0.2.4 assembly. Confirm both in MainAssembly.iam.

DERIVATIVES (max payload)          before     V-tail
  CLde   elevator lift              0.360      0.321
  Cmde   elevator power            -1.000     -0.953
  CYdr   rudder side force          0.150      0.150
  Cndr   rudder power              -0.080     -0.056
  Cldr   roll from rudder           0.008     -0.017   sign change: rolls INTO the turn
  Cmq    pitch damping            -10.00     -13.97
  CYbeta side force                -0.250     -0.240
  Cnbeta weathercock stability      0.070      0.045
  Cnr    yaw damping               -0.090     -0.060
  Clbeta dihedral effect           -0.060     -0.042   the inverted V removes some
  Trim elevator moves from -2.4 to -2.5 deg. Cmalpha, CLq and the wing terms are
  untouched (Cmalpha needs the real CG).
  To change only the five control derivatives, set scope = 'controls' at the top of
  swift_vtail.m.

MIXING_GAIN
  ArduPlane sends  left = (yaw + pitch) x MIXING_GAIN,  right = (-yaw + pitch) x MIXING_GAIN.
  The file sets 1.0, so full elevator alone still gives 25 deg as it did before, and
  elevator + rudder together run a ruddervator into its stop. ArduPlane's default
  0.5 never hits the stop but halves both throws. Change P.tail.mix_gain in
  swift_vtail.m, then:  swift_vtail_add params

CHECK AFTER INSTALL (SITL, MANUAL mode)
  Stick back and right rudder should move delta_e and delta_r the same way as before.
  One axis wrong     -> swap 79 and 80 in swift_sitl_vtail.param
  Both axes wrong    -> flip that sign in P.sitl.sgn (swift_sitl_params.m)

REAL AIRCRAFT (Pixhawk 6X)
  Same three parameters. One ruddervator servo is mirrored, so expect one
  SERVOn_REVERSED = 1. For an inverted V the 79/80 assignment is often the opposite
  of an upright V. Do not mix in the transmitter.

NOT TESTED IN MATLAB
  Both files pass a MATLAB syntax check and the numbers were cross-checked
  separately, but neither has been run in MATLAB or against the .slx files. The
  backups are there for that reason. If swift_vtail_add stops with "Expected block
  not found", send me the message.

KNOWN
  - All tail aero is hand theory (NACA Report 823), not AVL/XFLR5.
  - DataSheet.xlsx in the repository is out of date (boom spacing 0.62 m, tapered
    NACA 2412 wing).
  - The ruddervator limit is hard; servo rate and lag are not modelled.
