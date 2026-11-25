%% SWIFT_PLACEHOLDER  Every number swift_6dof.slx needs, in one place.
%
%  WHY THIS FILE EXISTS
%    Not one block dialog in swift_6dof.slx contains a number. Every field
%    is a P. expression pointing back into this script. When better numbers
%    arrive you change them HERE and no block changes. After editing any
%    aerodynamic derivative at the command line, rebuild the matrix:
%
%        P = swift_aero_matrix(P);
%
%    The model calls this file itself. Its InitFcn callback runs it whenever
%    P is missing from the base workspace, so the model works from a cold
%    MATLAB session.
%
%  WHAT THESE NUMBERS ARE (v0.5)
%    Real SWIFT values from "UAV Specs.xlsx" wherever the spreadsheet has
%    them, and flagged estimates everywhere else. Every value carries a tag:
%      [SHEET]      taken from UAV Specs.xlsx (24 SEP 2026)
%      [SPEC]       manufacturer datasheet
%      [STD]        physical constant / published standard
%      [EST]        engineering estimate - basis given in the comment
%      [EST->CAD]   estimate; REPLACE with the CAD mass-properties export
%      [EST->AERO]  estimate; REPLACE via AVL/XFLR5, wind tunnel, or flight ID
%      [EST->PROP]  estimate; REPLACE with motor/prop bench-test data
%      [CHECK]      spreadsheet value that looks inconsistent - confirm it
%
%  PAYLOAD SWITCH
%    Default is max payload (4.5 kg). For an empty-weight run:
%        P_with_payload = false;  swift_placeholder
%    Mass, weight, stall speed, cruise speed and trim all follow the switch.
%
%  REQUIRES
%    swift_aero_matrix.m in the same folder - called in section 10.
%
%  Ryan Bilock, ARTS Lab | v0.7
%    v0.7  trim now includes the aileron and rudder that hold the wings level
%          against the motor's reaction torque (was zero, so the standalone
%          run rolled left from t = 0). Side-force sign fixed in
%          swift_aero_matrix v0.3.
%    v0.6  real motor data: MAD X7215 IPE 220KV on 12S with a 21x10 prop and
%          AMPX 120A ESC (MAD Components bench table, 11 NOV 2026). Motor
%          rating, RPM, static thrust vs throttle, reaction torque and the
%          motor+ESC efficiency now come from that table. The old sheet
%          values (469.1 W, 12.19 A, 48.02 V) were its 45%-throttle row.
%          Trim throttle is now read off the thrust table.
%    v0.5  battery model for the Battery subsystem (swift_battery_add):
%          pack resistance, LiPo open-circuit-voltage curve, start charge,
%          low/critical cell voltages, P.batt.prm / P.batt.ocv / P.batt.x0;
%          cruise battery draw and endurance printed in the banner.
%    v0.4  replaced the generic 4 kg airframe with UAV Specs.xlsx values
%          (geometry, masses, battery, motor, prop, ESC, power board,
%          structural limits); component mass budget with payload switch;
%          wing lift from NACA 4415 finite-wing theory; trim computed from
%          the parameters instead of typed in; thrust-power feasibility
%          check in the banner.
%    v0.3  added the four scalar P.init.delta_* fields the Controls
%          subsystem reads; end-of-file check against the model's variables.
% -------------------------------------------------------------------------

if ~exist('P_with_payload', 'var'), P_with_payload = true; end

P = struct();
P.meta.vehicle = 'SWIFT UAV (Pelican V0.3)';
P.meta.version = 'v0.7 - UAV Specs.xlsx + MAD X7215 motor data + flagged estimates';
P.meta.with_payload = logical(P_with_payload);
clear P_with_payload

%% 1 | Constants ------------------------------------------------------------
P.const.g    = 9.80665;      % m/s^2                                        [STD]
P.const.rho0 = 1.225;        % kg/m^3   sea-level ISA, for hand checks only [STD]

%% 2 | Mass properties ------------------------------------------------------
%  Read by the 6DOF block (step 7) and the weight Constant (step 19).
%  All-up mass is the SUM of this budget. Overwrite a line as real weights
%  come in and everything downstream (weight, stall, trim) updates.
P.mass.payload_max = 4.5;    % kg   payload capacity                       [SHEET]
P.mass.budget = { ...
  % component                                   kg      source
    'Wing (PLA)',                               7.400;  % [SHEET]
    'Tail (NACA 0012)',                         1.600;  % [SHEET]
    'Fuselage (PLA + 20x20 aluminum)',          1.500;  % [EST->CAD] sheet says "?"
    'Landing gear (230 mm)',                    0.400;  % [EST->CAD] sheet says "?"
    'Battery (12S, 10 Ah)',                     2.600;  % [SHEET]
    'Motor',                                    0.535;  % [SHEET]
    'Propeller 21x10',                          0.080;  % [SHEET] marked "unsure"
    'ESC (120 A)',                              0.014;  % [SHEET][CHECK] light for 120 A
    'Power distribution board',                 0.005;  % [SHEET]
    'Flight controller (Pixhawk 6X)',           0.090;  % [SHEET]
    'M10 GPS',                                  0.035;  % [EST] weigh to retire
    'Telemetry radio + antenna',                0.045;  % [EST] weigh to retire
    'Servos: 4 x DS215MG @ 20 g',               0.080;  % [SPEC] count of 4 assumed
    'Wiring, connectors, mounts',               0.300;  % [EST] 12S leads are heavy
    'Payload mount',                            2.200;  % [SHEET]
    'Payload',       P.mass.payload_max * P.meta.with_payload;  % [SHEET] switchable
  };
P.mass.m   = sum([P.mass.budget{:,2}]);   % kg   all-up mass (computed)

%  Inertia: strip build-up, not CAD. Ixx = wing as a uniform 7.4 kg bar over
%  2.6 m (m*b^2/12 = 4.17) plus fuselage and tail; Iyy = fuselage/payload
%  over 0.856 m plus the 1.6 kg tail at an ASSUMED 1.0 m arm; Izz ~ Ixx+Iyy.
%  Payload in/out changes these by under 5%, so they are held fixed.
P.mass.Ixx = 4.3;            % kg*m^2   roll                          [EST->CAD]
P.mass.Iyy = 2.4;            % kg*m^2   pitch  (tail arm assumed)     [EST->CAD]
P.mass.Izz = 6.7;            % kg*m^2   yaw                           [EST->CAD]
P.mass.Ixz = 0.10;           % kg*m^2   roll-yaw cross term           [EST->CAD]
P.mass.I   = [ P.mass.Ixx  0           -P.mass.Ixz;
               0           P.mass.Iyy   0;
              -P.mass.Ixz  0            P.mass.Izz ];

%  Aerodynamic reference point. Both the CG and CP inputs on the aero block
%  (step 17) read this, which makes its transfer-moment term vanish - the
%  moment coefficients already describe moments about the CG. When the CAD
%  export gives you a real offset, this is where it goes in.
P.mass.r_cg = [0; 0; 0];     % m                                      [EST->CAD]

%% 3 | Geometry -------------------------------------------------------------
%  Note: span x chord = 2.6 x 0.33 = 0.858 m^2, but the sheet area is 0.81.
%  The gap (0.048 m^2) equals the 150 mm fuselage width x 0.33 m chord, so
%  0.81 is the exposed area. AR 8.4 on the sheet uses 0.81, so it is kept.
P.geom.b = 2.60;             % m    wingspan         (block: Reference span)   [SHEET]
P.geom.S = 0.81;             % m^2  wing area        (block: Reference area)   [SHEET]
P.geom.c = 0.33;             % m    chord, rectangular = MAC (Reference length) [SHEET]
P.geom.airfoil_wing = 'NACA 4415';                                          % [SHEET]
P.geom.airfoil_tail = 'NACA 0012';                                          % [SHEET]
P.geom.fus_length   = 0.85625;   % m                                        [SHEET]
P.geom.fus_width    = 0.150;     % m                                        [SHEET]
P.geom.fus_height   = 0.153038;  % m                                        [SHEET]
P.geom.gear_height  = 0.230;     % m                                        [SHEET]
P.geom.wing_mount   = 'Tongue and groove';                                  % [SHEET]
P.geom.motor_mount  = 'Single-screw motor mount';                           % [SHEET]
P.geom.payload_mount = 'Inside fuselage';                                   % [SHEET]

%% 4 | Aerodynamic derivatives ----------------------------------------------
%  swift_aero_matrix.m packs these into the single 5x9 matrix the model
%  multiplies by. Names here must match the names it expects exactly.
%
%  Lift comes from the NACA 4415 section + finite-wing correction:
%    a0 = 6.0 /rad (0.105 /deg, typical at Re ~ 5e5), alpha_L0 = -4.0 deg
%    CLalpha = a0 / (1 + a0/(pi*e*AR)) = 4.67 /rad at AR 8.35, e 0.80
%    CL0     = CLalpha * 4.0 deg       = 0.33
%  Everything else is still a generic small-UAV value until AVL/XFLR5.
P.aero.CL0     =  0.33;   P.aero.CLalpha =  4.67;   % per rad          [EST->AERO] 4415 basis
P.aero.CLq     =  7.00;   P.aero.CLde    =  0.36;   %                  [EST->AERO]
P.aero.CLmax   =  1.30;   P.aero.alpha_stall = deg2rad(12);   % 4415 wing [EST->AERO]

P.aero.CD0     =  0.035;  P.aero.e_osw   =  0.80;   % parasite, Oswald [EST->AERO]

P.aero.CYbeta  = -0.25;   P.aero.CYdr    =  0.15;   %                  [EST->AERO]
P.aero.CYp     =  0.00;   P.aero.CYr     =  0.00;   %                  [EST->AERO]

P.aero.Clbeta  = -0.06;   P.aero.Clp     = -0.45;   % dihedral, roll damping [EST->AERO]
P.aero.Clr     =  0.08;   P.aero.Clda    =  0.18;   P.aero.Cldr = 0.008;     % [EST->AERO]

P.aero.Cm0     =  0.02;   P.aero.Cmalpha = -0.60;   % pitch stiffness  [EST->AERO]
P.aero.Cmq     = -10.0;   P.aero.Cmde    = -1.00;   % pitch damping    [EST->AERO]

P.aero.Cnbeta  =  0.07;   P.aero.Cnp     = -0.03;   % weathercock      [EST->AERO]
P.aero.Cnr     = -0.09;   P.aero.Cnda    = -0.01;   P.aero.Cndr = -0.08;     % [EST->AERO]

%  V-tail: overwrite the tail-driven derivatives from the CAD geometry and
%  define the ruddervator mixing (swift_vtail.m).
P = swift_vtail(P);

%% 5 | Propulsion, battery, power -------------------------------------------
% -- Battery (12S lithium polymer: 44.4 V / 3.7 V per cell = 12 cells) -----
P.batt.V_nom    = 44.4;      % V                                            [SHEET]
P.batt.cells    = round(P.batt.V_nom / 3.7);   % -   = 12S (derived)
P.batt.V_full   = 4.2 * P.batt.cells;          % V   = 50.4 V full charge (derived)
P.batt.Ah       = 10.0;      % A*h                                          [SHEET]
P.batt.E_Wh     = P.batt.V_nom * P.batt.Ah;    % Wh  = 444 Wh, matches sheet
P.batt.I_cont   = 1200;      % A    [SHEET][CHECK] 1200 A is 120C - likely 120 A
P.batt.usable   = 0.80;      % -    fly to a 20% reserve                    [EST]
P.batt.E_usable = P.batt.E_Wh * P.batt.usable;

%  Battery model (read by the Battery subsystem, swift_battery_add.m).
%  Pack terminal voltage = cells * OCV(SOC) - I * R_pack. OCV is the resting
%  (open-circuit) cell voltage against state of charge; R_pack makes the
%  voltage sag under load, the way a real pack does on a full-throttle climb.
P.batt.SOC0       = 1.00;    % -    charge at the start of the run (1 = full) [OPS]
P.batt.R_cell     = 0.0035;  % ohm  per cell, 10 Ah high-C LiPo             [EST] measure to retire
P.batt.R_wire     = 0.008;   % ohm  leads, XT90/AS150 connector, ESC input  [EST]
P.batt.R_pack     = P.batt.cells * P.batt.R_cell + P.batt.R_wire;   % ohm = 0.050
P.batt.Vcell_low  = 3.50;    % V    per cell, loaded: "low" (ArduPilot BATT_LOW_VOLT) [OPS]
P.batt.Vcell_crit = 3.30;    % V    per cell, loaded: "critical" (BATT_CRT_VOLT)       [OPS]
P.batt.SOC_res    = 1 - P.batt.usable;   % -  reserve line, 0.20
%  Resting LiPo cell voltage vs state of charge, 0 to 100 % in 5 % steps.
%  Generic hobby-LiPo curve [EST]. To retire: discharge one pack at a low,
%  steady current, log voltage against mAh, and paste the curve here.
P.batt.ocv = [ 0:0.05:1 ;
   3.27 3.61 3.69 3.71 3.73 3.75 3.77 3.79 3.80 3.82 3.84 ...
   3.85 3.87 3.91 3.95 3.98 4.02 4.08 4.11 4.15 4.20 ];          % [EST]

% -- Motor: MAD X7215 IPE 220KV on 12S, 21x10 prop ---------------------------
%  From the MAD Components bench table (motor + AMPX 120A ESC, 12S supply).
%  The sheet's 469.1 W / 12.19 A / 48.02 V were the 45%-throttle row.
P.motor.model    = 'MAD X7215 IPE 220KV';
P.motor.Kv       = 220;       % rpm/V                                       [SPEC]
P.motor.V_max    = 50.4;      % V    rated for 12S (bench pack)             [SPEC]
P.motor.I_max    = 107.96;    % A    100% throttle                          [SPEC]
P.motor.P_max    = 5093.6;    % W    electrical input at 100% throttle      [SPEC]
P.motor.P_out_max = 3634.6;   % W    shaft output at 100% throttle          [SPEC]
P.motor.RPM_max  = 7782;      % rpm  21x10 prop, 100% throttle              [SPEC]

%  Bench table, 12S, 21x10: throttle (-), thrust (gf), torque (N*m), rpm,
%  input power (W). Rows 30% to 100% in 5% steps.                       [SPEC]
P.motor.bench = [ ...
  0.30  1866  0.490  2884   206.8;   0.35  2452  0.646  3291   298.5;
  0.40  3101  0.831  3690   414.5;   0.45  3995  1.080  4148   585.4;
  0.50  5005  1.370  4610   806.9;   0.55  5899  1.618  5025  1050.3;
  0.60  6943  1.927  5404  1335.9;   0.65  7799  2.157  5762  1617.8;
  0.70  8832  2.460  6085  1960.9;   0.75  9949  2.787  6417  2352.6;
  0.80 11109  3.116  6774  2814.9;   0.85 12278  3.484  7088  3349.9;
  0.90 13193  3.810  7324  3783.5;   0.95 13830  4.126  7596  4367.2;
  1.00 14795  4.460  7782  5093.6 ];

% -- ESC -------------------------------------------------------------------
P.esc.model     = 'AMPX 120A (5-14S)';
P.esc.V_max     = 58.8;      % V    14S x 4.2 V                             [SPEC]
P.esc.I_max     = 120;       % A                                            [SPEC]

% -- Power distribution board (servo / avionics rail) -----------------------
P.elec.pdb.V_in  = 44.4;     % V                                            [SHEET]
P.elec.pdb.V_out = 8.0;      % V    within DS215MG 6.0-8.4 V range          [SHEET]
P.elec.pdb.I_max = 9.0;      % A                                            [SHEET]
P.elec.pdb.eta   = 0.90;     % -                                            [SHEET]
P.elec.fc.V_op   = 5.0;      % V    flight controller                       [SHEET]
P.elec.fc.P      = 2.0;      % W    flight controller                       [SHEET]
P.elec.P_hotel   = P.elec.fc.P + 6.0;  % W  FC [SHEET] + GPS, radio, servos [EST]

% -- Efficiency chain (read by the Battery subsystem) ------------------------
P.prop.eta_prop = 0.65;      % -    ideal (momentum) power / shaft power    [EST->PROP]
P.prop.eta_mot  = 0.82;      % -    motor + ESC together, bench 80-82% at 45-60%  [SPEC]
P.prop.eta_esc  = 1.00;      % -    folded into eta_mot: bench input power is
                             %      measured ahead of the ESC                    [SPEC]

% -- Propeller ---------------------------------------------------------------
P.prop.D     = 0.5334;       % m    21 in                                   [SHEET]
P.prop.pitch = 0.254;        % m    10 in                                   [SHEET]
P.prop.A_disk = pi * P.prop.D^2 / 4;   % m^2  0.2235, prop disk (derived)

%  Thrust table. Rows: throttle 0..1. Columns: airspeed. Values: thrust, N.
%  Law: T = T_static(throttle) * max(0, 1 - V/V_pitch)
%    T_static: MAD bench static thrust at each throttle step (30-100%),
%          straight line to 0 N at 0% throttle.                          [SPEC]
%    V_pitch: full-throttle pitch speed = RPM_max * pitch / 60 = 32.9 m/s.
%          The straight-line drop with airspeed is still an estimate; a
%          wind-tunnel or prop-performance curve would replace it.     [EST->PROP]
%  The airspeed breakpoints run to 40 m/s (well past cruise) so the table
%  block never extrapolates a negative thrust.
P.prop.throttle_bp  = [0, P.motor.bench(:,1)'];            % table row breakpoints
P.prop.T_static     = [0, P.motor.bench(:,2)' * P.const.g / 1000];   % N, per row
P.prop.T_static_max = P.prop.T_static(end);                % N  145.1   [SPEC]
P.prop.V_pitch      = P.motor.RPM_max * P.prop.pitch / 60; % m/s 32.9   [EST->PROP]
P.prop.V_bp         = 0:5:40;                              % table column breakpoints, m/s
P.prop.thrust_map   = P.prop.T_static' * max(0, 1 - P.prop.V_bp / P.prop.V_pitch);

P.prop.kQ       = 0.029;     % N*m of reaction torque per N of thrust, bench 0.027-0.031 [SPEC]
P.prop.spin_dir = +1;        % +1 = clockwise seen from behind              [TBD]

% -- Battery subsystem inputs (derived - edit the fields above, not these) ---
%  P.batt.prm order is fixed; the Battery model MATLAB Function reads it by
%  index. If you add a field, add it at the END and update that function.
P.batt.prm = [ P.batt.cells;        %  1  -     cells in series
               P.batt.Ah;           %  2  A*h   capacity
               P.batt.R_pack;       %  3  ohm   pack + wiring resistance
               P.prop.A_disk;       %  4  m^2   prop disk area
               P.prop.eta_prop;     %  5  -     ideal power / shaft power
               P.prop.eta_mot;      %  6  -     motor
               P.prop.eta_esc;      %  7  -     ESC
               P.elec.P_hotel;      %  8  W     avionics + servos
               P.elec.pdb.eta;      %  9  -     power board feeding the hotel load
               P.batt.Vcell_low;    % 10  V     low cell voltage
               P.batt.Vcell_crit;   % 11  V     critical cell voltage
               P.batt.SOC_res;      % 12  -     reserve state of charge
               P.motor.P_max ];     % 13  W     motor electrical rating
P.batt.x0  = [P.batt.SOC0; 0; 0];   % Integrator start: [SOC; mAh used; Wh used]

%% 6 | Environment ----------------------------------------------------------
P.env.wind_ned     = [0; 0; 0];   % m/s  steady wind in NED - a test knob
P.env.W20_light    = 7.72;        % m/s  (15 kt) Dryden intensity           [STD]
P.env.W20_moderate = 15.43;       % m/s  (30 kt)                            [STD]
P.env.W20_severe   = 23.15;       % m/s  (45 kt)                            [STD]

%% 7 | Structural limits (not read by the model; for load checks) -----------
P.struct.n_max        = 3.0;      % g    design load factor at cruise       [SHEET]
P.struct.wing_max_load = 588;     % N    wing max load at n = 3             [SHEET]
                                  %      (588/3 = 196 N -> a 20.0 kg design weight)
P.struct.n_landing    = 2.0;      % g    max landing load, sheet "1-2 g"    [SHEET]
P.struct.material     = 'PLA + aluminum 20x20 extrusion';                % [SHEET]

%% 8 | Initial conditions - trimmed from the parameters above ---------------
%  Cruise is set at 1.3 x the 1-g stall speed, then the level-flight
%  alpha and elevator are solved from the lift and pitch equations:
%     CLalpha*alpha + CLde*de = CL_req - CL0
%     Cmalpha*alpha + Cmde*de = -Cm0
%  Change mass or aero and the trim follows - no hand-typed trim values.
swift_W   = P.mass.m * P.const.g;
swift_Vs  = sqrt(2*swift_W / (P.const.rho0 * P.geom.S * P.aero.CLmax));
P.init.h0 = 100;                                % m
P.init.V0 = round(1.3 * swift_Vs);              % m/s  (23 loaded, 21 empty)

swift_qS  = 0.5 * P.const.rho0 * P.init.V0^2 * P.geom.S;
swift_CL  = swift_W / swift_qS;
swift_ad  = [P.aero.CLalpha P.aero.CLde; P.aero.Cmalpha P.aero.Cmde] \ ...
            [swift_CL - P.aero.CL0; -P.aero.Cm0];
swift_D   = (P.aero.CD0 + swift_CL^2 / (pi*P.aero.e_osw*P.geom.b^2/P.geom.S)) * swift_qS;
swift_Tfull = P.prop.T_static_max * max(0, 1 - P.init.V0/P.prop.V_pitch);

P.trim.alpha   = swift_ad(1);                   % rad
P.trim.de      = swift_ad(2);                   % rad
P.trim.drag    = swift_D;                       % N    thrust needed for level flight
P.trim.T_avail = swift_Tfull;                   % N    full-throttle thrust at V0
P.trim.P_req   = swift_D * P.init.V0;           % W    thrust power needed
P.trim.P_avail = P.motor.P_out_max * P.prop.eta_prop;  % W  thrust power available
P.trim.feasible = swift_Tfull >= swift_D;
if P.trim.feasible
    %  Throttle that gives exactly the drag at V0, read off the thrust table.
    swift_Trow = P.prop.T_static * max(0, 1 - P.init.V0/P.prop.V_pitch);
    swift_dT   = interp1(swift_Trow, P.prop.throttle_bp, swift_D, 'linear');
else
    swift_dT = 1.0;          % cannot hold level flight: pinned at full throttle
end

P.init.uvw0   = P.init.V0 * [cos(P.trim.alpha); 0; sin(P.trim.alpha)];  % m/s body
P.init.euler0 = [0; P.trim.alpha; 0];          % rad  level flight: theta = alpha
P.init.om0    = [0; 0; 0];                     % rad/s
P.init.pos0   = [0; 0; -P.init.h0];            % m    NED, so pD = -h0

%  Lateral trim. The motor's reaction torque (-spin_dir*kQ*T about body x,
%  T = drag in level flight) rolls the aircraft. Aileron and rudder cancel it
%  with zero sideslip. The moment coefficients are in wind axes, so the torque
%  is resolved there first: [cos(alpha); 0; -sin(alpha)] * torque.
%     Clda*da + Cldr*dr = -L_prop*cos(alpha) / (qbar*S*b)
%     Cnda*da + Cndr*dr = +L_prop*sin(alpha) / (qbar*S*b)
swift_Lp  = -P.prop.spin_dir * P.prop.kQ * swift_D;              % N*m, body x
swift_lat = [P.aero.Clda P.aero.Cldr; P.aero.Cnda P.aero.Cndr] \ ...
            ([-cos(P.trim.alpha); sin(P.trim.alpha)] * swift_Lp / (swift_qS * P.geom.b));
P.trim.da = swift_lat(1);                       % rad
P.trim.dr = swift_lat(2);                       % rad

%  Trim control positions: [aileron; elevator; rudder; throttle].
P.init.delta0 = [P.trim.da; P.trim.de; P.trim.dr; swift_dT];

%  The four Constants inside the Controls subsystem (step 13) read these
%  scalars, one per channel. Deriving them here keeps ONE place to change.
P.init.delta_a = P.init.delta0(1);   % rad   aileron
P.init.delta_e = P.init.delta0(2);   % rad   elevator
P.init.delta_r = P.init.delta0(3);   % rad   rudder
P.init.delta_T = P.init.delta0(4);   % -     throttle, 0 to 1

%  Cruise battery check - the same equations the Battery subsystem runs.
%  Momentum theory gives the ideal power for thrust T at airspeed V:
%     v_i = -V/2 + sqrt(V^2/4 + T/(2*rho*A)),   P_ideal = T*(V + v_i)
%  then the efficiency chain and the hotel load give the battery's watts.
swift_vi  = -P.init.V0/2 + sqrt(P.init.V0^2/4 + P.trim.drag/(2*P.const.rho0*P.prop.A_disk));
P.trim.P_shaft = P.trim.drag * (P.init.V0 + swift_vi) / P.prop.eta_prop;      % W
P.trim.P_batt  = P.trim.P_shaft / (P.prop.eta_mot*P.prop.eta_esc) + ...
                 P.elec.P_hotel / P.elec.pdb.eta;                             % W
P.trim.I_batt  = P.trim.P_batt / P.batt.V_nom;                                % A  (approx.)
P.trim.endurance_min = 60 * P.batt.E_usable / P.trim.P_batt;                  % min to reserve

%  Control-surface travel limits. The standalone model does not read these -
%  its surfaces are ideal. The ArduPilot SITL bridge does: it turns servo
%  PWM (1000-2000 us, 1500 = centre) into deflection with
%      delta = P.act.d?_max * (pwm - 1500) / 500
%  so these set how far full stick moves each surface. Put each surface at
%  full stick, measure the angle, and replace the estimate.
P.act.da_max = deg2rad(25);          % rad   aileron, +/- from centre  [EST] measure to retire
P.act.de_max = P.tail.drv_max;          % rad   elevator                  [EST] measure to retire
P.act.dr_max = P.tail.drv_max;          % rad   rudder                    [EST] measure to retire

if abs(P.trim.de) > P.act.de_max
    warning('swift_placeholder:trimTravel', ...
        'Trim elevator %.1f deg is beyond the %.0f deg travel limit - the aircraft cannot trim.', ...
        rad2deg(P.trim.de), rad2deg(P.act.de_max));
end

%% 9 | Solver and autopilot -------------------------------------------------
P.sim.dt     = 0.005;        % s    not read by the model; for your own code
P.sim.tfinal = 60;           % s    read by Model Settings -> Stop time

%  Autopilot gains - starting values. Nothing reads these until stage 5.
%  NOTE: tuned for the old 4 kg placeholder; retune for the ~21 kg SWIFT.
P.gnc.dt        = 0.02;               % s   50 Hz outer-loop rate
P.gnc.KP_V      = 0.060;   P.gnc.KI_V = 0.020;
P.gnc.dT_min    = 0.00;    P.gnc.dT_max = 1.00;
P.gnc.KP_h      = 0.010;   P.gnc.KI_h = 0.0012;
P.gnc.theta_max = deg2rad(15);
P.gnc.KP_theta  = 1.00;    P.gnc.KD_q = 0.25;
P.gnc.KP_psi    = 0.80;    P.gnc.phi_max = deg2rad(35);
P.gnc.KP_phi    = 0.90;    P.gnc.KD_p = 0.08;
P.gnc.KR_r      = 0.30;    P.gnc.tau_wo = 4.0;

%% 10 | Derived - never edit these by hand ----------------------------------
P = swift_aero_matrix(P);

%% 11 | Check against what the model actually reads -------------------------
%  Every workspace variable swift_6dof.slx refers to, read straight out of
%  the block dialogs. If you rename a field, this tells you before Simulink.
swift_required = { ...
    'const.g'            % weight Constant, step 19
    'env.wind_ned'       % wind Constant, step 9
    'geom.S'             % aero block reference area, step 17
    'geom.b'             % aero block span; nondim rate gains, steps 12 & 17
    'geom.c'             % aero block chord; pitch rate gain, steps 12 & 17
    'mass.m'             % 6DOF mass; weight Constant, steps 7 & 19
    'mass.I'             % 6DOF inertia matrix, step 7
    'mass.r_cg'          % aero block CG and CP Constants, step 17
    'init.pos0'          % 6DOF initial position, step 7
    'init.uvw0'          % 6DOF initial body velocity, step 7
    'init.euler0'        % 6DOF initial attitude, step 7
    'init.om0'           % 6DOF initial body rates, step 7
    'init.delta_a'       % Controls, step 13
    'init.delta_e'       % Controls, step 13
    'init.delta_r'       % Controls, step 13
    'init.delta_T'       % Controls, step 13
    'aero.A'             % the matrix Gain, step 15        (derived)
    'aero.CD0'           % drag Constant, step 16
    'aero.k_ind'         % induced-drag Gain, step 16       (derived)
    'prop.thrust_map'    % thrust table data, step 18
    'prop.throttle_bp'   % thrust table row breakpoints, step 18
    'prop.V_bp'          % thrust table column breakpoints, step 18
    'prop.kQ'            % reaction-torque Gain, step 18
    'prop.spin_dir'      % reaction-torque Gain, step 18
    'sim.tfinal'         % Model Settings -> Stop time, step 5
    'batt.prm'           % Battery subsystem, model-parameter Constant (swift_battery_add)
    'batt.ocv'           % Battery subsystem, OCV-curve Constant
    'batt.x0'            % Battery subsystem, Integrator initial condition
    };

swift_missing = {};
for swift_i = 1:numel(swift_required)
    swift_parts = strsplit(swift_required{swift_i}, '.');
    if ~isfield(P, swift_parts{1}) || ~isfield(P.(swift_parts{1}), swift_parts{2})
        swift_missing{end+1} = ['P.' swift_required{swift_i}]; %#ok<SAGROW>
    end
end

%% ---- banner --------------------------------------------------------------
if P.meta.with_payload, swift_case = 'max payload'; else, swift_case = 'no payload'; end
fprintf('\n*** SWIFT parameters v0.7 (%s) - spreadsheet + MAD X7215 motor data ***\n', swift_case);
fprintf('  %.2f kg | %.2f m span | %.3f m^2 | AR %.1f | wing loading %.1f kg/m^2\n', ...
        P.mass.m, P.geom.b, P.geom.S, P.geom.AR, P.perf.wing_loading);
fprintf('  Weight %.1f N | V_stall %.1f m/s | cruise %.0f m/s (%.2fx stall margin)\n', ...
        P.mass.W, P.perf.V_stall, P.init.V0, P.init.V0/P.perf.V_stall);
fprintf('  Trim: alpha %.1f deg | elevator %.1f deg | throttle %.2f\n', ...
        rad2deg(P.trim.alpha), rad2deg(P.trim.de), P.init.delta_T);
fprintf('        aileron %.2f deg | rudder %.2f deg (hold off motor torque)\n', ...
        rad2deg(P.trim.da), rad2deg(P.trim.dr));
fprintf('  Aero matrix P.aero.A is %dx%d | induced-drag factor k = %.4f\n', ...
        size(P.aero.A,1), size(P.aero.A,2), P.aero.k_ind);
fprintf('  Battery %dS %.0f Ah (%.0f Wh, %.0f usable) | R %.3f ohm | start %.0f%%\n', ...
        P.batt.cells, P.batt.Ah, P.batt.E_Wh, P.batt.E_usable, P.batt.R_pack, 100*P.batt.SOC0);
fprintf('  Cruise draw %.0f W (%.1f A) -> about %.0f min to the %.0f%% reserve\n', ...
        P.trim.P_batt, P.trim.I_batt, P.trim.endurance_min, 100*P.batt.SOC_res);
if ~P.trim.feasible
    fprintf(2, '  ** CANNOT HOLD LEVEL FLIGHT at %.0f m/s: needs %.1f N thrust, has %.1f N.\n', ...
            P.init.V0, P.trim.drag, P.trim.T_avail);
    fprintf(2, '     Thrust power needed %.0f W vs about %.0f W available from the motor.\n', ...
            P.trim.P_req, P.trim.P_avail);
    fprintf(2, '     Expect the sim to slow down and descend. See section 5.\n');
end

if isempty(swift_missing)
    fprintf('  All %d variables swift_6dof reads are defined.\n', numel(swift_required));
else
    fprintf(2, '  ** MISSING: %s\n', strjoin(swift_missing, ', '));
    fprintf(2, '     The model cannot compile until these exist.\n');
end
fprintf('  Tags to retire: search this file for [EST and [CHECK].\n\n');

clear swift_required swift_missing swift_i swift_parts swift_case ...
      swift_W swift_Vs swift_qS swift_CL swift_ad swift_D swift_Tfull swift_dT swift_vi swift_Trow swift_Lp swift_lat
