function [xdot, batt, VI] = swift_battery_fcn(T, V, rho, x, prm, ocv)
%SWIFT_BATTERY_FCN  Battery model for the SWIFT Battery subsystem.
%
%   [xdot, batt, VI] = swift_battery_fcn(T, V, rho, x, prm, ocv)
%
%   Called every step by the "Battery model" MATLAB Function block inside
%   the Battery subsystem of swift_6dof and swift_6dof_sitl. Plain MATLAB,
%   so you can also call it at the command line to check a single point:
%       [~, b] = swift_battery_fcn(17, 23, 1.225, P.batt.x0, P.batt.prm, P.batt.ocv)
%
%   INPUTS
%     T     N       thrust (Thrust table output)
%     V     m/s     true airspeed (Incidence, Sideslip & Airspeed, port 3)
%     rho   kg/m^3  air density (COESA Atmosphere Model, port 4)
%     x     3x1     Integrator states [SOC (0-1); mAh used; Wh used]
%     prm   13x1    P.batt.prm (order listed in swift_placeholder.m)
%     ocv   2xN     P.batt.ocv, row 1 SOC 0..1, row 2 resting cell volts
%
%   OUTPUTS
%     xdot  3x1     derivatives for the Integrator
%                   [dSOC/dt (1/s); dmAh/dt (mAh/s); dWh/dt (Wh/s)]
%     batt  10x1    logged as out_batt:
%                    1 SOC          -    state of charge, 0-1
%                    2 V_pack       V    terminal voltage under load
%                    3 I            A    pack current
%                    4 P_out        W    power leaving the pack (V_pack * I)
%                    5 mAh_used     mAh
%                    6 Wh_used      Wh
%                    7 V_cell       V    average loaded cell voltage
%                    8 P_shaft      W    propeller shaft power
%                    9 motor_load   -    motor electrical input / its rating
%                   10 status       -    0 OK, 1 below reserve, 2 low volts,
%                                        3 critical volts, 4 overload,
%                                        5 empty (SOC at 0; the model keeps
%                                        supplying power - it measures, it
%                                        does not cut the motor)
%     VI    2x1     [V_pack; I] for the ArduPilot bridge (SITL model only)
%
%   PHYSICS
%     1  Momentum theory turns thrust and airspeed into ideal power:
%          v_i = -V/2 + sqrt(V^2/4 + T/(2 rho A)),   P_ideal = T (V + v_i)
%        which, unlike T*V, is not zero on the takeoff roll.
%     2  Efficiency chain and hotel load give the battery's demand:
%          P_batt = P_ideal / (eta_prop eta_mot eta_esc) + P_hotel / eta_pdb
%     3  Thevenin pack: V_pack = V_oc(SOC) - I R. With P_batt = V_pack I,
%          I = 2 P_batt / (V_oc + sqrt(V_oc^2 - 4 R P_batt))
%        (the stable root, written so it never divides by R). If the pack
%        cannot deliver P_batt at all, I is held at V_oc/(2R), the most power
%        it can give, and status = 4.
%     4  Coulomb counting: dSOC/dt = -I / (3600 * Ah).
%
%   Ryan Bilock, ARTS Lab | battery model | v0.1
%#codegen

cells = prm(1);   Ah     = prm(2);   R     = prm(3);   A     = prm(4);
e_pr  = prm(5);   e_mot  = prm(6);   e_esc = prm(7);
Phot  = prm(8);   e_pdb  = prm(9);
Vlow  = prm(10);  Vcrit  = prm(11);  SOCr  = prm(12);  Pmot  = prm(13);

% ---- 1 | propeller power ---------------------------------------------------
T   = max(T, 0);                     % windmilling (negative thrust) ignored
V   = max(V, 0);
rho = max(rho, 0.05);
vi  = -V/2 + sqrt(V^2/4 + T/(2*rho*A));            % m/s induced velocity
P_shaft = T * (V + vi) / e_pr;                       % W

% ---- 2 | battery demand ----------------------------------------------------
P_mot  = P_shaft / e_mot;                            % W  into the motor
P_batt = P_mot / e_esc + Phot / e_pdb;               % W  out of the pack

% ---- 3 | terminal voltage and current --------------------------------------
soc  = min(max(x(1), 0), 1);
Voc  = cells * interp1(ocv(1,:), ocv(2,:), soc, 'linear');   % V
disc = Voc^2 - 4*R*P_batt;
status = 0;
if disc >= 0
    I = 2*P_batt / (Voc + sqrt(disc));               % A
else
    I = Voc / (2*R);                                 % A  max-power point
    status = 4;
end
Vp = Voc - I*R;                                      % V
Vc = Vp / cells;                                     % V per cell

% ---- warnings, worst one wins ----------------------------------------------
if soc <= 0
    status = 5;
elseif status < 4
    if Vc < Vcrit
        status = 3;
    elseif Vc < Vlow
        status = 2;
    elseif soc <= SOCr
        status = 1;
    end
end

% ---- 4 | states ------------------------------------------------------------
xdot = [-I / (3600*Ah);      % 1/s
         I / 3.6;            % mAh/s  (1 A for 1 s = 1/3.6 mAh)
         Vp*I / 3600];       % Wh/s

batt = [soc; Vp; I; Vp*I; x(2); x(3); Vc; P_shaft; P_mot/Pmot; status];
VI   = [Vp; I];
end
