function S = swift_sitl_cub_e(varargin)
%SWIFT_SITL_CUB_E  Session E: out-and-back range test from Owens Downtown (CUB) at best-range speed.
%
%   swift_sitl_cub_e                          set up P, write mission + params
%   swift_sitl_cub_e('addr', '172.20.240.1')  same, with your Windows address
%   S = swift_sitl_cub_e(...)                 struct with the files, the best-range
%                                             table, the plan and the estimate
%
%   OPTIONS
%     'addr'      ''      Windows address seen from WSL (fills in the Ubuntu command)
%     'land_mAh'  3000    mAh the model should still have at touchdown. The turn-back
%                         point is moved until the estimate lands with this much left.
%                         ArduPlane's low-battery RTL fires at 2000 mAh left
%                         (BATT_LOW_MAH), so 3000 leaves 1000 mAh of margin.
%     'v_range'   []      m/s, fly this cruise speed instead of the computed best
%     'x_turn'    []      m, fixed turn-back distance instead of the energy solve
%     'alt'       100     m AGL cruise altitude (out and back)
%     'write'     true    write the .waypoints and .param files
%
%   Run it AFTER swift_sitl_preflight (preflight rebuilds P and would put the
%   parked heading back to north). Sessions C and D are not touched: Session E
%   writes its own .waypoints and .param files.
%
%   WHAT THE FLIGHT IS
%     Takeoff on runway 13, straight climb-out to 100 m, then one long straight
%     leg out along the runway heading at the best-range airspeed, ONE gentle
%     right 180 (radius 325 m, about 10 deg of bank), the same leg back 650 m to
%     the right of the outbound line - which is the right-hand downwind for
%     runway 13 - a slow descent to pattern height and the Session C/D autoland.
%     No other manoeuvres: the two long legs are the measurement.
%
%   BEST-RANGE SPEED  (section "best range")
%     Distance per unit of charge is  V / I(V).  In level flight the thrust is
%     the drag,
%         T(V) = D(V) = q S (CD0 + k CL^2),    CL = W / (q S),   q = rho V^2 / 2
%     and the pack current I(V) comes from the same swift_battery_fcn the
%     Simulink Battery block runs (momentum-theory prop, motor efficiency,
%     hotel load, pack resistance). The script sweeps V from AIRSPEED_MIN to
%     27 m/s and picks the speed with the lowest mAh per km. Throttle at that
%     speed comes from inverting the MAD X7215 thrust table. ArduPlane flies
%     airspeed, not throttle, so the mission commands the SPEED and TECS finds
%     the throttle; TRIM_THROTTLE is set to the computed value so it starts
%     there.
%
%   TURN-BACK POINT
%     The whole mission is estimated leg by leg (same method as Session D:
%     thrust from drag plus climb term, current and charge from
%     swift_battery_fcn in 1 s steps) and the out-leg length is solved so the
%     estimate touches down with 'land_mAh' left.
%
%   RUNWAY FRAME  (used for every point)
%     x  m  along runway 13 from its threshold (125.55 deg true, toward SE)
%     y  m  to the right of the centreline (215.55 deg true, toward SW)
%
%   Tags:  [OPS] mission-planning choice, change freely   [FAA] airport data
%          [EST] estimate
%
%   This is a simulation plan only. The out leg runs 15+ km SE of CUB, under the
%   Columbia (CAE) Class C shelf and toward the Fort Jackson restricted areas,
%   and is far beyond visual line of sight. Do not fly this route for real.
%
%   Ryan Bilock, ARTS Lab | SITL session E | v0.1
%     v0.1  new: out-and-back at the computed best-range airspeed, energy-solved
%           turn-back point, single gentle 180, Session C/D landing pattern

%% ---- options -------------------------------------------------------------
o.addr = '<step-6 address>';  o.write = true;  o.set_p = true;
o.land_mah = 3000;  o.v_range = [];  o.x_turn = [];  o.alt = 100;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
hr = repmat('-', 1, 78);

%% ---- airport [FAA] (same as Sessions C and D) ------------------------------
A.id      = 'KCUB';
A.lat0    = 33 + 58.467377/60;          % deg   RWY 13 threshold
A.lon0    = -(81 + 0.119555/60);        % deg
A.elev    = 193.4 * 0.3048;             % m AMSL (58.9)
A.hdg     = 125.55;                     % deg true, RWY 13
A.len     = 1528;                       % m
A.dthr    = 119;                        % m     RWY 13 displaced threshold

%% ---- speeds (match swift_sitl.param) [OPS] --------------------------------
V.cr   = 23;        % m/s  AIRSPEED_CRUISE, used for the climb-out
V.slow = 22;        % m/s  AIRSPEED_MIN - slowest ArduPlane will hold
V.top  = 27;        % m/s  top of the sweep (AIRSPEED_MAX is 28)
V.rot  = 19;        % m/s  TKOFF_ROTATE_SPD
L.disarm = 20;      % s    LAND_DISARMDELAY
L.Rturn  = 325;     % m    turn-back radius: about 10 deg of bank at 23 m/s [OPS]
L.ylane  = 2 * L.Rturn;   % m  return lane, right of the centreline (= downwind)

%% ---- park the aircraft on runway 13 (P is needed for everything below) ------
if o.set_p
    if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
    evalin('base', sprintf('P.sitl.hdg0 = %.2f; P.sitl.start = ''ground''; P = swift_sitl_params(P);', A.hdg));
end
havP = evalin('base', 'exist(''P'',''var'')');
if havP, P = evalin('base', 'P'); else, P = []; end
here = fileparts(mfilename('fullpath'));
S.batt_file = fullfile(fileparts(here), 'ardupilot', 'swift_sitl_batt.param');   % shared ArduPlane params
okB = battery_ready(P, S.batt_file, hr);
B = batt_model(P);
if ~B.ok
    error('swift_sitl_cub_e:nobatt', ['Session E needs P.batt and swift_battery_fcn to pick the ' ...
          'speed and the turn-back point. Run swift_setup, then swift_sitl_preflight.']);
end
if isfield(P, 'sitl') && isfield(P.sitl, 'prop_used') && strcmp(P.sitl.prop_used, 'test')
    error('swift_sitl_cub_e:testprop', ['P is on the SITL test propeller. Session E needs the MAD ' ...
          'X7215 table (swift_placeholder v0.6+, P.sitl.prop = ''sheet'').']);
end

%% ---- best range: sweep airspeed ------------------------------------------------
Vs = P.perf.V_stall;
Vlo = max(V.slow, ceil(11.5 * Vs) / 10);              % never below 1.15 x stall
xm  = [0.6; 0; 0];                                    % mid-flight charge for the current
vv  = (Vlo:0.1:V.top)';
TT  = B.D(vv, 1);
II  = arrayfun(@(T, v) B.I(xm, T, v), TT, vv);
thr = arrayfun(@(T, v) B.thr(T, v), TT, vv);
PP  = arrayfun(@(T, v) B.Pb(xm, T, v), TT, vv);
mpk = II ./ vv * 1000 / 3.6;                          % mAh per km
[~, ib] = min(mpk);
vfree = (0.5 * Vs:0.1:V.top)';                        % where the optimum would be with no floor
[~, ifr] = min(arrayfun(@(v) B.I(xm, B.D(v, 1), v), vfree) ./ vfree);
Vopt = vfree(ifr);
Vbr  = round(2 * vv(ib)) / 2;                         % to 0.5 m/s, the curve is flat
if ~isempty(o.v_range), Vbr = o.v_range; end
Vbr  = min(max(Vbr, Vlo), V.top);
[~, ibr] = min(abs(vv - Vbr));
BR = struct('V', vv, 'T', TT, 'I', II, 'P', PP, 'thr', thr, 'mAh_per_km', mpk, ...
            'V_best', vv(ib), 'V_fly', Vbr, 'V_unconstrained', Vopt, ...
            'thr_fly', thr(ibr), 'I_fly', II(ibr), 'P_fly', PP(ibr), 'mAh_km_fly', mpk(ibr), ...
            'km_per_Ah', 1000 / mpk(ibr));
fprintf('\n%s\n SWIFT SITL SESSION E  -  %s runway 13 out-and-back range test\n%s\n', hr, A.id, hr);
fprintf(' BEST-RANGE SPEED [EST]   mass %.2f kg, stall %.1f m/s, sweep %.1f-%.1f m/s\n', ...
        P.mass.m, Vs, Vlo, V.top);
fprintf('   %5s %7s %7s %7s %7s %8s %7s\n', 'V m/s', 'drag N', 'thr %', 'I A', 'P W', 'mAh/km', 'km/Ah');
for v = unique([Vlo, 22:1:V.top, Vbr])
    [~, j] = min(abs(vv - v));
    mk = '';  if abs(vv(j) - Vbr) < 0.05, mk = '  <- fly'; end
    fprintf('   %5.1f %7.2f %7.1f %7.2f %7.0f %8.1f %7.2f%s\n', vv(j), TT(j), thr(j), II(j), PP(j), ...
            mpk(j), 1000 / mpk(j), mk);
end
if Vopt < Vlo - 0.05
    fprintf(['   The unconstrained optimum is %.1f m/s, below the %.1f m/s floor (AIRSPEED_MIN /\n' ...
             '   1.15 x stall), so the test flies the floor speed.\n'], Vopt, Vlo);
end
fprintf('   Flying %.1f m/s: about %.0f %% throttle, %.1f A, %.0f W, %.2f km per Ah\n', ...
        Vbr, BR.thr_fly, BR.I_fly, BR.P_fly, BR.km_per_Ah);
fprintf('   (within 1 %% of the best between %.1f and %.1f m/s - the curve is flat)\n', ...
        min(vv(mpk <= 1.01 * min(mpk))), max(vv(mpk <= 1.01 * min(mpk))));

%% ---- turn-back point: solve the out-leg length ------------------------------------
mAh_have  = 1000 * P.batt.Ah * P.batt.SOC0;
mAh_goal  = mAh_have - o.land_mah;                    % used at touchdown
mAh_fs    = 1000 * P.batt.Ah - 2000;                  % ArduPlane BATT_LOW_MAH 2000 -> RTL here
assert(mAh_goal > 1500, 'swift_sitl_cub_e: land_mAh %g leaves no charge for the flight.', o.land_mah);
mk_plan = @(xt) plan(B, P, mission(xt, Vbr, V, L, o.alt), V, L, false);
if isempty(o.x_turn)
    xa = 4000;  xb = 14000;  fa = mk_plan(xa).mAh - mAh_goal;  fb = mk_plan(xb).mAh - mAh_goal;
    for it = 1:8                                      % secant: the cost is nearly linear in xt
        xt = xb - fb * (xb - xa) / (fb - fa);
        xt = min(max(xt, 2500), 60000);
        ft = mk_plan(xt).mAh - mAh_goal;
        xa = xb;  fa = fb;  xb = xt;  fb = ft;
        if abs(ft) < 10, break, end
    end
    xt = floor(xb / 50) * 50;                         % to 50 m, on the safe side
else
    xt = o.x_turn;
end
seg = mission(xt, Vbr, V, L, o.alt);
n   = size(seg, 1);
PL  = plan(B, P, seg, V, L, true, hr);                % prints the table

S.airport = A;  S.speeds = V;  S.seg = seg;  S.dist_m = PL.dist;  S.time_s = PL.t;
S.best_range = BR;  S.x_turn = xt;  S.R_turn = L.Rturn;  S.y_lane = L.ylane;
S.cruise_alt = o.alt;
S.legs = struct('out_x', [1500 + 600, xt - 200], 'out_y', 0, ...
                'back_x', [xt - 200, 2500 + 400], 'back_y', L.ylane);   % steady windows, runway frame
S.batt_est = struct('ok', true, 'SOC_end_pct', 100 * PL.x(1), 'mAh', PL.x(2), 'Wh', PL.x(3), ...
                    'I_peak', PL.Ipk, 'I_leg', PL.Ileg, 'SOC_leg', PL.SOCleg, 'thr_leg', PL.Tleg, ...
                    'mAh_leg', PL.mAhleg, 'dist_leg', PL.dleg, 't_leg', PL.tleg, ...
                    'I_avg', 3.6 * PL.x(2) / max(PL.t + L.disarm, eps), 'land_mAh_left', mAh_have - PL.x(2));
Voc_end = P.batt.cells * interp1(P.batt.ocv(1,:), P.batt.ocv(2,:), PL.x(1), 'linear');
S.batt_est.Voc_end = Voc_end;

fprintf('%s\n Turn back at x = %.1f km along the runway heading, out-and-back ground distance ~%.1f km\n', ...
        hr, xt / 1000, PL.dist / 1000);
fprintf(' Arm-to-disarm ~%.1f min   cruise legs at %.1f m/s, ~%.0f %% throttle\n', ...
        (PL.t + L.disarm) / 60, Vbr, BR.thr_fly);
fprintf(' Battery [EST]  %.0f%% -> %.0f%%  |  %.0f mAh, %.0f Wh used  |  peak %.0f A, avg %.1f A\n', ...
        100 * P.batt.SOC0, 100 * PL.x(1), PL.x(2), PL.x(3), PL.Ipk, S.batt_est.I_avg);
fprintf(' Lands with ~%.0f mAh left. Low-battery RTL (at %.0f mAh used) margin: %.0f mAh.\n', ...
        mAh_have - PL.x(2), mAh_fs, mAh_fs - PL.x(2));
fprintf(' Resting pack voltage at touchdown ~%.1f V (BATT_LOW_VOLT 42.0 V)\n', Voc_end);
if PL.x(2) > mAh_fs
    fprintf(2, ' Estimate passes BATT_LOW_MAH: ArduPlane would RTL early. Raise land_mAh.\n');
end
if Voc_end < 42.5
    fprintf(2, ' Resting voltage ends within 0.5 V of BATT_LOW_VOLT: the voltage failsafe may fire.\n');
end
if P.batt.SOC0 < 0.999
    fprintf(2, [' P.batt.SOC0 = %.2f: ArduPlane assumes a full pack at boot, so its mAh failsafe\n' ...
                ' will be later than the model. Use a full pack for this test.\n'], P.batt.SOC0);
end

%% ---- files --------------------------------------------------------------
S.wp_file  = fullfile(here, 'swift_sitl_cub_e.waypoints');
S.prm_file = fullfile(here, 'swift_sitl_cub_e.param');
Re = 6378137;
c = cosd(A.hdg);  s = sind(A.hdg);
xy2ll = @(x, y) [A.lat0 + rad2deg((x*c - y*s) / Re), ...
                 A.lon0 + rad2deg((x*s + y*c) / (Re * cosd(A.lat0)))];
alts = cell2mat(seg(:,5));
assert(all(alts(~isnan(alts)) <= 121.9), 'swift_sitl_cub_e: a point is above 400 ft (120 m).');

if o.write
    fid = fopen(S.wp_file, 'w');  assert(fid > 0, 'Could not write %s', S.wp_file);
    cw  = onCleanup(@() fclose(fid));
    row = @(i, cur, fr, cmd, p, ll, alt) fprintf(fid, ...
        '%d\t%d\t%d\t%d\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1\n', ...
        i, cur, fr, cmd, p(1), p(2), p(3), p(4), ll(1), ll(2), alt);
    fprintf(fid, 'QGC WPL 110\n');
    row(0, 1, 0, 16, [0 0 0 0], [A.lat0 A.lon0], A.elev);          % HOME, AMSL
    for k = 1:n
        typ = seg{k,2};  xr = seg{k,3};  yr = seg{k,4};  h = seg{k,5};
        if isnan(xr), ll = [0 0]; else, ll = xy2ll(xr, yr); end
        switch typ
            case 'TAKEOFF',    row(k, 0, 3,  22, [seg{k,6} 0 0 0], [0 0], h);
            case 'WP',         row(k, 0, 3,  16, [0 0 0 0], ll, h);
            case 'SPEED',      row(k, 0, 3, 178, [0 seg{k,6} -1 0], [0 0], 0);
            case 'LAND_START', row(k, 0, 3, 189, [0 0 0 0], ll, 0);
            case 'LAND',       row(k, 0, 3,  21, [0 0 0 0], ll, 0);
            otherwise, error('swift_sitl_cub_e: unknown type %s', typ);
        end
    end
    clear cw

    prm = { ...
      'TRIM_THROTTLE',    round(BR.thr_fly), sprintf('%%    level-flight throttle at the %.1f m/s best-range speed [EST] (swift_sitl.param had 56)', Vbr);
      'TECS_CLMB_MAX',    2.0,      'm/s  climb-out rate, as Session D (swift_sitl.param had 1.1)';
      'TECS_LAND_ARSPD',  V.slow,   'm/s  approach airspeed (= AIRSPEED_MIN)';
      'TECS_LAND_SINK',   0.6,      'm/s  sink rate after the flare';
      'LAND_FLARE_ALT',   3,        'm    flare starts at this height ...';
      'LAND_FLARE_SEC',   2,        's    ... or this many seconds to touchdown';
      'LAND_PITCH_DEG',   1,        'deg  nose-up target in the flare (tricycle gear)';
      'LAND_DISARMDELAY', L.disarm, 's    disarm this long after touchdown';
      'RTL_AUTOLAND',     2,        '-    an RTL (incl. battery failsafe) jumps to the landing sequence';
      };
    fid = fopen(S.prm_file, 'w');  assert(fid > 0, 'Could not write %s', S.prm_file);
    cp  = onCleanup(@() fclose(fid));
    fprintf(fid, '# SWIFT UAV - SITL session E (KCUB runway 13, out-and-back range test)\n');
    fprintf(fid, '# Written by swift_sitl_cub_e v0.1, %s\n', datestr(now, 'yyyy-mm-dd HH:MM'));
    fprintf(fid, '# Load after plane.parm and swift_sitl.param (batt and vtail files last).\n');
    fprintf(fid, '# Best-range speed %.1f m/s is commanded in the mission (DO_CHANGE_SPEED), not here.\n', Vbr);
    for k = 1:size(prm, 1)
        fprintf(fid, '# %s\n%s,%g\n', prm{k,3}, prm{k,1}, prm{k,2});
    end
    clear cp
    fprintf(' Wrote %s\n Wrote %s\n', S.wp_file, S.prm_file);
end
if o.set_p, fprintf(' P.sitl.hdg0 = %.2f deg  (parked on runway 13, nose toward SE)\n', A.hdg); end

%% ---- run order ------------------------------------------------------------
w = @(f) wsl_or_native(f);   % Windows C:\... -> /mnt/c/... ; Linux/macOS paths unchanged
nItems = n + 1;
iLS = find(strcmp(seg(:,2), 'LAND_START'));  iLand = find(strcmp(seg(:,2), 'LAND'));
iOut = find(strcmp(seg(:,1), 'Out leg'));  iBack = find(strcmp(seg(:,1), 'Back leg'));
fprintf('%s\n RUN ORDER\n%s\n', hr, hr);
fprintf(' 1  MATLAB   open_system(''swift_6dof_sitl'') and press Run (it waits).\n');
fprintf(' 2  Ubuntu   D="%s"\n', w(here));
fprintf('             A="%s"\n', w(fullfile(fileparts(here), 'ardupilot')));
fprintf('             cp "$A"/swift_sitl.param "$D"/swift_sitl_cub_e.param \\\n');
fprintf('                "$A"/swift_sitl_batt.param "$A"/swift_sitl_vtail.param "$D"/swift_sitl_cub_e.waypoints ~/\n');
fprintf('             cd ~/ardupilot/ArduPlane\n');
fprintf(['             ../Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:%s -N -w \\\n' ...
         '               --custom-location=%.7f,%.7f,%.1f,%.1f \\\n' ...
         '               --add-param-file=../Tools/autotest/models/plane.parm \\\n' ...
         '               --add-param-file=$HOME/swift_sitl.param \\\n' ...
         '               --add-param-file=$HOME/swift_sitl_cub_e.param \\\n' ...
         '               --add-param-file=$HOME/swift_sitl_batt.param \\\n' ...
         '               --add-param-file=$HOME/swift_sitl_vtail.param \\\n' ...
         '               --out=udp:%s:14550 --console --map\n'], ...
         o.addr, A.lat0, A.lon0, A.elev, A.hdg, o.addr);
fprintf(' 3  MAVProxy wait for "EKF3 IMU0 is using GPS", then\n');
fprintf('             wp load ../../swift_sitl_cub_e.waypoints    (expect %d items incl. home)\n', nItems);
fprintf('             param show TRIM_THROTTLE     (expect %d)\n', round(BR.thr_fly));
fprintf('             param show SERVO2_FUNCTION   (expect 79 - V-tail left)\n');
fprintf('             param show BATT_MONITOR      (expect 4; console Batt ~50 V)\n');
fprintf('             mode FBWA  ->  arm throttle  ->  mode AUTO\n');
fprintf(' 4  Watch    item %d = out leg, %d = back leg, %d = landing sequence, %d = LAND.\n', ...
        iOut, iBack, iLS, iLand);
fprintf('             It lands and disarms on its own %d s after touchdown (~%.0f min).\n', L.disarm, (PL.t + L.disarm) / 60);
fprintf(' 5  Stop     Simulink Stop first, then Ctrl+C in Ubuntu.\n');
fprintf(' 6  Plot     outE = out;\n');
fprintf('             R  = swift_plot_flight(outE, ''tag'', ''sessionE_cub'', ''wp'', S.wp_file);\n');
fprintf('             RC = swift_range_check(outE, ''tag'', ''sessionE_cub'');\n');
fprintf('             VT = swift_vtail_check(outE, ''tag'', ''sessionE_cub'');\n');
fprintf('             swift_flight_video(outE, ''tag'', ''sessionE_cub'', ''speed'', 20);\n');
fprintf('%s\n\n', hr);
if ~okB
    fprintf(2, ' Battery not ready - the flight will run but battery plots will be empty. See above.\n\n');
end
end

% =========================================================================
function seg = mission(xt, Vbr, V, L, alt)
%  The Session E mission in the runway frame for a turn-back point xt (m).
%  name                          type          x            y         alt   p1    p2
R = L.Rturn;  yl = L.ylane;
seg = { ...
 'Takeoff, climb to 30 m',      'TAKEOFF',      0,            0,        30,  10,    0;
 'Climb-out speed',             'SPEED',        NaN,          NaN,      NaN, V.cr,  0;
 'Climb out to cruise alt',     'WP',           1500,         0,        alt,  0,    0;
 'Best-range speed',            'SPEED',        NaN,          NaN,      NaN, Vbr,   0;
 'Out leg',                     'WP',           xt,           0,        alt,  0,    0;
 'Turn back: 60 deg right',     'WP',           xt + R*sind(60), R - R*cosd(60),  alt, 0, 0;
 'Turn back: 120 deg right',    'WP',           xt + R*sind(120), R - R*cosd(120), alt, 0, 0;
 'Turn back: on return lane',   'WP',           xt,           yl,       alt,  0,    0;
 'Back leg',                    'WP',           2500,         yl,       alt,  0,    0;
 'Descend to 45 m',             'WP',           -200,         yl,       45,   0,    0;
 'Landing sequence start',      'LAND_START',   200,          0,        0,    0,    0;
 'Pattern speed',               'SPEED',        NaN,          NaN,      NaN, V.slow, 0;
 'Downwind, 40 m',              'WP',           -1100,        yl,       40,   0,    0;
 'Right base, 35 m',            'WP',           -1100,        0,        35,   0,    0;
 'Right onto final, 28 m',      'WP',           -600,         0,        28,   0,    0;
 'Touchdown, runway 13',        'LAND',         200,          0,        0,    0,    0;
 };
end

function PL = plan(B, P, seg, V, L, show, hr)
%  Leg-by-leg estimate of the whole mission (Session D method).
if nargin < 7, hr = ''; end
n = size(seg, 1);
x = P.batt.x0(:);
cur = [0 0 0];  vnow = V.cr;  dist = 0;  tsec = 0;  prevH = NaN;  Ipk = 0;
Ileg = nan(n, 1);  SOCleg = nan(n, 1);  Tleg = nan(n, 1);  mAhleg = nan(n, 1);
dleg = zeros(n, 1);  tleg = zeros(n, 1);
if show
    fprintf('%s\n MISSION PLAN [EST]\n', hr);
    fprintf(' %-2s %-27s %-10s %7s %5s %5s %6s %6s %5s %7s %6s\n', '#', 'segment', 'type', ...
        'leg m', 'alt', 'm/s', 'turn', 'est A', 'thr%', 'mAh', 'SOC%');
end
for k = 1:n
    typ = seg{k,2};  p = [seg{k,3} seg{k,4} seg{k,5}];
    if strcmp(typ, 'SPEED'), vnow = seg{k,6}; end
    Lk = 0;  turn = '';  tk = 0;
    if any(strcmp(typ, {'TAKEOFF', 'WP', 'LAND'}))
        if strcmp(typ, 'TAKEOFF')
            [x, tk, Lk, Ik, Ipk] = takeoff_leg(B, x, V.rot, seg{k,6}, p(3), Ipk);
            Tk = 100;  p(1) = Lk;
        else
            d = p(1:2) - cur(1:2);  Lk = hypot(d(1), d(2));
            if Lk > 1
                H = atan2d(d(2), d(1));
                if ~isnan(prevH)
                    dh = mod(H - prevH + 180, 360) - 180;
                    if abs(dh) > 10, turn = sprintf('%+4.0f %s', dh, char('L' + ('R'-'L')*(dh > 0))); end
                end
                prevH = H;
            end
            tk = Lk / vnow;  dz = p(3) - cur(3);
            [x, Ik, Ipk, Tk] = cruise_leg(B, x, vnow, 1, dz, tk, Ipk);
            if strcmp(typ, 'LAND')
                [x, Ir] = cruise_leg(B, x, 8, 1, 0, 30 + L.disarm, Ipk, true);
                tk = tk + 30;  Ik = (Ik*(tk-30) + Ir*30) / tk;
            end
        end
        dist = dist + Lk;  tsec = tsec + tk;  cur = p;
        Ileg(k) = Ik;  SOCleg(k) = 100*x(1);  Tleg(k) = Tk;  mAhleg(k) = x(2);
        dleg(k) = Lk;  tleg(k) = tk;
    end
    if show
        fprintf(' %-2d %-27s %-10s %7.0f %5s %5.1f %6s %6s %5s %7s %6s\n', k, seg{k,1}, typ, Lk, ...
            numstr(seg{k,5}, '%g'), vnow, turn, numstr(Ileg(k), '%.1f'), numstr(Tleg(k), '%.0f'), ...
            numstr(mAhleg(k), '%.0f'), numstr(SOCleg(k), '%.1f'));
    end
end
PL = struct('x', x, 'mAh', x(2), 'dist', dist, 't', tsec, 'Ipk', Ipk, 'Ileg', Ileg, ...
            'SOCleg', SOCleg, 'Tleg', Tleg, 'mAhleg', mAhleg, 'dleg', dleg, 'tleg', tleg);
end

% =========================================================================
function ok = battery_ready(P, bfile, hr)
ok = true;
fprintf('%s\n BATTERY AND V-TAIL CHECK\n', hr);
if isempty(P) || ~isfield(P, 'batt') || ~isfield(P.batt, 'prm')
    fprintf(2, ' [FAIL] P.batt.prm missing - run swift_placeholder (v0.5 or later).\n');  ok = false;
else
    fprintf(' [ OK ] P.batt: %dS %.0f Ah, start %.0f%%\n', P.batt.cells, P.batt.Ah, 100*P.batt.SOC0);
end
if ~isempty(P) && isfield(P, 'sitl') && isfield(P.sitl, 'send_batt')
    if P.sitl.send_batt > 0.5
        fprintf(' [ OK ] P.sitl.send_batt = 1 (pack voltage and current go to ArduPlane)\n');
    else
        fprintf(2, ' [WARN] P.sitl.send_batt = 0 - ArduPlane will show SIM_BATT values, not the model.\n');
    end
end
if isfile(bfile)
    fprintf(' [ OK ] %s\n', bfile);
else
    fprintf(2, ' [FAIL] %s not found - run:  swift_battery_add swift_6dof_sitl\n', bfile);  ok = false;
end
if ~isempty(P) && isfield(P, 'tail') && isfield(P.tail, 'drv_max')
    fprintf(' [ OK ] P.tail: inverted V, ruddervators +/- %.0f deg, MIXING_GAIN %.2g\n', ...
            rad2deg(P.tail.drv_max), P.tail.mix_gain);
else
    fprintf(2, ' [WARN] P.tail missing - V-tail not installed (swift_vtail_add). swift_vtail_check will have nothing to check.\n');
end
vf = fullfile(fileparts(bfile), 'swift_sitl_vtail.param');
if isfile(vf)
    fprintf(' [ OK ] %s\n', vf);
else
    fprintf(2, ' [WARN] %s not found - run:  swift_vtail_add params\n', vf);
end
M = 'swift_6dof_sitl';
try
    load_system(M);
    for nm = {'out_batt', 'out_rv'}
        b = find_system(M, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                        'BlockType', 'ToWorkspace', 'VariableName', nm{1});
        if isempty(b)
            fprintf(2, ' [FAIL] no %s logger in %s\n', nm{1}, M);
            if strcmp(nm{1}, 'out_batt'), ok = false; end
        else
            fprintf(' [ OK ] %s logs %s\n', M, nm{1});
        end
    end
catch
    fprintf(' [ -- ] could not open %s to check the loggers (skipped)\n', M);
end
end

function B = batt_model(P)
B.ok = ~isempty(P) && isfield(P, 'batt') && isfield(P.batt, 'prm') && ...
       ~isempty(which('swift_battery_fcn'));
if ~B.ok, return, end
rho = P.const.rho0;  W = P.mass.m * P.const.g;  Sw = P.geom.S;
k   = 1 / (pi * P.aero.e_osw * P.geom.b^2 / Sw);
if isfield(P.aero, 'k_ind'), k = P.aero.k_ind; end
B.W = W;  B.m = P.mass.m;  B.rho = rho;  B.mu = 0.04;
if isfield(P, 'gnd') && isfield(P.gnd, 'mu_roll'), B.mu = P.gnd.mu_roll; end
B.D = @(V, n) (P.aero.CD0 + k * (n.*W ./ (0.5*rho*V.^2*Sw)).^2) .* (0.5*rho*Sw*V.^2);
B.Tmax = @(V) P.prop.T_static_max * max(0, 1 - V / P.prop.V_pitch);
B.thr = @(T, V) 100 * interp1(P.prop.T_static(:), P.prop.throttle_bp(:), ...
            min(max(T / max(1 - V / P.prop.V_pitch, 0.05), 0), P.prop.T_static(end)), 'linear');
B.step = @(x, T, V, dt) x + dt * swift_battery_fcn(T, V, rho, x, P.batt.prm, P.batt.ocv);
B.I    = @(x, T, V) pick(swift_battery_fcn_out(T, V, rho, x, P.batt.prm, P.batt.ocv), 3);
B.Pb   = @(x, T, V) pick(swift_battery_fcn_out(T, V, rho, x, P.batt.prm, P.batt.ocv), 4);
end

function v = pick(b, i), v = b(i); end
function b = swift_battery_fcn_out(T, V, rho, x, prm, ocv)
[~, b] = swift_battery_fcn(T, V, rho, x, prm, ocv);
end

function [x, t, L, Iavg, Ipk] = takeoff_leg(B, x, Vr, pitch, h, Ipk)
t = 0;  L = 0;  Vg = 0.5;  dt = 0.1;  Q = 0;
while Vg < Vr && t < 120                       % roll
    T  = B.Tmax(Vg);
    a  = (T - B.D(max(Vg, 1), 0) - B.mu * B.W) / B.m;
    I  = B.I(x, T, Vg);  Ipk = max(Ipk, I);  Q = Q + I*dt;
    x  = B.step(x, T, Vg, dt);
    Vg = Vg + max(a, 0.05) * dt;  L = L + Vg * dt;  t = t + dt;
end
Vc = Vr + 2;  T = B.Tmax(Vc);                  % climb: limited by pitch or excess thrust
vz = min(Vc * sind(pitch), max(0.3, (T - B.D(Vc, 1)) * Vc / B.W));
tc = h / vz;
for kk = 1:ceil(tc)
    dtk = min(1, tc - (kk-1));
    I = B.I(x, T, Vc);  Ipk = max(Ipk, I);  Q = Q + I*dtk;
    x = B.step(x, T, Vc, dtk);
end
t = t + tc;  L = L + Vc * tc;  Iavg = Q / t;
end

function [x, Iavg, Ipk, thr] = cruise_leg(B, x, V, n, dz, t, Ipk, idle)
Iavg = NaN;  thr = NaN;
if t <= 0, return, end
if nargin < 8, idle = false; end
if idle
    T = 0;  thr = 0;
else
    T = B.D(V, n) + B.W * (dz / t) / V;
    T = min(max(T, 0), B.Tmax(V));
    thr = B.thr(T, V);
end
Q = 0;
for kk = 1:ceil(t)
    dtk = min(1, t - (kk-1));
    I = B.I(x, T, V);  Ipk = max(Ipk, I);  Q = Q + I*dtk;
    x = B.step(x, T, V, dtk);
end
Iavg = Q / t;
end

function s = numstr(v, f)
if isnan(v), s = '-'; else, s = sprintf(f, v); end
end

function p = wsl_or_native(p)
%  Path as the ArduPilot shell sees it: Windows drive paths become WSL /mnt/<drive>/...
if numel(p) > 2 && p(2) == ':'
    p = ['/mnt/' lower(p(1)) strrep(p(3:end), '\', '/')];
end
end
