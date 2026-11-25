function S = swift_sitl_cub_d(varargin)
%SWIFT_SITL_CUB_D  Session D: motor-validation flight from Owens Downtown (CUB), Columbia SC.
%
%   swift_sitl_cub_d                         set up P, write mission + params
%   swift_sitl_cub_d('addr', '172.20.240.1') same, with your Windows address
%   S = swift_sitl_cub_d(...)                struct with the files, plan and
%                                            battery / throttle estimate
%
%   Run it AFTER swift_sitl_preflight (preflight rebuilds P and would put
%   the parked heading back to north). Session C (swift_sitl_cub) is not
%   touched: Session D writes its own .waypoints and .param files.
%
%   WHY A NEW FLIGHT
%     Session C flew on the stand-in propeller. swift_placeholder v0.6 now holds
%     the real MAD X7215 IPE 220KV / 21x10 / AMPX 120A / 12S bench table, so
%     this flight is built to exercise the whole throttle range and compare
%     it with that table afterwards (swift_motor_check). The plan below prints
%     the throttle and pack current each leg should show.
%
%   WHAT IT DOES
%     1  Parks the aircraft on runway 13 at the threshold, nose on the
%        runway's true heading (P.sitl.hdg0 = 125.55 deg).
%     2  Checks the battery is ready (same checks as Session C).
%     3  Writes sitl/swift_sitl_cub_d.waypoints and sitl/swift_sitl_cub_d.param.
%     4  Prints the plan with a leg-by-leg estimate of throttle, pack current
%        and charge left, the Ubuntu launch command and the plot commands.
%
%   RUNWAY FRAME  (used for every point below)
%     x  m  along runway 13 from its threshold (125.55 deg true, toward SE)
%     y  m  to the right of the centreline (215.55 deg true, toward SW)
%     Right-hand traffic for runway 13 puts the pattern on the +y side.
%
%   THE FLIGHT  (about 17.6 km, about 15 min airborne, ceiling 120 m AGL)
%      1  Takeoff roll at full throttle, climb to 30 m
%      2  Max-rate climb to 90 m, then right turns up to 120 m
%      3  PROPULSION STAIRCASE on a straight line at y = 1300 m, 700 m a step:
%           22 m/s level, 22 m/s gentle descent, 24, 26 and 27.5 m/s level
%         Each step is long enough for a clean steady window (10+ s).
%      4  Figure-8: one LEFT circle then one RIGHT circle (100 m radius)
%      5  Descending LEFT spiral to 55 m (low throttle), then a climbing
%         RIGHT spiral to 100 m (high throttle)
%      6  Slalom: seven gates alternating left and right, 450 m apart
%      7  Upwind leg, two LEFT turns onto downwind, then the same right-hand
%         pattern and autoland as Session C (touchdown at x = 200 m)
%   Together the legs span roughly 30 % to 100 % throttle, the bench table.
%
%   ESTIMATES  [EST]
%     Thrust per leg from level-flight drag plus the climb (or minus the
%     descent) term at the turn load factor in the circles:
%        T = D(V, n) + W * (dh/dt) / V,   0 <= T <= T_max(V)
%     Throttle comes from inverting the thrust table, T_static(thr) *
%     (1 - V/V_pitch). Current and charge use the same swift_battery_fcn the
%     Simulink Battery block runs, in 1 s steps. A planning check, not the
%     flight result: the real run is what swift_motor_check compares.
%
%   Sources for the runway (FAA data via AirNav / SkyVector, Nov 2026):
%     RWY 13 threshold 33-58.467377N 081-00.119555W, 193.4 ft
%     RWY 31 threshold 33-57.988432N 080-59.311278W, 182.4 ft
%     5011 x 75 ft, RWY 13 displaced threshold 391 ft (119 m)
%   The model's ground is flat; the real runway drops 3.4 m end to end.
%
%   Tags:  [OPS] mission-planning choice, change freely   [FAA] airport data
%          [EST] estimate
%
%   This is a simulation plan only. Real flight at CUB sits under the
%   Columbia (CAE) Class C shelf and needs airspace authorization, and the
%   far legs (2.7 km from the field) are beyond visual line of sight.
%
%   Ryan Bilock, ARTS Lab | SITL session D | v0.1
%     v0.1  new mission from swift_sitl_cub v0.2: propulsion staircase,
%           figure-8, slalom, climbing/descending spirals; est. throttle
%           column; 120 m ceiling; TRIM_THROTTLE 54 and TECS_CLMB_MAX 2.0
%           for the real motor

%% ---- options -------------------------------------------------------------
o.addr = '<step-6 address>';  o.write = true;  o.set_p = true;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
hr = repmat('-', 1, 78);

%% ---- airport [FAA] -------------------------------------------------------
A.id      = 'KCUB';
A.lat0    = 33 + 58.467377/60;          % deg   RWY 13 threshold
A.lon0    = -(81 + 0.119555/60);        % deg
A.elev    = 193.4 * 0.3048;             % m AMSL (58.9)
A.hdg     = 125.55;                     % deg true, RWY 13 (from the two ends)
A.len     = 1528;                       % m
A.dthr    = 119;                        % m     RWY 13 displaced threshold

%% ---- speeds (match swift_sitl.param) [OPS] --------------------------------
V.cr   = 23;        % m/s  AIRSPEED_CRUISE
V.slow = 22;        % m/s  AIRSPEED_MIN - slowest ArduPlane will hold
V.fast = 27;        % m/s  under AIRSPEED_MAX = 28
V.rot  = 19;        % m/s  TKOFF_ROTATE_SPD
L.disarm = 20;      % s    LAND_DISARMDELAY

%% ---- the mission, runway frame [OPS] --------------------------------------
%  name                          type            x      y     alt   p1     p2
%  TAKEOFF p1 = pitch deg | SPEED p1 = m/s | LOITER_TURNS p1 = turns, p2 = radius
%  (negative radius = counter-clockwise = LEFT circles) | LOITER_TO_ALT p2 = radius
V.s24  = 24;        % m/s  propulsion step 2
V.s26  = 26;        % m/s  propulsion step 3
V.s27  = 27.5;      % m/s  propulsion step 4 (AIRSPEED_MAX is 28)
seg = { ...
 'Takeoff, climb to 30 m',      'TAKEOFF',         0,     0,   30,   10,    0;
 'Cruise speed',                'SPEED',         NaN,   NaN,  NaN, V.cr,    0;
 'Max-rate climb to 90 m',      'WP',            950,     0,   90,    0,    0;
 'Climbing right turn, 100 m',  'WP',           1800,   600,  100,    0,    0;
 'Right turn, climb to 120 m',  'WP',           1500,  1300,  120,    0,    0;
 'Right turn onto test line',   'WP',           1100,  1300,  120,    0,    0;
 'Step 1: 22 m/s level',        'SPEED',         NaN,   NaN,  NaN, V.slow,  0;
 'Step 1 leg, level 120 m',     'WP',            400,  1300,  120,    0,    0;
 'Step 1b: 22 m/s descent',     'WP',           -300,  1300,   95,    0,    0;
 'Step 2: 24 m/s',              'SPEED',         NaN,   NaN,  NaN, V.s24,   0;
 'Step 2 leg',                  'WP',          -1000,  1300,   95,    0,    0;
 'Step 3: 26 m/s',              'SPEED',         NaN,   NaN,  NaN, V.s26,   0;
 'Step 3 leg',                  'WP',          -1700,  1300,   95,    0,    0;
 'Step 4: 27.5 m/s',            'SPEED',         NaN,   NaN,  NaN, V.s27,   0;
 'Step 4 leg',                  'WP',          -2400,  1300,   95,    0,    0;
 'Back to 22 m/s',              'SPEED',         NaN,   NaN,  NaN, V.slow,  0;
 'Figure-8 left circle',        'LOITER_TURNS', -2700,  1300,   95,    1, -100;
 'Figure-8 right circle',       'LOITER_TURNS', -2700,  1560,   95,    1,  100;
 'Left spiral down to 55 m',    'LOITER_TO_ALT',-2700,  1300,   55,    0, -130;
 'Right spiral up to 100 m',    'LOITER_TO_ALT',-2700,  1560,  100,    0,  130;
 'Cruise speed',                'SPEED',         NaN,   NaN,  NaN, V.cr,    0;
 'Slalom gate 1',               'WP',          -2200,  2050,  100,    0,    0;
 'Slalom gate 2',               'WP',          -1750,  1750,  100,    0,    0;
 'Slalom gate 3',               'WP',          -1300,  2050,  100,    0,    0;
 'Slalom gate 4',               'WP',           -850,  1750,  100,    0,    0;
 'Slalom gate 5',               'WP',           -400,  2050,  100,    0,    0;
 'Slalom gate 6',               'WP',             50,  1750,  100,    0,    0;
 'Slalom gate 7',               'WP',            500,  2050,  100,    0,    0;
 'Upwind leg',                  'WP',           1100,  1700,   60,    0,    0;
 'Left turn, 50 m',             'WP',           1100,   650,   50,    0,    0;
 'Left onto downwind, 45 m',    'WP',           -200,   650,   45,    0,    0;
 'Landing sequence start',      'LAND_START',    200,     0,    0,    0,    0;
 'Pattern speed',               'SPEED',         NaN,   NaN,  NaN, V.slow,  0;
 'Downwind, 40 m',              'WP',          -1100,   650,   40,    0,    0;
 'Right base, 35 m',            'WP',          -1100,     0,   35,    0,    0;
 'Right onto final, 28 m',      'WP',           -600,     0,   28,    0,    0;
 'Touchdown, runway 13',        'LAND',          200,     0,    0,    0,    0;
 };
n = size(seg, 1);

%% ---- runway frame -> lat/lon -------------------------------------------
Re = 6378137;                                   % m  WGS-84 equatorial radius
c = cosd(A.hdg);  s = sind(A.hdg);
xy2ll = @(x, y) [A.lat0 + rad2deg((x*c - y*s) / Re), ...
                 A.lon0 + rad2deg((x*s + y*c) / (Re * cosd(A.lat0)))];

%% ---- checks: geometry ----------------------------------------------------
alts = cell2mat(seg(:,5));
assert(all(alts(~isnan(alts)) <= 121.9), 'swift_sitl_cub_d: a point is above 400 ft (120 m).');
land = seg(strcmp(seg(:,2), 'LAND'), :);
assert(land{3} > A.dthr && land{3} < A.len - 600, ...
    'swift_sitl_cub_d: touchdown point is not on the usable runway.');

%% ---- park the aircraft on runway 13 (P is needed for the estimate) -------
if o.set_p
    if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
    evalin('base', sprintf('P.sitl.hdg0 = %.2f; P.sitl.start = ''ground''; P = swift_sitl_params(P);', A.hdg));
end
havP = evalin('base', 'exist(''P'',''var'')');
if havP, P = evalin('base', 'P'); else, P = []; end
if havP && V.slow < 1.15 * P.perf.V_stall
    warning('swift_sitl_cub_d:slow', 'Slowest speed %g m/s is under 1.15 x stall (%.1f m/s).', ...
        V.slow, 1.15 * P.perf.V_stall);
end

%% ---- checks: battery ready ------------------------------------------------
here = fileparts(mfilename('fullpath'));
S.batt_file = fullfile(fileparts(here), 'ardupilot', 'swift_sitl_batt.param');   % shared ArduPlane params
okB = battery_ready(P, S.batt_file, hr);

%% ---- plan: distance, time, battery -----------------------------------------
B = batt_model(P);                                  % empty if P has no battery
x = [];  if B.ok, x = P.batt.x0(:); end
cur = [0 0 0];  vnow = V.cr;  dist = 0;  tsec = 0;  prevH = NaN;
Ileg = nan(n, 1);  SOCleg = nan(n, 1);  Tleg = nan(n, 1);  Ipk = 0;
fprintf('\n%s\n SWIFT SITL SESSION D  -  %s runway 13, Columbia SC   (battery: %s)\n%s\n', ...
    hr, A.id, tern(B.ok, 'estimate on', 'no P.batt - estimate off'), hr);
fprintf(' %-2s %-28s %-13s %6s %6s %4s %6s %6s %6s %6s\n', '#', 'segment', 'type', ...
    'leg m', 'alt m', 'm/s', 'turn', 'est A', 'thr %', 'SOC %');
for k = 1:n
    typ = seg{k,2};  p = [seg{k,3} seg{k,4} seg{k,5}];
    if strcmp(typ, 'SPEED'), vnow = seg{k,6}; end
    Lk = 0;  turn = '';  tk = 0;
    if any(strcmp(typ, {'TAKEOFF','WP','LOITER_TURNS','LOITER_TO_ALT','LAND'}))
        if strcmp(typ, 'TAKEOFF')
            %  Roll to rotate, then a full-throttle climb at ~10 deg to 30 m.
            [x, tk, Lk, Ik, Ipk] = takeoff_leg(B, x, V.rot, seg{k,6}, p(3), Ipk);
            Tk = 100;
            p(1) = Lk;
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
            tk = Lk / vnow;  nk = 1;  dz = p(3) - cur(3);  Tk = NaN;
            if strcmp(typ, 'LOITER_TURNS')
                tk = tk + seg{k,6} * 2*pi*abs(seg{k,7}) / vnow;
                nk = load_factor(P, vnow, seg{k,7});
            elseif strcmp(typ, 'LOITER_TO_ALT')
                tk = tk + max(0, -dz) / 1.9 + max(0, dz) / 1.5;     % ~1.9 m/s sink, ~1.5 m/s climb
                nk = load_factor(P, vnow, seg{k,7});
            end
            [x, Ik, Ipk, Tk] = cruise_leg(B, x, vnow, nk, dz, tk, Ipk);
            if strcmp(typ, 'LAND')
                %  Flare, rollout to a stop, then the disarm delay: hotel load only.
                [x, Ir] = cruise_leg(B, x, 8, 1, 0, 30 + L.disarm, Ipk, true);
                tk = tk + 30;  if B.ok, Ik = (Ik*(tk-30) + Ir*30) / tk; end
            end
        end
        dist = dist + Lk;  tsec = tsec + tk;
        cur = p;
        if B.ok, Ileg(k) = Ik;  SOCleg(k) = 100*x(1);  Tleg(k) = Tk; end
    end
    fprintf(' %-2d %-28s %-13s %6.0f %6s %4.1f %6s %6s %6s %6s\n', k, seg{k,1}, typ, Lk, ...
        numstr(seg{k,5}, '%g'), vnow, turn, numstr(Ileg(k), '%.1f'), numstr(Tleg(k), '%.0f'), ...
        numstr(SOCleg(k), '%.1f'));
end
fprintf('%s\n Ground distance ~%.1f km   arm-to-disarm ~%.1f min\n', hr, dist/1000, (tsec + L.disarm)/60);

S.airport = A;  S.speeds = V;  S.seg = seg;  S.dist_m = dist;  S.time_s = tsec;
S.batt_est = struct('ok', B.ok);
if B.ok
    S.batt_est.SOC_end_pct = 100*x(1);   S.batt_est.mAh = x(2);   S.batt_est.Wh = x(3);
    S.batt_est.I_peak = Ipk;             S.batt_est.I_leg = Ileg;  S.batt_est.SOC_leg = SOCleg;
    S.batt_est.thr_leg = Tleg;
    S.batt_est.I_avg = 3.6 * x(2) / max(tsec + L.disarm, eps);
    mAh_low = 1000*P.batt.Ah - 2000;                         % BATT_LOW_MAH 2000 -> RTL here
    fprintf(' Battery [EST]  %.0f%% -> %.0f%%  |  %.0f mAh, %.0f Wh used  |  peak %.0f A, avg %.0f A\n', ...
        100*P.batt.SOC0, S.batt_est.SOC_end_pct, x(2), x(3), Ipk, S.batt_est.I_avg);
    if x(2) < mAh_low
        fprintf(' Low-battery failsafe (RTL at %.0f mAh used) has %.0f mAh of margin - it should not fire.\n', ...
            mAh_low, mAh_low - x(2));
    else
        fprintf(2, [' Estimate passes BATT_LOW_MAH: ArduPlane would RTL (and autoland) before the\n' ...
                    ' mission ends. Start with a full pack (P.batt.SOC0 = 1) or shorten the fast leg.\n']);
    end
end

%% ---- files --------------------------------------------------------------
S.wp_file  = fullfile(here, 'swift_sitl_cub_d.waypoints');
S.prm_file = fullfile(here, 'swift_sitl_cub_d.param');

if o.write
    % --- waypoints (QGC WPL 110: seq cur frame cmd p1 p2 p3 p4 lat lon alt auto)
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
            case 'TAKEOFF',       row(k, 0, 3,  22, [seg{k,6} 0 0 0], [0 0], h);
            case 'WP',            row(k, 0, 3,  16, [0 0 0 0], ll, h);
            case 'SPEED',         row(k, 0, 3, 178, [0 seg{k,6} -1 0], [0 0], 0);
            case 'LOITER_TURNS',  row(k, 0, 3,  18, [seg{k,6} 0 seg{k,7} 0], ll, h);
            case 'LOITER_TO_ALT', row(k, 0, 3,  31, [0 seg{k,7} 0 0], ll, h);
            case 'LAND_START',    row(k, 0, 3, 189, [0 0 0 0], ll, 0);
            case 'LAND',          row(k, 0, 3,  21, [0 0 0 0], ll, 0);
            otherwise, error('swift_sitl_cub_d: unknown type %s', typ);
        end
    end
    clear cw

    % --- landing parameters (ArduPlane 4.5+ names)
    prm = { ...
      'TRIM_THROTTLE',    54,       '%    level-flight throttle at 23 m/s from the MAD X7215 table (swift_sitl.param had 56)';
      'TECS_CLMB_MAX',    2.0,      'm/s  allow the max-rate climb: the real motor has ~2.9 m/s spare at 23 m/s (swift_sitl.param had 1.1)';
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
    fprintf(fid, '# SWIFT UAV - SITL session D (KCUB runway 13, motor-validation flight)\n');
    fprintf(fid, '# Written by swift_sitl_cub_d v0.1, %s\n', datestr(now, 'yyyy-mm-dd HH:MM'));
    fprintf(fid, '# Load after plane.parm and swift_sitl.param (swift_sitl_batt.param last).\n# Replaces TRIM_THROTTLE and TECS_CLMB_MAX from swift_sitl.param for this flight only.\n');
    for k = 1:size(prm, 1)
        fprintf(fid, '# %s\n%s,%g\n', prm{k,3}, prm{k,1}, prm{k,2});
    end
    clear cp
    fprintf(' Wrote %s\n Wrote %s\n', S.wp_file, S.prm_file);
end
if o.set_p
    fprintf(' P.sitl.hdg0 = %.2f deg  (parked on runway 13, nose toward SE)\n', A.hdg);
end

%% ---- run order ------------------------------------------------------------
w = @(f) wsl_or_native(f);   % Windows C:\... -> /mnt/c/... ; Linux/macOS paths unchanged
fprintf('%s\n RUN ORDER\n%s\n', hr, hr);
fprintf(' 1  MATLAB   open_system(''swift_6dof_sitl'') and press Run (it waits).\n');
fprintf(' 2  Ubuntu   D="%s"\n', w(here));
fprintf('             A="%s"\n', w(fullfile(fileparts(here), 'ardupilot')));
fprintf('             cp "$A"/swift_sitl.param "$D"/swift_sitl_cub_d.param \\\n');
fprintf('                "$A"/swift_sitl_batt.param "$A"/swift_sitl_vtail.param "$D"/swift_sitl_cub_d.waypoints ~/\n');
fprintf('             cd ~/ardupilot/ArduPlane\n');
fprintf(['             ../Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:%s -N -w \\\n' ...
         '               --custom-location=%.7f,%.7f,%.1f,%.1f \\\n' ...
         '               --add-param-file=../Tools/autotest/models/plane.parm \\\n' ...
         '               --add-param-file=$HOME/swift_sitl.param \\\n' ...
         '               --add-param-file=$HOME/swift_sitl_cub_d.param \\\n' ...
         '               --add-param-file=$HOME/swift_sitl_batt.param \\\n' ...
         '               --add-param-file=$HOME/swift_sitl_vtail.param \\\n' ...
         '               --out=udp:%s:14550 --console --map\n'], ...
         o.addr, A.lat0, A.lon0, A.elev, A.hdg, o.addr);
fprintf(' 3  MAVProxy wait for "EKF3 IMU0 is using GPS", then\n');
fprintf('             wp load ../../swift_sitl_cub_d.waypoints\n');
fprintf('             param show BATT_MONITOR      (expect 4; console Batt ~50 V)\n');
fprintf('             mode FBWA  ->  arm throttle  ->  mode AUTO\n');
fprintf(' 4  Watch    Mission Planner (UDP 14550) or the MAVProxy map. It lands\n');
fprintf('             and disarms on its own %d s after touchdown.\n', L.disarm);
fprintf(' 5  Stop     Simulink Stop first, then Ctrl+C in Ubuntu.\n');
fprintf(' 6  Plot     outD = out;\n');
fprintf('             R = swift_plot_flight(outD, ''tag'', ''sessionD_cub'', ''wp'', S.wp_file);\n');
fprintf('             (flight, takeoff, landing and battery figures, one run folder)\n');
fprintf(' 7  Motor    M = swift_motor_check(outD, ''tag'', ''sessionD_cub'');\n');
fprintf('             (steady-throttle points against the MAD X7215 bench table)\n');
fprintf('%s\n\n', hr);
if ~okB
    fprintf(2, ' Battery not ready - the flight will run but battery plots will be empty. See above.\n\n');
end
end

% =========================================================================
function ok = battery_ready(P, bfile, hr)
%  Checks the pieces swift_battery_add makes. Prints one line each.
ok = true;
fprintf('%s\n BATTERY CHECK\n', hr);
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
M = 'swift_6dof_sitl';
try
    load_system(M);
    b = find_system(M, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                    'BlockType', 'ToWorkspace', 'VariableName', 'out_batt');
    if isempty(b)
        fprintf(2, ' [FAIL] no out_batt logger in %s - run:  swift_battery_add swift_6dof_sitl\n', M);  ok = false;
    else
        fprintf(' [ OK ] %s logs out_batt\n', M);
    end
catch
    fprintf(' [ -- ] could not open %s to check for out_batt (skipped)\n', M);
end
end

function B = batt_model(P)
%  Handles for the planning estimate. B.ok = false if P lacks what it needs.
B.ok = ~isempty(P) && isfield(P, 'batt') && isfield(P.batt, 'prm') && ...
       ~isempty(which('swift_battery_fcn'));
if ~B.ok, return, end
rho = P.const.rho0;  W = P.mass.m * P.const.g;  Sw = P.geom.S;
k   = 1 / (pi * P.aero.e_osw * P.geom.b^2 / Sw);
B.W = W;  B.m = P.mass.m;  B.rho = rho;  B.mu = 0.04;
if isfield(P, 'gnd') && isfield(P.gnd, 'mu_roll'), B.mu = P.gnd.mu_roll; end
B.D = @(V, n) (P.aero.CD0 + k * (n*W ./ (0.5*rho*V.^2*Sw)).^2) .* 0.5*rho*V.^2*Sw;
if isfield(P, 'sitl') && isfield(P.sitl, 'prop_used') && strcmp(P.sitl.prop_used, 'test')
    B.Tmax = @(V) P.sitl.prop_T0 * max(0, 1 - V / P.sitl.prop_Vp);
else
    B.Tmax = @(V) P.prop.T_static_max * max(0, 1 - V / P.prop.V_pitch);
end
if isfield(P, 'sitl') && isfield(P.sitl, 'prop_used') && strcmp(P.sitl.prop_used, 'test')
    B.thr = @(T, V) NaN;                         % test prop has no bench table
else
    %  throttle (%) that makes thrust T at airspeed V: invert T_static(thr)*(1 - V/V_pitch)
    B.thr = @(T, V) 100 * interp1(P.prop.T_static(:), P.prop.throttle_bp(:), ...
                min(max(T / max(1 - V / P.prop.V_pitch, 0.05), 0), P.prop.T_static(end)), 'linear');
end
B.step = @(x, T, V, dt) x + dt * swift_battery_fcn(T, V, rho, x, P.batt.prm, P.batt.ocv);
B.I    = @(x, T, V) pick3(swift_battery_fcn_out(T, V, rho, x, P.batt.prm, P.batt.ocv));
end

function I = pick3(b), I = b(3); end
function b = swift_battery_fcn_out(T, V, rho, x, prm, ocv)
[~, b] = swift_battery_fcn(T, V, rho, x, prm, ocv);
end

function [x, t, L, Iavg, Ipk] = takeoff_leg(B, x, Vr, pitch, h, Ipk)
%  Ground roll at full throttle to Vr, then a full-throttle climb to h.
t = 0;  L = 0;  Iavg = NaN;  Vg = 0.5;  dt = 0.1;  Q = 0;
if ~B.ok
    t = 25 + h / (Vr * sind(pitch));  L = 300 + t * Vr;  return
end
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
%  Constant airspeed V, load factor n, altitude change dz over t seconds.
%  thr = estimated throttle in percent (NaN without the bench table).
Iavg = NaN;  thr = NaN;
if ~B.ok || t <= 0, return, end
if nargin < 8, idle = false; end
if idle
    T = 0;
else
    T = B.D(V, n) + B.W * (dz / t) / V;
    T = min(max(T, 0), B.Tmax(V));
end
if idle, thr = 0; else, thr = B.thr(T, V); end
Q = 0;
for kk = 1:ceil(t)
    dtk = min(1, t - (kk-1));
    I = B.I(x, T, V);  Ipk = max(Ipk, I);  Q = Q + I*dtk;
    x = B.step(x, T, V, dtk);
end
Iavg = Q / t;
end

function nz = load_factor(P, V, r)
%  Steady coordinated circle of radius r: tan(bank) = V^2 / (g r).
if isempty(P), nz = 1; return, end
nz = 1 / cos(atan(V^2 / (P.const.g * abs(r))));
end

function s = numstr(v, f)
%  Table cell text: a dash for rows with no value.
if isnan(v), s = '-'; else, s = sprintf(f, v); end
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end

function p = wsl_or_native(p)
%  Path as the ArduPilot shell sees it: Windows drive paths become WSL /mnt/<drive>/...
if numel(p) > 2 && p(2) == ':'
    p = ['/mnt/' lower(p(1)) strrep(p(3:end), '\', '/')];
end
end
