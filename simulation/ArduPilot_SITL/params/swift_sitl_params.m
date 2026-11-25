function P = swift_sitl_params(P)
%SWIFT_SITL_PARAMS  SITL-bridge, ground-gear and test settings for swift_6dof_sitl.
%
%   P = swift_sitl_params(P)
%
%   Adds P.sitl and P.gnd. swift_6dof_sitl's InitFcn calls it on every run,
%   straight after making sure P exists.
%
%   TWO KINDS OF FIELD
%     Settings (section 1) are only filled in when missing, so a field you
%     set at the command line survives the next run:
%         P.sitl.use_ap = 0;      % fly on the cmd_* Constants, no UDP at all
%         P.gnd.on      = 0;      % no ground contact
%         P.sitl.start  = 'air';  % start in the air even with ArduPlane
%     Re-running swift_placeholder rebuilds P from scratch, which puts every
%     default below back. Edit HERE to change a default permanently.
%
%     Derived fields (section 2) are rebuilt on EVERY call from P.init,
%     P.prop and P.env, so they always match the settings:
%         P.sitl.ic         initial state the 6DOF block starts from
%         P.sitl.wind_ned   wind the model flies in
%         P.sitl.thrust_map thrust table the Thrust block reads
%         P.gnd.vec         15-element vector the Ground block reads
%     swift_sitl_patch points those four blocks at these fields. Nothing
%     here touches P.init, P.prop or P.env, so swift_6dof (the standalone
%     model) is never affected and "swift_sitl_build verify" stays valid.
%
%   P.sitl  (settings)
%     rate       Hz    frame rate ArduPlane sends (the bridge prints it)
%     dt         s     1/rate - the model's fixed step. Always recomputed.
%     use_ap     -     1 = ArduPlane flies it, 0 = the cmd_* Constants
%     ch         -     servo channel for [aileron elevator rudder throttle]
%     sgn        -     +1/-1 per surface [aileron elevator rudder]
%     log_decim  -     log every Nth step (12 at 1200 Hz = 100 Hz)
%     start      -     'auto' | 'ground' | 'air'. auto = ground when
%                      use_ap = 1 (takeoff), air otherwise (verify)
%     prop       -     'auto' | 'test' | 'sheet'. auto = sheet (real MAD
%                      X7215 bench data, v0.5); 'test' = old stand-in
%     prop_T0    N     SITL-test static thrust            [SITL-TEST]
%     prop_Vp    m/s   SITL-test pitch speed              [SITL-TEST]
%     calm       m/s   headwind floor while parked, keeps airspeed > 0
%     hdg0       deg   true heading the aircraft is parked on (runway
%                      heading). 0 = north. swift_sitl_cub sets 125.55
%                      for Owens Downtown runway 13.
%     jig        -     virtual tilt-table schedule, [] = off. Set by
%                      swift_sitl_signcheck('jig'); read by ap_json_step
%     send_batt  -     1 = send the Battery subsystem's voltage and current
%                      to ArduPlane (needs swift_battery_add), 0 = don't
%
%   P.gnd  (settings) - three-wheel tricycle spring-damper with friction
%     on         -        1 = ground contact active
%     k          N/m      per-wheel stiffness, 50 x weight / 3
%     c          N/(m/s)  per-wheel damping, 1.5 x critical
%     mu_roll    -        rolling resistance (paved 0.02-0.04, grass 0.08)
%     mu_lat     -        sideways tyre grip
%     v_eps      m/s      friction smoothing speed (no stick-slip chatter)
%     wheels     m        3x3 contact points in body axes [nose main-R main-L]
%                         x fwd, y right, z down, measured from the CG
%
%   PROPULSION (v0.5)
%     swift_placeholder v0.6 now holds the real motor: MAD X7215 IPE 220KV,
%     12S, 21x10 prop, 145 N static thrust, 32.9 m/s pitch speed, from the
%     MAD bench table. 'auto' uses it. The old test stand-in (90 N, 35 m/s)
%     is kept only as prop = 'test' for comparing against earlier runs
%     (Session C used it).
%
%   Tags:  [OPS] test knob   [EST] estimate   [EST->CAD] from CAD
%          [SITL-TEST] stand-in so the autopilot can be tested
%
%   Ryan Bilock, ARTS Lab | SITL guide steps 9-18 + stage 3b | v0.5
%     v0.5  prop 'auto' = real MAD X7215 data (sheet); test set kept as an option
%     v0.4  send_batt: battery voltage/current to ArduPlane on or off
%     v0.3  hdg0: park on any runway heading (headwind floor follows it)
%     v0.2  ground start, three-wheel gear with friction, test propulsion,
%           parked headwind floor, virtual tilt-table hook. Derived fields
%           are rebuilt every call; P.init/P.prop/P.env are never edited.
%     v0.1  bridge settings and single-point ground spring

%% 1 | Settings - filled in only when missing ------------------------------
W = P.mass.m * P.const.g;                           % N

d.sitl.rate      = 1200;            % Hz   from the bridge's printout     [OPS]
d.sitl.use_ap    = 1;               % -    1 = ArduPlane in control       [OPS]
d.sitl.ch        = [1 2 4 3];       % -    ail, elev, rud, thr channels   [OPS]
d.sitl.sgn       = [1 -1 -1];       % -    ail, elev, rud sign            [EST]
                                    %      from the aero signs: Clda > 0,
                                    %      Cmde < 0, Cndr < 0. Confirm with
                                    %      swift_sitl_signcheck.
d.sitl.log_decim = 12;              % -    1200/12 = 100 Hz logging       [OPS]
d.sitl.start     = 'auto';          % -    see header                     [OPS]
d.sitl.prop      = 'auto';          % -    see header                     [OPS]
d.sitl.prop_T0   = 90;              % N    static thrust            [SITL-TEST]
d.sitl.prop_Vp   = 35;              % m/s  pitch speed              [SITL-TEST]
d.sitl.calm      = 0.5;             % m/s  parked headwind floor          [OPS]
d.sitl.jig       = [];              % -    off                            [OPS]
d.sitl.hdg0      = 0;               % deg  parked true heading, 0 = north [OPS]
d.sitl.send_batt = 1;               % -    battery -> ArduPlane           [OPS]

d.gnd.on      = 1;                                  % -               [OPS]
d.gnd.k       = 50 * W / 3;                         % N/m per wheel   [EST]
d.gnd.c       = 1.5 * 2*sqrt(d.gnd.k * P.mass.m/3); % N/(m/s)/wheel   [EST]
d.gnd.mu_roll = 0.04;                               % -               [EST]
d.gnd.mu_lat  = 0.50;                               % -               [EST]
d.gnd.v_eps   = 0.20;                               % m/s             [OPS]
%                 nose    main R   main L
d.gnd.wheels  = [ 0.40   -0.12    -0.12 ;   % x  m  fwd of CG      [EST->CAD]
                  0.00    0.35    -0.35 ;   % y  m  right          [EST->CAD]
                  0.30    0.30     0.30 ];  % z  m  below CG       [EST->CAD]
%  z = 230 mm gear (sheet) + about 70 mm from the fuselage floor to the CG.

for grp = fieldnames(d)'
    g = grp{1};
    if ~isfield(P, g), P.(g) = struct(); end
    for f = fieldnames(d.(g))'
        if ~isfield(P.(g), f{1}), P.(g).(f{1}) = d.(g).(f{1}); end
    end
end

%% 2 | Derived - rebuilt on every call -------------------------------------
P.sitl.dt = 1 / P.sitl.rate;        % s   always follows rate

% ---- start mode -----------------------------------------------------------
st = lower(char(P.sitl.start));
if strcmp(st, 'auto')
    if P.sitl.use_ap > 0.5, st = 'ground'; else, st = 'air'; end
end
P.sitl.start_used = st;

if strcmp(st, 'ground')
    % Parked on the runway, level, nose on hdg0, wheels just touching.
    psi0 = mod(P.sitl.hdg0, 360) * pi/180;
    P.sitl.ic.pos0   = [0; 0; -max(P.gnd.wheels(3,:))];
    P.sitl.ic.uvw0   = [0; 0; 0];
    P.sitl.ic.euler0 = [0; 0; psi0];
    P.sitl.ic.om0    = [0; 0; 0];
    % Air moving against the nose at "calm" m/s = a light headwind, so
    % airspeed is never exactly zero (sideslip = asin(v/V) needs V > 0).
    P.sitl.wind_ned  = P.env.wind_ned - P.sitl.calm * [cos(psi0); sin(psi0); 0];
else
    P.sitl.ic.pos0   = P.init.pos0;
    P.sitl.ic.uvw0   = P.init.uvw0;
    P.sitl.ic.euler0 = P.init.euler0;
    P.sitl.ic.om0    = P.init.om0;
    P.sitl.wind_ned  = P.env.wind_ned;
end

% ---- propulsion -----------------------------------------------------------
pm = lower(char(P.sitl.prop));
if strcmp(pm, 'auto')
    pm = 'sheet';                    % real MAD X7215 bench data (v0.5)
end
P.sitl.prop_used = pm;

if strcmp(pm, 'test')
    P.sitl.thrust_map = P.prop.throttle_bp' * ...
        (P.sitl.prop_T0 * max(0, 1 - P.prop.V_bp / P.sitl.prop_Vp));
    fprintf(2, ['[swift_sitl_params] SITL-TEST propulsion: %g N static, %g m/s ' ...
                'pitch speed. Old stand-in, not the aircraft (use prop = ''auto'').\n'], ...
                P.sitl.prop_T0, P.sitl.prop_Vp);
else
    P.sitl.thrust_map = P.prop.thrust_map;
    fprintf('[swift_sitl_params] Propulsion: %s, %.0f N static, %.1f m/s pitch speed.\n', ...
            P.motor.model, P.prop.T_static_max, P.prop.V_pitch);
end

% ---- ground vector for the Ground block ---------------------------------
%  [on; k; c; mu_roll; mu_lat; v_eps; wheels(:)]  -> 15 x 1
P.gnd.vec = [P.gnd.on; P.gnd.k; P.gnd.c; P.gnd.mu_roll; P.gnd.mu_lat; ...
             P.gnd.v_eps; reshape(P.gnd.wheels, 9, 1)];
end
