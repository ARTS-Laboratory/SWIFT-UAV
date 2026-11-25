function ok = swift_sitl_preflight(addr)
%SWIFT_SITL_PREFLIGHT  One command before every SITL session (task 1).
%
%   swift_sitl_preflight                    checks, with <step-6 address> left blank
%   swift_sitl_preflight('172.20.240.1')    same, with your Windows address filled in
%   ok = swift_sitl_preflight(...)          true when nothing blocking was found
%
%   WHAT IT DOES
%     1  swift_setup, then a FRESH P (clears any stale P from older runs)
%     2  prints the numbers this run will use: mass, stall, cruise, start
%        mode, propulsion, thrust vs drag, estimated takeoff roll
%     3  checks swift_6dof_sitl has the stage-3b patch
%     4  checks the surface signs against the aero derivatives
%     5  checks the tilt table is off (unless you meant it on)
%     6  checks UDP 9002 is free
%     7  prints the exact Ubuntu commands, in order
%
%   Ryan Bilock, ARTS Lab | SITL task 1 | v0.1

if nargin < 1 || isempty(addr), addr = '<step-6 address>'; end
ok = true;
M  = 'swift_6dof_sitl';
hr = repmat('-', 1, 66);

swift_setup;
evalin('base', 'clear P; swift_placeholder;');
evalin('base', 'P = swift_sitl_params(P);');
P = evalin('base', 'P');

fprintf('\n%s\n SWIFT SITL PREFLIGHT   %s\n%s\n', hr, datestr(now, 'dd mmm yyyy HH:MM'), hr);

% ---- 2. numbers ---------------------------------------------------------
V0 = P.init.V0;  Vs = P.perf.V_stall;
if strcmp(P.sitl.prop_used, 'test')
    Tf = @(V) P.sitl.prop_T0 * max(0, 1 - V / P.sitl.prop_Vp);
else
    Tf = @(V) P.prop.T_static_max * max(0, 1 - V / P.prop.V_pitch);
end
fprintf(' Aircraft     %.2f kg  |  stall %.1f m/s  |  cruise %g m/s\n', P.mass.m, Vs, V0);
fprintf(' Start        %s   (P.sitl.start = ''%s'')\n', upper(P.sitl.start_used), P.sitl.start);
fprintf(' Propulsion   %s   static %.1f N  |  at cruise %.1f N vs drag %.1f N\n', ...
        upper(P.sitl.prop_used), Tf(0), Tf(V0), P.trim.drag);
vr = round(1.05 * Vs);
[xr, tr, okr] = roll_estimate(P, Tf, vr);
if okr
    fprintf(' Takeoff roll about %.0f m / %.0f s to rotate at %d m/s\n', xr, tr, vr);
else
    fprintf(2, ' Takeoff roll IMPOSSIBLE: thrust runs out at %.1f m/s, rotate needs %d m/s\n', xr, vr);
    ok = false;
end
if Tf(V0) <= P.trim.drag
    fprintf(2, ' Cannot hold level flight at cruise with this propulsion.\n');
    ok = false;
end
fprintf(' Ground       %s  |  %d wheels  |  k %.0f N/m  c %.0f N/(m/s) per wheel\n', ...
        tern(P.gnd.on, 'ON', 'OFF'), size(P.gnd.wheels, 2), P.gnd.k, P.gnd.c);
fprintf(' Bridge       %g Hz (dt %.5f s)  |  channels A%d E%d R%d T%d\n', ...
        P.sitl.rate, P.sitl.dt, P.sitl.ch);

% ---- 3. model patched? ------------------------------------------------------
fprintf('%s\n', hr);
try
    load_system(M);
    x0  = get_param([M '/6DOF (Euler Angles)'], 'xme_0');
    % Found by name anywhere in the model, so grouping blocks into a
    % subsystem (as you did on 8 Nov) does not break the check.
    gb  = find_system(M, 'LookUnderMasks', 'all', 'Name', 'Ground');
    mx  = find_system(M, 'LookUnderMasks', 'all', 'Name', 'Mxyz');
    assert(~isempty(gb) && ~isempty(mx), 'Ground or Mxyz block not found anywhere in the model');
    pG  = get_param(gb{1}, 'PortHandles');
    nMx = numel(strrep(get_param(mx{1}, 'Inputs'), '|', ''));
    okIC = contains(x0, 'P.sitl.ic');
    okGd = numel(pG.Inport) == 5 && numel(pG.Outport) == 2 && nMx >= 3;
    if okIC && okGd
        fprintf(' [ OK ] %s has the stage-3b patch\n', M);
    else
        fprintf(2, ' [FAIL] %s is not patched (start state %s, wheel moments %s). Run:  swift_sitl_patch\n', ...
            M, tern(okIC, 'ok', 'missing'), tern(okGd, 'ok', 'missing'));
        ok = false;
    end
catch err
    fprintf(2, ' [FAIL] could not open %s: %s\n', M, err.message);
    ok = false;
end

% ---- 4. signs -------------------------------------------------------------------
sp = sign([P.aero.Clda, P.aero.Cmde, P.aero.Cndr]);  sp(sp == 0) = 1;
if isequal(P.sitl.sgn(:)', sp)
    fprintf(' [ OK ] surface signs [%d %d %d] match the aero derivatives\n', sp);
else
    fprintf(2, ' [WARN] P.sitl.sgn = [%d %d %d] but the aero derivatives say [%d %d %d]\n', P.sitl.sgn, sp);
    fprintf(2, '        see  swift_sitl_signcheck predict\n');
end

% ---- 5. tilt table ------------------------------------------------------------
if isfield(P.sitl, 'jig') && ~isempty(P.sitl.jig)
    fprintf(2, ' [NOTE] virtual tilt table is ARMED - nothing will take off\n');
else
    fprintf(' [ OK ] virtual tilt table off\n');
end

% ---- 6. port -------------------------------------------------------------------
clear ap_json_step
try
    u = udpport("datagram", "IPV4", "LocalPort", 9002);
    clear u
    fprintf(' [ OK ] UDP 9002 is free\n');
catch
    fprintf(2, ' [FAIL] UDP 9002 is in use. Close other MATLAB sessions, or restart MATLAB.\n');
    ok = false;
end

% ---- 7. commands -------------------------------------------------------------
%  Path of this folder's swift_sitl.param as Ubuntu (WSL) sees it - follows
%  the folder, so moving or renaming it never breaks this line.
pf = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'ardupilot', 'swift_sitl.param');   % sitl/checks -> sitl/ardupilot
pf = ['/mnt/' lower(pf(1)) strrep(pf(3:end), '\', '/')];
fprintf('%s\n RUN ORDER\n%s\n', hr, hr);
fprintf(' 1  MATLAB   open_system(''%s'')  and press Run. It waits - normal.\n', M);
fprintf(' 2  Ubuntu   cp "%s" ~/swift_sitl.param\n', pf);
fprintf('             cp "%s" ~/swift_sitl_vtail.param\n', strrep(pf, 'swift_sitl.param', 'swift_sitl_vtail.param'));
fprintf('             cd ~/ardupilot/ArduPlane\n');
fprintf(['             ../Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:%s -N -w \\\n' ...
         '               --add-param-file=../Tools/autotest/models/plane.parm \\\n' ...
         '               --add-param-file=$HOME/swift_sitl.param \\\n' ...
         '               --add-param-file=$HOME/swift_sitl_vtail.param \\\n' ...
         '               --out=udp:%s:14550 --console --map\n'], addr, addr);
fprintf('             (-w wipes saved parameters - use it the first time and whenever\n');
fprintf('              the .param files change; leave it off otherwise)\n');
fprintf(' 3  Mission Planner   UDP, port 14550, Connect\n');
fprintf(' 4  Stop: Simulink Stop first, then Ctrl+C in Ubuntu\n');
fprintf('%s\n', hr);
if ok
    fprintf(' PREFLIGHT PASSED\n\n');
else
    fprintf(2, ' PREFLIGHT FOUND PROBLEMS - fix the red lines first\n\n');
end
end

% =========================================================================
function [x, t, ok] = roll_estimate(P, Tf, vr)
%  Point-mass ground roll at alpha 0, rolling friction on the wheel load.
m = P.mass.m;  W = P.mass.W;  S = P.geom.S;  rho = P.const.rho0;
CL = P.aero.CL0;  CD = P.aero.CD0 + P.aero.k_ind * CL^2;
mu = P.gnd.mu_roll;
v = 0;  x = 0;  t = 0;  dt = 0.01;  ok = true;
while v < vr
    q = 0.5 * rho * v^2 * S;
    a = (Tf(v) - q*CD - mu * max(0, W - q*CL)) / m;
    if a <= 0 || t > 120, ok = false; x = v; return, end
    v = v + a*dt;  x = x + v*dt;  t = t + dt;
end
end

function s = tern(c, a, b), if c, s = a; else, s = b; end, end
