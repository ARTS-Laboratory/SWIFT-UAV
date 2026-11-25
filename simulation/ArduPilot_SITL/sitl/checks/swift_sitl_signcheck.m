function swift_sitl_signcheck(action, out)
%SWIFT_SITL_SIGNCHECK  Prove each control surface moves the right way (task 2).
%
%   swift_sitl_signcheck            same as 'predict'
%   swift_sitl_signcheck predict    what P.sitl.sgn SHOULD be, from the aero
%                                   derivatives and ArduPilot's conventions
%   swift_sitl_signcheck jig        arm the virtual tilt table for the next run
%   swift_sitl_signcheck jigoff     disarm it
%   swift_sitl_signcheck report     grade the last run (reads 'out' from the
%   swift_sitl_signcheck('report', out)            base workspace)
%
%   THE CONVENTION BEING CHECKED
%     ArduPlane, with SERVOn_REVERSED = 0, raises the servo PWM to
%       roll right (aileron), pitch nose UP (elevator), yaw nose RIGHT (rudder).
%     In the model,  delta = sgn * d_max * (pwm - 1500)/500,  and the moment
%     a deflection makes is  Cl = Clda*da,  Cm = Cmde*de,  Cn = Cndr*dr.
%     So a higher PWM gives the right moment only if
%       sgn(1) = sign(Clda),  sgn(2) = sign(Cmde),  sgn(3) = sign(Cndr).
%
%   TWO TESTS, ONE RUN  (all on the runway, nothing flies)
%     A  Virtual tilt table, FBWA mode. ArduPlane is told the aircraft is
%        rolled/pitched; it must deflect the surfaces to level it. This also
%        proves the attitude/gyro/accelerometer signs the bridge sends.
%     B  Stick test, MANUAL mode. You push each stick to 1900 us at the
%        MAVProxy prompt; each must make a positive (right / up / right)
%        moment.
%
%   Ryan Bilock, ARTS Lab | SITL task 2 | v0.1

if nargin < 1, action = 'predict'; end
switch lower(char(action))
    case 'predict', predict();
    case 'jig',     set_jig(true);
    case 'jigoff',  set_jig(false);
    case 'report'
        if nargin < 2, out = []; end
        report(out);
    otherwise
        error('swift_sitl_signcheck:action', 'Use predict, jig, jigoff or report.');
end
end

% =========================================================================
function P = getP()
if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
evalin('base', 'P = swift_sitl_params(P);');
P = evalin('base', 'P');
end

function sp = predicted(P)
sp = sign([P.aero.Clda, P.aero.Cmde, P.aero.Cndr]);
sp(sp == 0) = 1;
end

% =========================================================================
function predict()
P  = getP();
sp = predicted(P);
nm = {'aileron', 'elevator', 'rudder'};
dv = [P.aero.Clda, P.aero.Cmde, P.aero.Cndr];
dn = {'Clda', 'Cmde', 'Cndr'};
fprintf('\nSurface sign prediction (ArduPlane PWM up = roll R / nose up / yaw R)\n');
fprintf('  %-9s %-5s %8s  %10s  %10s  %s\n', 'surface', 'ch', 'deriv', 'predicted', 'P.sitl.sgn', 'verdict');
for k = 1:3
    v = 'OK';
    if P.sitl.sgn(k) ~= sp(k), v = 'FLIP'; end
    fprintf('  %-9s %-5d %4s %+5.2f  %10d  %10d  %s\n', nm{k}, P.sitl.ch(k), dn{k}, dv(k), sp(k), P.sitl.sgn(k), v);
end
if any(P.sitl.sgn(:)' ~= sp)
    fprintf(2, '\n  Set  d.sitl.sgn = [%d %d %d];  in swift_sitl_params.m, then  clear P\n', sp);
end
fprintf('  Throttle on channel %d. Prove it on the runway: swift_sitl_signcheck jig\n\n', P.sitl.ch(4));
end

% =========================================================================
function J = jig_table()
%  [t_start t_end roll_deg pitch_deg]. The first 60 s give the EKF time to
%  settle and you time to type  mode FBWA .
J = [ 60  75   20    0 ;
      85 100  -20    0 ;
     110 125    0   15 ;
     135 150    0  -10 ];
end

function set_jig(on)
getP();
if on
    J = jig_table();
    assignin('base', 'SWIFT_JIG_TMP', J);
    evalin('base', 'P.sitl.jig = SWIFT_JIG_TMP; clear SWIFT_JIG_TMP');
    fprintf(['\nVirtual tilt table ARMED for the next run (sim time):\n' ...
             '    0-60 s   level   -> type  mode FBWA  at the MAVProxy prompt\n']);
    for k = 1:size(J, 1)
        fprintf('  %3d-%3d s  roll %+3d  pitch %+3d deg\n', J(k,1), J(k,2), J(k,3), J(k,4));
    end
    fprintf(['  after %d s  real state again. Then the stick test (MANUAL):\n' ...
             '      mode MANUAL\n' ...
             '      rc 1 1900   (wait 5 s)   rc 1 1500\n' ...
             '      rc 2 1900   (wait 5 s)   rc 2 1500\n' ...
             '      rc 4 1900   (wait 5 s)   rc 4 1500\n' ...
             '  Stop Simulink, then:  swift_sitl_signcheck report\n\n'], max(J(:,2)));
else
    evalin('base', 'P.sitl.jig = [];');
    fprintf('Virtual tilt table OFF.\n');
end
end

% =========================================================================
function report(out)
P  = evalin('base', 'P');
if isempty(out) && evalin('base', 'exist(''out'',''var'')'), out = evalin('base', 'out'); end
[t, d] = sig(out, 'out_delta');                  % [da de dr dT], rad / 0-1
if isempty(t)
    error('swift_sitl_signcheck:nolog', 'No out_delta found. Run the model first (it logs to ''out'').');
end
dmax = [P.act.da_max, P.act.de_max, P.act.dr_max];
cder = [P.aero.Clda,  P.aero.Cmde,  P.aero.Cndr];
nm   = {'aileron', 'elevator', 'rudder'};
mom  = {'roll', 'pitch', 'yaw'};
pass = true;

% ---- A: virtual tilt table ----------------------------------------------
J = [];
if isfield(P.sitl, 'jig'), J = P.sitl.jig; end
tEndJig = 0;
fprintf('\nA  Virtual tilt table (FBWA must push back toward level)\n');
if isempty(J)
    fprintf('   not run (swift_sitl_signcheck jig before the run to include it)\n');
else
    tEndJig = max(J(:,2));
    for k = 1:size(J, 1)
        w = t >= J(k,1) + 4 & t <= J(k,2) - 2;          % plateau only
        if ~any(w)
            fprintf('   %3d-%3d s  no data (run was shorter)\n', J(k,1), J(k,2)); pass = false; continue
        end
        if J(k,3) ~= 0, ax = 1; tilt = J(k,3); else, ax = 2; tilt = J(k,4); end
        dm = mean(d(w, ax));
        M  = cder(ax) * dm;                              % moment coefficient
        ok = abs(dm) > deg2rad(0.5) && sign(M) == -sign(tilt);
        if abs(dm) <= deg2rad(0.5)
            v = 'NO MOVE - was FBWA on?';
        elseif ok
            v = 'PASS';
        else
            v = 'FAIL - reversed';
        end
        pass = pass && ok;
        fprintf('   %s %+3d deg  %-8s %+6.1f deg  -> %s moment %+.3f  %s\n', ...
            mom{ax}, tilt, nm{ax}, rad2deg(dm), mom{ax}, M, v);
    end
end

% ---- B: stick test ----------------------------------------------------------
fprintf('\nB  Stick test (MANUAL, each stick to 1900 us must give + moment)\n');
after = t > tEndJig + 1;
for ax = 1:3
    dd = d(after, ax);
    if isempty(dd) || max(abs(dd)) < 0.4 * dmax(ax)
        fprintf('   %-8s  no stick input found after %g s\n', nm{ax}, tEndJig);
        continue
    end
    [~, i] = max(abs(dd));
    M  = cder(ax) * dd(i);
    ok = M > 0;
    pass = pass && ok;
    if ok, v = 'PASS'; else, v = 'FAIL - reversed'; end
    fprintf('   %-8s  %+6.1f deg  -> %s moment %+.3f  %s\n', nm{ax}, rad2deg(dd(i)), mom{ax}, M, v);
end

% ---- throttle --------------------------------------------------------------
fprintf('\n   throttle  max %.2f (0-1)  %s\n', max(d(:,4)), ...
    tern(max(d(:,4)) > 0.05, 'moved', 'never moved (normal if not armed)'));

sp = predicted(P);
fprintf('\n   P.sitl.sgn = [%d %d %d]   predicted [%d %d %d]\n', P.sitl.sgn, sp);
if pass
    fprintf('   RESULT: all tested surfaces move the right way.\n\n');
else
    fprintf(2, ['   RESULT: at least one FAIL. Flip that surface in d.sitl.sgn\n' ...
                '   (swift_sitl_params.m), clear P, and run the check again.\n' ...
                '   If tilt-table pitch FAILS but stick pitch PASSES (or the reverse),\n' ...
                '   the attitude sign the bridge sends is wrong, not P.sitl.sgn.\n\n']);
end
end

% =========================================================================
function [t, y] = sig(o, name)
t = [];  y = [];
s = [];
if isa(o, 'Simulink.SimulationOutput')
    try, s = o.get(name); catch, s = []; end
end
if isempty(s) && evalin('base', sprintf('exist(''%s'',''var'')', name))
    s = evalin('base', name);
end
if isempty(s), return, end
if isa(s, 'timeseries')
    t = s.Time;  y = squeeze(s.Data);
elseif isstruct(s)
    t = s.time;  y = squeeze(s.signals.values);
else
    y = s;  t = (0:size(y,1)-1)' * 0.01;
end
if size(y, 1) ~= numel(t), y = y.'; end
end

function s = tern(c, a, b), if c, s = a; else, s = b; end, end
