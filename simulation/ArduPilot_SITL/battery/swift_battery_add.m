function swift_battery_add(target, opt)
%SWIFT_BATTERY_ADD  Add the Battery subsystem to swift_6dof_sitl (ArduPilot_SITL folder).
%
%   swift_battery_add                     swift_6dof_sitl (backs it up first)
%   swift_battery_add swift_6dof_sitl     same
%   swift_battery_add params              rewrite sitl/ardupilot/swift_sitl_batt.param only
%
%   This is the ArduPilot_SITL copy: it only ever touches swift_6dof_sitl.
%   The Standalone folder has its own copy for swift_6dof.
%
%   Safe to run again: an existing Battery subsystem, its logger and its
%   bridge link are removed and rebuilt, so the result is always the same.
%
%   WHAT IT ADDS (at the bottom of the root diagram)
%     Battery          subsystem. Inputs: thrust (Thrust table), airspeed
%                      (Incidence block port 3), air density (COESA port 4).
%                      Inside: two Constants (P.batt.prm, P.batt.ocv), the
%                      "Battery model" MATLAB Function (calls
%                      swift_battery_fcn.m), and one 3-state Integrator
%                      [SOC; mAh used; Wh used] started at P.batt.x0.
%     out_batt         To Workspace, 10 signals (see swift_battery_fcn.m).
%                      Plot with swift_battery_plot.
%     batt VI term     Terminator on the [V; I] port (standalone model only).
%   SITL MODEL ONLY
%     batt z^-1        Unit Delay on [V_pack; I], one step. Without it the
%                      bridge -> PWM -> throttle -> thrust -> battery -> bridge
%                      path is an algebraic loop.
%     SITL state       the bridge Mux (root diagram or one subsystem down;
%                      if it is in a subsystem, a "batt VI" input port is
%                      added to that subsystem) grows from 7 to 8 inputs, so
%                      ap_json_step sends "battery":{"voltage","current"}
%                      to ArduPlane each frame (switch: P.sitl.send_batt).
%     swift_sitl_batt.param   BATT_* settings so ArduPlane reads that battery
%                      and its low/critical failsafes act on it.
%
%   NEEDS, ON THE PATH (run swift_setup first)
%     swift_placeholder.m v0.5 or later (P.batt.prm, P.batt.ocv, P.batt.x0),
%     swift_battery_fcn.m, and for SITL: swift_sitl_params.m, ap_json_step.m v0.3
%
%   Each model is backed up to archive/pre_battery_<date_time>/ before it is
%   opened for writing. swift_sitl_build calls this with 'nobackup' at the
%   end of a rebuild, because the build already made its own backup.
%
%   Ryan Bilock, ARTS Lab | battery model | v0.3
%     v0.3  ArduPilot_SITL copy - swift_6dof_sitl only (folders split 10 Nov 2026)
%     v0.2  finds the SITL state Mux at any depth (it can live inside a
%           grouped subsystem) and routes [V; I] in through a new port

if nargin < 1 || isempty(target), target = 'swift_6dof_sitl'; end
if nargin < 2, opt = ''; end
target = char(target);
backup = ~strcmpi(char(opt), 'nobackup');

if strcmpi(target, 'params')
    fresh_P(true);
    write_ap_params();
    return
end
if ~any(strcmpi(target, {'swift_6dof_sitl', 'both'}))
    error('swift_battery_add:folder', ['This is the ArduPilot_SITL copy - it only adds the battery ' ...
          'to swift_6dof_sitl. For %s use the Standalone folder (run its swift_setup).'], target);
end
models = {'swift_6dof_sitl'};

need = {'swift_placeholder.m', 'swift_battery_fcn.m'};
for k = 1:numel(need)
    if isempty(which(need{k}))
        error('swift_battery_add:missing', '%s is not on the MATLAB path. Run swift_setup first.', need{k});
    end
end

for k = 1:numel(models)
    add_to(models{k}, backup);
end
fprintf(['\nNext:\n' ...
    '  1. Run swift_6dof_sitl (with ArduPlane), then:  swift_battery_plot\n' ...
    '  2. SITL: start ArduPlane with the battery parameters added at boot:\n' ...
    '       sim_vehicle.py ... --add-param-file="<path>/sitl/ardupilot/swift_sitl_batt.param"\n' ...
    '     then check the Mission Planner HUD voltage drops when you add throttle.\n\n']);
end

% =========================================================================
function add_to(M, backup)
file = which([M '.slx']);
if isempty(file)
    error('swift_battery_add:model', '%s.slx is not on the path. Run swift_setup first.', M);
end
if bdIsLoaded(M) && strcmp(get_param(M, 'Dirty'), 'on')
    error('swift_battery_add:dirty', ...
        '%s has unsaved changes. Save it (Ctrl+S) or close it without saving, then run again.', M);
end

isSITL = ~isempty(regexp(M, '_sitl$', 'once'));
P = fresh_P(isSITL);

if backup
    root = fileparts(fileparts(file));               % this setup's folder (backups go to its archive\)
    bdir = fullfile(root, 'archive', ['pre_battery_' datestr(now, 'yyyy-mm-dd_HHMMSS')]);
    if ~isfolder(bdir), mkdir(bdir); end
    if bdIsLoaded(M), close_system(M, 0); end
    copyfile(file, bdir);
    fprintf('\nBacked up %s.slx to %s\n', M, bdir);
end

load_system(M);
fprintf('Adding the battery to %s ...\n', M);

% --- start clean ---------------------------------------------------------
kill([M '/Battery']);
kill([M '/out_batt']);
kill([M '/batt z^-1']);
kill([M '/batt VI term']);

% --- the three signals the battery needs ----------------------------------
TH  = [M '/Thrust'];
need_block(TH);
inc = find_system(M, 'SearchDepth', 1, 'RegExp', 'on', 'Name', '^Incidence');
atm = find_system(M, 'SearchDepth', 1, 'RegExp', 'on', 'Name', '^COESA');
assert(~isempty(inc), 'swift_battery_add: Incidence, Sideslip & Airspeed block not found in %s.', M);
assert(~isempty(atm), 'swift_battery_add: COESA Atmosphere Model block not found in %s.', M);
pT   = outport(TH, 1);              % N
pV   = outport(inc{1}, 3);          % m/s  raw airspeed
pRho = outport(atm{1}, 4);          % kg/m^3  (COESA outputs T, a, P, rho)

% --- the subsystem -------------------------------------------------------
y0 = max_y(M) + 90;
B  = add([M '/Battery'], 'built-in/Subsystem', [520 y0 660 y0+130]);
build_battery([M '/Battery']);
set_param(B, 'BackgroundColor', 'yellow');

tw0 = find_system(M, 'BlockType', 'ToWorkspace');
fmt = 'Timeseries';
if ~isempty(tw0), fmt = get_param(tw0{1}, 'SaveFormat'); end
L = add([M '/out_batt'], 'simulink/Sinks/To Workspace', [740 y0+20 830 y0+50], ...
        'VariableName', 'out_batt', 'SaveFormat', fmt, 'MaxDataPoints', 'inf');
if isSITL
    set_param(L, 'Decimation', 'P.sitl.log_decim');
end

wire(M, pT,   inport(B, 1));
wire(M, pV,   inport(B, 2));
wire(M, pRho, inport(B, 3));
wire(M, outport(B, 1), inport(L, 1));
name_line(outport(B, 1), 'batt');
say('Battery', 'subsystem in: Thrust, airspeed (port 3), rho (COESA port 4)');
say('Logger ', 'out_batt (%s): SOC, V, I, P, mAh, Wh, V_cell, P_shaft, motor load, status', fmt);

% --- SITL: hand [V; I] to the ArduPilot bridge ------------------------------
%  The bridge Mux may sit on the root diagram (as swift_sitl_build makes it)
%  or inside a subsystem it was grouped into. Either way, find it.
mx = find_system(M, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                 'BlockType', 'Mux', 'Name', 'SITL state');
if ~isempty(mx)
    MUX = mx{1};
    S   = get_param(MUX, 'Parent');                  % where the bridge lives
    if ~strcmp(S, M) && ~strcmp(get_param(S, 'Parent'), M)
        error('swift_battery_add:depth', ...
            'The SITL state Mux is more than one subsystem deep (%s). Move it up one level.', S);
    end
    kill([S '/batt z^-1']);
    kill([S '/batt VI']);
    n = str2double(get_param(MUX, 'Inputs'));
    if n < 8, set_param(MUX, 'Inputs', '8'); end
    pm  = get_param(MUX, 'PortHandles');
    old = get_param(pm.Inport(8), 'Line');
    if old ~= -1, delete_line(old); end

    mp = get_param(MUX, 'Position');                 % park the delay just below the Mux
    D  = add([S '/batt z^-1'], 'simulink/Discrete/Unit Delay', ...
             [mp(1)-110 mp(4)+30 mp(1)-70 mp(4)+60], 'SampleTime', 'P.sitl.dt');
    ic = '[P.batt.cells*interp1(P.batt.ocv(1,:), P.batt.ocv(2,:), P.batt.SOC0); 0]';
    try
        set_param(D, 'InitialCondition', ic);
    catch
        set_param(D, 'X0', ic);
    end

    if strcmp(S, M)                                  % bridge on the root diagram
        wire(M, outport(B, 2), inport(D, 1));
        where = 'root diagram';
    else                                             % bridge inside a subsystem
        Ip = add([S '/batt VI'], 'simulink/Sources/In1', ...
                 [mp(1)-200 mp(4)+38 mp(1)-170 mp(4)+52]);
        wire(S, outport(Ip, 1), inport(D, 1));
        ps = get_param(S, 'PortHandles');
        wire(M, outport(B, 2), ps.Inport(end));      % the new port is the last one
        where = sprintf('inside "%s" (new input port %d, "batt VI")', ...
                        get_param(S, 'Name'), numel(ps.Inport));
    end
    wire(S, outport(D, 1), pm.Inport(8));
    name_line(outport(B, 2), 'VI');
    say('Bridge ', 'SITL state Mux %s -> 8 inputs; [V_pack; I] through batt z^-1 (P.sitl.dt)', where);
    write_ap_params();
else
    if isSITL
        warning('swift_battery_add:nobridge', ...
            'No "SITL state" Mux in %s - the battery is logged but not sent to ArduPlane.', M);
    end
    Tm = add([M '/batt VI term'], 'simulink/Sinks/Terminator', [740 y0+85 760 y0+105]);
    wire(M, outport(B, 2), inport(Tm, 1));
    say('VI     ', 'no ArduPilot bridge in this model - [V; I] capped with a Terminator');
end

save_system(M);
compile_check(M);
end

% =========================================================================
function build_battery(S)
add([S '/T'],   'simulink/Sources/In1', [30  48  60  62]);
add([S '/V'],   'simulink/Sources/In1', [30  98  60 112]);
add([S '/rho'], 'simulink/Sources/In1', [30 148  60 162]);
add([S '/batt prm'],  'simulink/Sources/Constant', [20 200 120 230], 'Value', 'P.batt.prm');
add([S '/OCV curve'], 'simulink/Sources/Constant', [20 260 120 290], 'Value', 'P.batt.ocv');

F = add([S '/Battery model'], 'simulink/User-Defined Functions/MATLAB Function', ...
        [230 30 390 310]);
set_script(F, fcn_code());

I = add([S '/SOC | mAh | Wh'], 'simulink/Continuous/Integrator', [470 60 520 110], ...
        'InitialCondition', 'P.batt.x0', 'LimitOutput', 'on', ...
        'UpperSaturationLimit', '[1; inf; inf]', 'LowerSaturationLimit', '[0; 0; 0]');
add([S '/batt'], 'simulink/Sinks/Out1', [600 158 630 172]);
add([S '/VI'],   'simulink/Sinks/Out1', [600 248 630 262]);

wire(S, outport([S '/T'],   1), inport(F, 1));
wire(S, outport([S '/V'],   1), inport(F, 2));
wire(S, outport([S '/rho'], 1), inport(F, 3));
wire(S, outport(I, 1),          inport(F, 4));
wire(S, outport([S '/batt prm'],  1), inport(F, 5));
wire(S, outport([S '/OCV curve'], 1), inport(F, 6));
wire(S, outport(F, 1), inport(I, 1));
wire(S, outport(F, 2), inport([S '/batt'], 1));
wire(S, outport(F, 3), inport([S '/VI'], 1));
name_line(outport(F, 1), 'xdot');
name_line(outport(I, 1), 'x');
end

function c = fcn_code()
c = sprintf([ ...
'function [xdot, batt, VI] = battery(T, V, rho, x, prm, ocv)\n' ...
'%%BATTERY  SWIFT battery model. All the physics is in swift_battery_fcn.m.\n' ...
'%%   T N, V m/s, rho kg/m^3, x = [SOC; mAh; Wh], prm = P.batt.prm, ocv = P.batt.ocv\n' ...
'[xdot, batt, VI] = swift_battery_fcn(T, V, rho, x, prm, ocv);\n']);
end

% =========================================================================
function P = fresh_P(isSITL)
evalin('base', 'swift_placeholder;');
if isSITL
    evalin('base', 'P = swift_sitl_params(P);');
end
P = evalin('base', 'P');
if ~isfield(P, 'batt') || ~isfield(P.batt, 'prm') || ~isfield(P.batt, 'ocv')
    error('swift_battery_add:params', ...
        'P.batt.prm / P.batt.ocv missing. swift_placeholder.m must be v0.5 or later.');
end
end

function write_ap_params()
P = evalin('base', 'P');
c = P.batt.cells;
prm = { ...
  'BATT_MONITOR',     4,                        'analog monitor - the one SITL feeds (needs a reboot if changed)';
  'BATT_CAPACITY',    round(1000*P.batt.Ah),    'mAh, P.batt.Ah';
  'BATT_LOW_VOLT',    round(10*c*P.batt.Vcell_low)/10,  'V, P.batt.Vcell_low x cells';
  'BATT_CRT_VOLT',    round(10*c*P.batt.Vcell_crit)/10, 'V, P.batt.Vcell_crit x cells';
  'BATT_LOW_MAH',     round(1000*P.batt.Ah*P.batt.SOC_res),   'mAh left at the reserve line (P.batt.SOC_res)';
  'BATT_CRT_MAH',     round(1000*P.batt.Ah*P.batt.SOC_res/2), 'mAh, half the reserve';
  'BATT_FS_VOLTSRC',  1,                        'judge voltage on the sag-compensated (resting) estimate';
  'BATT_FS_LOW_ACT',  1,                        'low battery -> RTL';
  'BATT_FS_CRT_ACT',  1,                        'critical battery -> RTL (RTL_AUTOLAND=2 lands it)';
  'SIM_BATT_VOLTAGE', round(10*c*4.2)/10,          'V fallback if this SITL build ignores the JSON battery';
  'SIM_BATT_CAP_AH',  P.batt.Ah,                'Ah, same pack for the fallback'};

folder = fullfile(fileparts(fileparts(which('ap_json_step'))), 'ardupilot');   % sitl/bridge -> sitl/ardupilot
if isempty(which('ap_json_step')), folder = ''; end
if isempty(folder), folder = fileparts(mfilename('fullpath')); end
f = fullfile(folder, 'swift_sitl_batt.param');
fid = fopen(f, 'w');
assert(fid > 0, 'Could not write %s', f);
fprintf(fid, '# SWIFT UAV - ArduPlane SITL battery parameters\n');
fprintf(fid, '# Written by swift_battery_add, %s\n', datestr(now, 'yyyy-mm-dd HH:MM'));
fprintf(fid, '# %dS %.0f Ah LiPo. Load LAST, at boot: sim_vehicle.py ... --add-param-file=<this file>\n', ...
        c, P.batt.Ah);
for k = 1:size(prm, 1)
    fprintf(fid, '# %s\n%s,%g\n', prm{k,3}, prm{k,1}, prm{k,2});
end
fclose(fid);
say('Params ', 'wrote %s', f);
end

% =========================================================================
function compile_check(M)
% swift_6dof's unfinished Stateflow Chart will not compile. Comment it out
% for the check only, then put it back exactly as it was (nothing saved).
ch = [M '/Chart'];
restore = '';
if getSimulinkBlockHandle(ch) ~= -1 && strcmp(get_param(ch, 'Commented'), 'off')
    set_param(ch, 'Commented', 'on');
    restore = 'off';
end
try
    set_param(M, 'SimulationCommand', 'update');
    say('Check  ', 'Update Diagram (Ctrl+D) passed');
catch err
    fprintf(2, '\nUpdate Diagram failed - the model is saved, fix this and press Ctrl+D:\n%s\n', ...
        err.getReport('basic'));
end
if ~isempty(restore)
    set_param(ch, 'Commented', restore);
    set_param(M, 'Dirty', 'off');
end
end

% =========================================================================
%  SMALL HELPERS (same as swift_sitl_build)
% =========================================================================
function h = add(path, lib, pos, varargin)
h = add_block(lib, path, 'Position', pos, varargin{:});
end

function wire(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'on');
end

function p = outport(blk, k)
ph = get_param(blk, 'PortHandles');  p = ph.Outport(k);
end

function p = inport(blk, k)
ph = get_param(blk, 'PortHandles');  p = ph.Inport(k);
end

function name_line(srcPort, nm)
l = get_param(srcPort, 'Line');
if l ~= -1 && isempty(get_param(l, 'Name')), set_param(l, 'Name', nm); end
end

function kill(blk)
if getSimulinkBlockHandle(blk) == -1, return, end
lh = get_param(blk, 'LineHandles');
for f = {'Inport', 'Outport'}
    h = lh.(f{1});
    for k = 1:numel(h)
        if h(k) ~= -1 && ishandle(h(k)), delete_line(h(k)); end
    end
end
delete_block(blk);
end

function y = max_y(M)
b = find_system(M, 'SearchDepth', 1, 'Type', 'block');
y = 0;
for k = 1:numel(b)
    pos = get_param(b{k}, 'Position');
    y = max(y, pos(4));
end
end

function need_block(p)
if getSimulinkBlockHandle(p) == -1
    error('swift_battery_add:block', ...
        'Expected block not found: %s\nThe model has changed since this script was written.', p);
end
end

function set_script(blk, code)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', getfullname(blk));
assert(~isempty(ch), 'Could not open the MATLAB Function block %s', getfullname(blk));
ch.Script = code;
end

function say(step, fmt, varargin)
fprintf(['  %s  ' fmt '\n'], step, varargin{:});
end
