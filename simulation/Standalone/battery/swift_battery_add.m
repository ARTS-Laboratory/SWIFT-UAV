function swift_battery_add(target, opt)
%SWIFT_BATTERY_ADD  Add the Battery subsystem to swift_6dof (Standalone folder).
%
%   swift_battery_add                     swift_6dof (backs it up first)
%   swift_battery_add swift_6dof          same
%
%   This is the Standalone copy: it only ever touches swift_6dof and has no
%   ArduPilot code in it. The ArduPilot_SITL folder has its own copy.
%
%   Safe to run again: an existing Battery subsystem and its logger are
%   removed and rebuilt, so the result is always the same.
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
%     batt VI term     Terminator on the [V; I] port (nothing reads it here).
%
%   NEEDS, ON THE PATH (run swift_setup first)
%     swift_placeholder.m v0.5 or later (P.batt.prm, P.batt.ocv, P.batt.x0),
%     swift_battery_fcn.m
%
%   The model is backed up to archive/pre_battery_<date_time>/ (inside this
%   folder) before it is opened for writing. 'nobackup' as a second argument
%   skips that.
%
%   Ryan Bilock, ARTS Lab | battery model | v0.3
%     v0.3  Standalone copy - swift_6dof only, ArduPilot code removed
%           (folders split 10 Nov 2026)
%     v0.2  (shared version) bridge-routing changes for the SITL model

if nargin < 1 || isempty(target), target = 'swift_6dof'; end
if nargin < 2, opt = ''; end
target = char(target);
backup = ~strcmpi(char(opt), 'nobackup');

if ~any(strcmpi(target, {'swift_6dof', 'both'}))
    error('swift_battery_add:folder', ['This is the Standalone copy - it only adds the battery ' ...
          'to swift_6dof. For %s use the ArduPilot_SITL folder (run its swift_setup).'], target);
end
models = {'swift_6dof'};

need = {'swift_placeholder.m', 'swift_battery_fcn.m'};
for k = 1:numel(need)
    if isempty(which(need{k}))
        error('swift_battery_add:missing', '%s is not on the MATLAB path. Run swift_setup first.', need{k});
    end
end

for k = 1:numel(models)
    add_to(models{k}, backup);
end
fprintf('\nNext: run swift_6dof, then:  swift_battery_plot\n\n');
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

fresh_P();

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

wire(M, pT,   inport(B, 1));
wire(M, pV,   inport(B, 2));
wire(M, pRho, inport(B, 3));
wire(M, outport(B, 1), inport(L, 1));
name_line(outport(B, 1), 'batt');
say('Battery', 'subsystem in: Thrust, airspeed (port 3), rho (COESA port 4)');
say('Logger ', 'out_batt (%s): SOC, V, I, P, mAh, Wh, V_cell, P_shaft, motor load, status', fmt);

% --- [V; I] is only used by the ArduPilot bridge, which this model has not got
Tm = add([M '/batt VI term'], 'simulink/Sinks/Terminator', [740 y0+85 760 y0+105]);
wire(M, outport(B, 2), inport(Tm, 1));
say('VI     ', '[V; I] capped with a Terminator (no ArduPilot bridge in the standalone model)');

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
function P = fresh_P()
evalin('base', 'swift_placeholder;');
P = evalin('base', 'P');
if ~isfield(P, 'batt') || ~isfield(P.batt, 'prm') || ~isfield(P.batt, 'ocv')
    error('swift_battery_add:params', ...
        'P.batt.prm / P.batt.ocv missing. swift_placeholder.m must be v0.5 or later.');
end
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
%  SMALL HELPERS
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
