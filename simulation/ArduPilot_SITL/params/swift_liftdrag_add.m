function R = swift_liftdrag_add(varargin)
%SWIFT_LIFTDRAG_ADD  Add lift, drag, CL and CD to swift_6dof_sitl (ArduPilot_SITL folder) and plot them.
%
%   swift_liftdrag_add                    add the Lift & Drag subsystem to swift_6dof_sitl
%                                         (backs the model up first)
%   swift_liftdrag_add nobackup           same, no backup
%   swift_liftdrag_add plot               plot 'out' from the base workspace
%   R = swift_liftdrag_add('plot', out, 'save', true, 'tag', 'my_run')
%
%   This is the ArduPilot_SITL copy: it only ever touches swift_6dof_sitl.
%   The Standalone folder has its own copy. Nothing is sent to ArduPilot:
%   lift and drag are logged only, the JSON bridge is not touched.
%
%   Safe to run again: an existing Lift & Drag subsystem and its logger are
%   removed and rebuilt, so the result is always the same.
%
%   THE PHYSICS (all from signals the model already computes)
%     The Aero coefficients subsystem sends the Aerodynamic Forces and
%     Moments block a 6-vector  C = [CD; -CY; CL; Cl; Cm; Cn]  (wind axes),
%     where  CL = P.aero.A(1,:)*u  and  CD = CD0 + k*CL^2.
%     The Dynamic Pressure block gives  qbar = 0.5*rho*V^2  (Pa).
%
%         L   = qbar * S * CL        N, perpendicular to the airflow
%         D   = qbar * S * CD        N, along (against) the airflow
%         L/D = CL / CD              CD >= CD0 > 0, so never divides by zero
%
%     These are exactly the wind-axis forces the aero block applies before
%     it rotates them into body axes, so the log matches what the 6-DOF feels.
%     The new blocks only read signals - the flight itself is unchanged.
%
%   WHAT IT ADDS (root diagram, under the existing blocks)
%     Lift & Drag      subsystem, light blue.  In 1: aero coefficient vector
%                      (Aero coefficients port 1, branched).  In 2: qbar
%                      (Dynamic Pressure port 1, branched).  Inside: Selector
%                      [CL; CD], Gain P.geom.S (qbar*S), Product -> [L; D],
%                      Divide -> L/D, Mux -> one 5-wide output.
%     out_aero         To Workspace, 5 columns:
%                        1 CL   -      2 CD   -      3 L   N
%                        4 D    N      5 L/D  -
%                      Decimation P.sitl.log_decim (100 Hz), same as the other loggers.
%
%   THE PLOT (swift_liftdrag_add plot)
%     1  CL and CD against time, CLmax line
%     2  Lift and drag against time, weight W line (L ~ W in level flight)
%     3  L/D against time, model (L/D)max line
%     4  Drag polar: logged CD vs CL over the model curve CD0 + k*CL^2,
%        best-L/D point marked. Airborne points only (V >= 0.8 Vs when out_V
%        is logged).
%     R (also printed): max L, max D, max CL with CLmax margin, best logged
%     L/D vs the model (L/D)max, mean airborne CL and CD.
%     'save', true writes liftdrag.png to data/runs/<tag>/.
%
%   NEEDS, ON THE PATH (run swift_setup first)
%     swift_placeholder.m, swift_aero_matrix.m (P.geom.S, P.aero.CD0,
%     P.aero.k_ind, P.aero.CLmax, P.mass.W),
%     swift_sitl_params.m (P.sitl.log_decim)
%
%   The model is backed up to archive/pre_liftdrag_<date_time>/ (inside this
%   folder) before it is opened for writing.
%
%   Ryan Bilock, ARTS Lab | lift & drag | v0.1 (14 Nov 2026)

%% ---- which mode ----------------------------------------------------------
M      = 'swift_6dof_sitl';
isSITL = true;

mode = '';
if nargin >= 1 && (ischar(varargin{1}) || isstring(varargin{1})), mode = lower(char(varargin{1})); end

if strcmp(mode, 'plot')
    R = plot_liftdrag(M, isSITL, varargin(2:end));
    if nargout == 0, clear R; end
    return
end

backup = true;
if strcmp(mode, 'nobackup'), backup = false; end
if nargin >= 2 && strcmpi(char(varargin{2}), 'nobackup'), backup = false; end
if ~isempty(mode) && ~any(strcmp(mode, {lower(M), 'nobackup'}))
    error('swift_liftdrag_add:folder', ['This is the ArduPilot_SITL copy - it only works on %s.\n' ...
          'For %s use the other folder (cd there and run its swift_setup).'], M, mode);
end

need = {'swift_placeholder.m', 'swift_aero_matrix.m'};
if isSITL, need{end+1} = 'swift_sitl_params.m'; end
for k = 1:numel(need)
    if isempty(which(need{k}))
        error('swift_liftdrag_add:missing', '%s is not on the MATLAB path. Run swift_setup first.', need{k});
    end
end

add_to(M, isSITL, backup);
fprintf('\nNext: run %s, then:  swift_liftdrag_add plot\n\n', M);
if nargout > 0, R = []; end
end

% =========================================================================
function add_to(M, isSITL, backup)
file = which([M '.slx']);
if isempty(file)
    error('swift_liftdrag_add:model', '%s.slx is not on the path. Run swift_setup first.', M);
end
if bdIsLoaded(M) && strcmp(get_param(M, 'Dirty'), 'on')
    error('swift_liftdrag_add:dirty', ...
        '%s has unsaved changes. Save it (Ctrl+S) or close it without saving, then run again.', M);
end

fresh_P(isSITL);

if backup
    root = fileparts(fileparts(file));               % this setup's folder (backups go to its archive\)
    bdir = fullfile(root, 'archive', ['pre_liftdrag_' datestr(now, 'yyyy-mm-dd_HHMMSS')]);
    if ~isfolder(bdir), mkdir(bdir); end
    if bdIsLoaded(M), close_system(M, 0); end
    copyfile(file, bdir);
    fprintf('\nBacked up %s.slx to %s\n', M, bdir);
end

load_system(M);
fprintf('Adding lift, drag, CL and CD to %s ...\n', M);

% --- start clean ---------------------------------------------------------
kill([M '/Lift & Drag']);
kill([M '/out_aero']);

% --- the two signals it needs ---------------------------------------------
AC = [M '/Aero coefficients'];
need_block(AC);
dp = find_system(M, 'SearchDepth', 1, 'RegExp', 'on', 'Name', '^Dynamic Pressure');
assert(~isempty(dp), 'swift_liftdrag_add: Dynamic Pressure block not found in %s.', M);
check_width(AC);
pC = outport(AC, 1);                 % [CD; -CY; CL; Cl; Cm; Cn]  (6x1)
pQ = outport(dp{1}, 1);              % qbar  Pa

% --- the subsystem -------------------------------------------------------
y0 = max_y(M) + 90;
x0 = 1455;                           % under the Aero coefficients column
B  = add([M '/Lift & Drag'], 'built-in/Subsystem', [x0 y0 x0+140 y0+110]);
build_liftdrag([M '/Lift & Drag']);
set_param(B, 'BackgroundColor', 'lightBlue');

tw0 = find_system(M, 'BlockType', 'ToWorkspace');
fmt = 'Timeseries';
if ~isempty(tw0), fmt = get_param(tw0{1}, 'SaveFormat'); end
L = add([M '/out_aero'], 'simulink/Sinks/To Workspace', [x0+220 y0+40 x0+310 y0+70], ...
        'VariableName', 'out_aero', 'SaveFormat', fmt, 'MaxDataPoints', 'inf');
if isSITL
    set_param(L, 'Decimation', 'P.sitl.log_decim');     % 100 Hz, same as the other loggers
end

wire(M, pC, inport(B, 1));
wire(M, pQ, inport(B, 2));
wire(M, outport(B, 1), inport(L, 1));
name_line(pC, 'C_aero');
name_line(outport(B, 1), 'CL CD L D L/D');
say('Lift & Drag', 'subsystem in: coefficient vector (Aero coefficients port 1), qbar (Dynamic Pressure)');
say('Logger     ', 'out_aero (%s): CL, CD, L [N], D [N], L/D', fmt);

save_system(M);
compile_check(M);
end

% =========================================================================
function build_liftdrag(S)
%  [C] --Selector[3 1]--> [CL;CD] --+-------------------------------> Mux 1
%                                   +--Product (x qbar*S)--> [L;D] --> Mux 2
%                                   +--Demux--> CL / CD --> L/D -----> Mux 3
%  [qbar] --Gain S--> qbar*S --------^
add([S '/C'],    'simulink/Sources/In1', [30  58  60  72]);
add([S '/qbar'], 'simulink/Sources/In1', [30 178  60 192]);

Sel = add([S '/CL, CD'], 'simulink/Signal Routing/Selector', [130 45 175 85], ...
          'InputPortWidth', '6', 'IndexOptions', 'Index vector (dialog)', 'Indices', '[3 1]');
G   = add([S '/S'], 'simulink/Math Operations/Gain', [130 170 175 200], ...
          'Gain', 'P.geom.S');
Pr  = add([S '/x qbar S'], 'simulink/Math Operations/Product', [290 110 320 160], ...
          'Inputs', '2', 'Multiplication', 'Element-wise(.*)');
Dm  = add([S '/split'], 'simulink/Signal Routing/Demux', [290 210 295 260], 'Outputs', '2');
Dv  = add([S '/CL over CD'], 'simulink/Math Operations/Divide', [360 215 390 255]);
Mx  = add([S '/Mux'], 'simulink/Signal Routing/Mux', [470 40 475 260], 'Inputs', '3');
add([S '/aero'], 'simulink/Sinks/Out1', [540 143 570 157]);

wire(S, outport([S '/C'], 1),    inport(Sel, 1));
wire(S, outport([S '/qbar'], 1), inport(G, 1));
wire(S, outport(Sel, 1), inport(Mx, 1));
wire(S, outport(Sel, 1), inport(Pr, 1));
wire(S, outport(G, 1),   inport(Pr, 2));
wire(S, outport(Sel, 1), inport(Dm, 1));
wire(S, outport(Dm, 1),  inport(Dv, 1));
wire(S, outport(Dm, 2),  inport(Dv, 2));
wire(S, outport(Pr, 1),  inport(Mx, 2));
wire(S, outport(Dv, 1),  inport(Mx, 3));
wire(S, outport(Mx, 1),  inport([S '/aero'], 1));

name_line(outport(Sel, 1), '[CL; CD]');
name_line(outport(G, 1),   'qbar*S');
name_line(outport(Pr, 1),  '[L; D] N');
name_line(outport(Dv, 1),  'L/D');
set_param(S, 'Description', ['L = qbar*S*CL, D = qbar*S*CD, L/D = CL/CD. ' ...
    'Output [CL; CD; L; D; L/D]. Added by swift_liftdrag_add.']);
end

% =========================================================================
function P = fresh_P(isSITL)
evalin('base', 'swift_placeholder;');
if isSITL
    evalin('base', 'P = swift_sitl_params(P);');
end
P = evalin('base', 'P');
if ~isfield(P, 'geom') || ~isfield(P.geom, 'S')
    error('swift_liftdrag_add:params', 'P.geom.S is missing - check swift_placeholder.m.');
end
end

function check_width(AC)
%  The Selector assumes the 6-wide [CD; -CY; CL; Cl; Cm; Cn] vector. Stop if
%  the Coeffs Mux inside Aero coefficients has been changed.
mx = find_system(AC, 'SearchDepth', 1, 'BlockType', 'Mux', 'Name', 'Coeffs');
if ~isempty(mx) && ~strcmp(strtrim(get_param(mx{1}, 'Inputs')), '6')
    error('swift_liftdrag_add:width', ['The Coeffs Mux in %s no longer has 6 inputs.\n' ...
          'Update the Selector indices in build_liftdrag (CL is element 3, CD element 1).'], AC);
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
    say('Check      ', 'Update Diagram (Ctrl+D) passed');
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
%  PLOT
% =========================================================================
function R = plot_liftdrag(M, isSITL, args)
out = [];
if ~isempty(args) && ~(ischar(args{1}) || isstring(args{1}))
    out = args{1};  args = args(2:end);
end
if isempty(out) && evalin('base', 'exist(''out'',''var'')'), out = evalin('base', 'out'); end
o.save = false;  o.tag = ['liftdrag_' datestr(now, 'yyyymmdd_HHMM')];  o.show = true;
for k = 1:2:numel(args), o.(lower(char(args{k}))) = args{k+1}; end

if ~evalin('base', 'exist(''P'',''var'')'), fresh_P(isSITL); end
P = evalin('base', 'P');
if ~isfield(P.aero, 'k_ind') || ~isfield(P.mass, 'W'), P = swift_aero_matrix(P); end
CD0 = P.aero.CD0;   k = P.aero.k_ind;   W = P.mass.W;   CLmax = P.aero.CLmax;
LDmax  = 1 / (2*sqrt(CD0*k));          % best L/D of the parabolic polar
CLbest = sqrt(CD0/k);                  % CL where it happens (CD = 2*CD0 there)

[t, y] = sig(out, 'out_aero');
if isempty(t)
    error('swift_liftdrag_add:nolog', ['No out_aero in this run. Add it with  swift_liftdrag_add  ' ...
          'and run %s again.'], M);
end
CL = y(:,1);  CD = y(:,2);  L = y(:,3);  D = y(:,4);  LD = y(:,5);

air = true(size(t));
[tv, V] = sig(out, 'out_V');
if ~isempty(tv)
    Vi  = interp1(tv, V(:,1), t, 'linear', 'extrap');
    air = Vi >= 0.8 * P.perf.V_stall;
end
if ~any(air), air = true(size(t)); end

if t(end) - t(1) > 120, tt = (t - t(1))/60; tl = 'Time (min)'; else, tt = t - t(1); tl = 'Time (s)'; end
vis = 'on'; if ~o.show, vis = 'off'; end
c1 = [0.00 0.35 0.75];  c2 = [0.80 0.25 0.10];  c3 = [0.10 0.50 0.25];  cg = [0.30 0.30 0.30];

f  = figure('Name', [M ' - lift & drag'], 'Color', 'w', 'Visible', vis, 'Position', [80 80 1280 820]);
TL = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(TL, sprintf('%s  |  Lift & drag   (S = %.2f m^2,  C_{D0} = %.3f,  k = %.4f)', ...
      strrep(M, '_', '\_'), P.geom.S, CD0, k), 'FontWeight', 'bold', 'Color', 'k');

% 1 | coefficients
a = nexttile(TL);  hold(a, 'on');  grid(a, 'on');
yyaxis(a, 'left');
plot(a, tt, CL, '-', 'Color', c1, 'LineWidth', 1.4);
yline(a, CLmax, '--', sprintf('C_{L,max} = %.2f', CLmax), 'Color', c1, ...
      'LabelHorizontalAlignment', 'left', 'FontSize', 10);
ylabel(a, 'C_L');  a.YAxis(1).Color = c1;
yyaxis(a, 'right');
plot(a, tt, CD, '-', 'Color', c2, 'LineWidth', 1.4);
ylabel(a, 'C_D');  a.YAxis(2).Color = c2;
xlabel(a, tl);  title(a, 'Lift and drag coefficients');

% 2 | forces
a = nexttile(TL);  hold(a, 'on');  grid(a, 'on');
plot(a, tt, L, '-', 'Color', c1, 'LineWidth', 1.4, 'DisplayName', 'Lift  L = q S C_L');
plot(a, tt, D, '-', 'Color', c2, 'LineWidth', 1.4, 'DisplayName', 'Drag  D = q S C_D');
yline(a, W, '--', sprintf('W = %.1f N', W), 'Color', cg, 'HandleVisibility', 'off', 'FontSize', 10);
xlabel(a, tl);  ylabel(a, 'Force (N)');  title(a, 'Lift and drag forces');
legend(a, 'Location', 'best');

% 3 | L/D
a = nexttile(TL);  hold(a, 'on');  grid(a, 'on');
LDp = LD;  LDp(~air) = NaN;
plot(a, tt, LDp, '-', 'Color', c3, 'LineWidth', 1.4);
yline(a, LDmax, '--', sprintf('model (L/D)_{max} = %.1f', LDmax), 'Color', cg, 'FontSize', 10);
xlabel(a, tl);  ylabel(a, 'L/D');  title(a, 'Lift-to-drag ratio (airborne)');

% 4 | drag polar
a = nexttile(TL);  hold(a, 'on');  grid(a, 'on');
cl = linspace(min([CL(air); -0.2]), max([CL(air); CLmax]), 200);
plot(a, CD0 + k*cl.^2, cl, '-', 'Color', cg, 'LineWidth', 1.2, 'DisplayName', 'model  C_{D0} + k C_L^2');
scatter(a, CD(air), CL(air), 8, c1, 'filled', 'MarkerFaceAlpha', 0.35, 'DisplayName', 'logged');
plot(a, [0 2*CD0], [0 2*CD0*LDmax], ':', 'Color', c3, 'LineWidth', 1.3, ...
     'DisplayName', sprintf('best L/D = %.1f at C_L = %.2f', LDmax, CLbest));
plot(a, 2*CD0, CLbest, 'o', 'Color', c3, 'MarkerFaceColor', c3, 'HandleVisibility', 'off');
xlabel(a, 'C_D');  ylabel(a, 'C_L');  title(a, 'Drag polar');
legend(a, 'Location', 'southeast');
xlim(a, [0, max([CD(air); 2*CD0]) * 1.1]);

readable(f);

% --- summary --------------------------------------------------------------
[R.L_max, i1] = max(L);            R.t_L_max = t(i1);
[R.D_max, i2] = max(D);            R.t_D_max = t(i2);
R.CL_max      = max(CL(air));      R.CL_margin = CLmax - R.CL_max;
R.LD_best     = max(LD(air));      R.LD_model_max = LDmax;   R.CL_best_model = CLbest;
R.CL_mean     = mean(CL(air));     R.CD_mean = mean(CD(air));
R.W           = W;                 R.fig = f;
warn = '';
if R.CL_margin < 0, warn = '   ** above CLmax - the linear model has no stall **'; end
fprintf('\n  LIFT & DRAG  (%s, %d airborne samples of %d)\n', M, nnz(air), numel(t));
fprintf('  Max lift      %8.1f N   at t = %.1f s   (%.2f W)\n', R.L_max, R.t_L_max, R.L_max/W);
fprintf('  Max drag      %8.1f N   at t = %.1f s\n', R.D_max, R.t_D_max);
fprintf('  Max CL        %8.3f     CLmax %.2f -> margin %.3f%s\n', R.CL_max, CLmax, R.CL_margin, warn);
fprintf('  Best L/D      %8.2f     model (L/D)max %.2f at CL %.2f\n', R.LD_best, LDmax, CLbest);
fprintf('  Mean CL / CD  %8.3f / %.4f (airborne)\n\n', R.CL_mean, R.CD_mean);

if o.save
    root = fileparts(fileparts(which([M '.slx'])));
    d = fullfile(root, 'data', 'runs', o.tag);
    if ~isfolder(d), mkdir(d); end
    exportgraphics(f, fullfile(d, 'liftdrag.png'), 'Resolution', 150, 'BackgroundColor', 'white');
    fprintf('  Saved %s\n\n', fullfile(d, 'liftdrag.png'));
end
end

function [t, y] = sig(o, name)
t = [];  y = [];  s = [];
if isa(o, 'Simulink.SimulationOutput')
    try, s = o.get(name); catch, s = []; end
elseif isstruct(o) && isfield(o, name)
    s = o.(name);                          % a saved run rebuilt as a struct
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
if isvector(y), y = y(:); end
if size(y, 1) ~= numel(t), y = y.'; end
[t, iu] = unique(t(:));  y = y(iu, :);
end

function readable(f)
%  Black, easy-to-read text on a white figure, whatever MATLAB theme is on.
%  yyaxis sides keep their colours.
if ~isgraphics(f), return; end
try, theme(f, 'light'); catch, end            % R2025a+; older releases skip this
k = [0 0 0];
set(f, 'Color', 'w');
for a = findall(f, 'Type', 'axes')'
    a.Color = 'w';
    set(a, 'XColor', k, 'ZColor', k, 'GridColor', [0.15 0.15 0.15], 'FontSize', max(a.FontSize, 11));
    if numel(a.YAxis) == 1, a.YColor = k; end
    a.Title.Color = k;   a.Title.FontSize = max(a.Title.FontSize, 12);
end
set(findall(f, 'Type', 'legend'), 'TextColor', k, 'Color', 'w', 'EdgeColor', [0.6 0.6 0.6]);
try, set(findall(f, 'Type', 'constantline'), 'LabelColor', k); catch, end   % R2021a+
end

% =========================================================================
%  SMALL HELPERS (same as swift_battery_add)
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
    error('swift_liftdrag_add:block', ...
        'Expected block not found: %s\nThe model has changed since this script was written.', p);
end
end

function say(step, fmt, varargin)
fprintf(['  %s  ' fmt '\n'], step, varargin{:});
end
