function R = swift_battery_plot(out, varargin)
%SWIFT_BATTERY_PLOT  Battery plots and a summary for one swift_6dof / swift_6dof_sitl run.
%
%   swift_battery_plot                    use 'out' (or out_batt) from the base workspace
%   swift_battery_plot(out)               a SimulationOutput you kept
%   R = swift_battery_plot(out, Name, Value, ...)
%
%   OPTIONS
%     'save'   false    true = write the PNG and summary to data/runs/<tag>/
%     'tag'    auto     run folder name (default batt_yyyymmdd_HHMM). Use the
%                       same tag as swift_plot_flight to keep a run together.
%     'show'   true     false = build the figure without showing it
%     'events' auto     E from swift_flight_events (swift_plot_flight passes
%                       it). Left out: worked out here when the run has
%                       out_pos, using S.wp_file if S is in the workspace.
%                       [] or false = no markers.
%
%   FIGURE (one window, four panels, warning periods shaded)
%     1  State of charge, %, with the reserve line
%     2  Pack voltage under load, with the low and critical lines
%     3  Current (A) and power out of the pack (kW)
%     4  Energy used (Wh) against the usable energy, and mAh used
%
%   R (also printed)  duration, start/end charge, minimum pack and cell
%   voltage, peak current and power, energy and charge used, average power,
%   projected time left to the reserve at that average, time spent in each
%   warning, and time with the motor above its rating.
%
%   Reads out_batt, the 10-signal logger swift_battery_add puts on the root
%   diagram (columns listed in swift_battery_fcn.m). Thresholds come from P
%   in the base workspace.
%
%   Ryan Bilock, ARTS Lab | battery model | v0.4
%     v0.4  voltage and reserve labels moved clear of their lines; event
%           badges come from swift_mark_events v0.2; time axis wording
%     v0.3  numbered event markers (same numbers as swift_plot_flight)
%     v0.2  readable(): light theme + black text (dark-theme fix)

%% ---- inputs -------------------------------------------------------------
if nargin < 1 || isempty(out)
    if evalin('base', 'exist(''out'',''var'')')
        out = evalin('base', 'out');
    elseif evalin('base', 'exist(''out_batt'',''var'')')
        out = struct('out_batt', evalin('base', 'out_batt'));
    else
        error('swift_battery_plot:noout', 'No ''out'' or ''out_batt'' in the base workspace - run the model first.');
    end
end
o.save = false;  o.tag = ['batt_' datestr(now, 'yyyymmdd_HHMM')];  o.show = true;  o.events = 'auto';
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
E = o.events;
if ischar(E) && strcmp(E, 'auto')
    E = [];
    if exist('swift_flight_events', 'file') == 2 && evalin('base', 'exist(''P'',''var'')')
        try
            wp = '';  nm = {};
            if evalin('base', 'exist(''S'',''var'')')
                Sb = evalin('base', 'S');
                if isstruct(Sb) && isfield(Sb, 'wp_file'), wp = Sb.wp_file; end
                if isstruct(Sb) && isfield(Sb, 'seg'), nm = Sb.seg(:, 1); end
            end
            E = swift_flight_events(out, evalin('base', 'P'), 'wp', wp, 'names', nm);
        catch
            E = [];
        end
    end
elseif islogical(E) || ~isstruct(E)
    E = [];
end

[t, y] = sig(out, 'out_batt');
if isempty(t)
    error('swift_battery_plot:nolog', 'No out_batt in this run. Run swift_battery_add, then run the model again.');
end
if size(y, 2) ~= 10
    error('swift_battery_plot:width', 'out_batt has %d columns, expected 10.', size(y, 2));
end
SOC = y(:,1);  Vp = y(:,2);  I = y(:,3);  Pw = y(:,4);  mAh = y(:,5);  Wh = y(:,6);
Vc  = y(:,7);  Psh = y(:,8); ml = y(:,9); st = round(y(:,10));

% thresholds (defaults = the v0.5 placeholder values)
cells = 12;  Ah = 10;  Vlow = 3.50;  Vcrit = 3.30;  res = 0.20;  Vnom = 44.4;
if evalin('base', 'exist(''P'',''var'')')
    P = evalin('base', 'P');
    if isfield(P, 'batt') && isfield(P.batt, 'prm')
        cells = P.batt.cells;  Ah = P.batt.Ah;  Vlow = P.batt.Vcell_low;
        Vcrit = P.batt.Vcell_crit;  res = P.batt.SOC_res;  Vnom = P.batt.V_nom;
    end
end

%% ---- summary ------------------------------------------------------------
dur  = t(end) - t(1);                                 % s
dt   = diff(t);  dt = [dt; dt(end)];
Pavg = 3600 * (Wh(end) - Wh(1)) / max(dur, eps);      % W
left = max(0, SOC(end) - res) * Ah * Vnom;            % Wh to the reserve (approx.)
R.duration_s       = dur;
R.SOC_start_pct    = 100 * SOC(1);
R.SOC_end_pct      = 100 * SOC(end);
R.V_pack_min       = min(Vp);
R.V_cell_min       = min(Vc);
R.I_max            = max(I);
R.P_max_W          = max(Pw);
R.Wh_used          = Wh(end) - Wh(1);
R.mAh_used         = mAh(end) - mAh(1);
R.P_avg_W          = Pavg;
R.min_left_at_avg  = 60 * left / max(Pavg, eps);
R.t_below_reserve  = sum(dt(st == 1));
R.t_low_volt       = sum(dt(st == 2));
R.t_crit_volt      = sum(dt(st == 3));
R.t_overload       = sum(dt(st == 4));
R.t_empty          = sum(dt(st == 5));
R.t_motor_over     = sum(dt(ml > 1));

fprintf('\n  SWIFT battery - %.0f s run (%.1f min)\n', dur, dur/60);
fprintf('  %-34s %8.1f -> %.1f %%\n', 'State of charge',          R.SOC_start_pct, R.SOC_end_pct);
fprintf('  %-34s %8.2f V  (%.3f V/cell)\n', 'Lowest pack voltage', R.V_pack_min, R.V_cell_min);
fprintf('  %-34s %8.1f A  | %.0f W\n', 'Peak current | power',     R.I_max, R.P_max_W);
fprintf('  %-34s %8.1f Wh | %.0f mAh\n', 'Used',                   R.Wh_used, R.mAh_used);
fprintf('  %-34s %8.0f W\n', 'Average power',                      R.P_avg_W);
fprintf('  %-34s %8.1f min\n', sprintf('Left to %.0f%% reserve at that average', 100*res), R.min_left_at_avg);
fprintf('  %-34s %6.1f / %.1f / %.1f / %.1f s\n', 'Below reserve / low / crit / overload', ...
        R.t_below_reserve, R.t_low_volt, R.t_crit_volt, R.t_overload);
if R.t_empty > 0
    fprintf(2, '  Pack EMPTY (SOC 0) for %.1f s - mAh/Wh past that point are demand, not charge.\n', R.t_empty);
end
if R.t_motor_over > 0
    fprintf(2, '  Motor above its %s electrical rating for %.1f s (see P.motor.P_max).\n', ...
            'sheet', R.t_motor_over);
end
fprintf('\n');

%% ---- figure -------------------------------------------------------------
vis = 'on';  if ~o.show, vis = 'off'; end
f = figure('Name', 'SWIFT battery', 'Color', 'w', 'Visible', vis, 'Position', [80 80 1200 760]);
try, theme(f, 'light'); catch, end   % readable on screen too (R2025a+)
tm = t / 60;                                          % min
cWarn = [1.00 0.85 0.30];  cCrit = [1.00 0.55 0.55];

ax1 = subplot(2,2,1);
shade(ax1, tm, st, cWarn, cCrit);
plot(ax1, tm, 100*SOC, 'LineWidth', 1.6, 'Color', [0.00 0.45 0.74]);
yline(ax1, 100*res, '--', sprintf('%.0f%% reserve', 100*res), 'Color', [0.85 0.33 0.10], ...
      'LabelHorizontalAlignment', 'left');
ylim(ax1, [0 105]);  ylabel(ax1, 'State of charge (%)');  title(ax1, 'State of charge');

ax2 = subplot(2,2,2);
shade(ax2, tm, st, cWarn, cCrit);
plot(ax2, tm, Vp, 'LineWidth', 1.6, 'Color', [0.47 0.67 0.19]);
yline(ax2, cells*Vlow,  '--', sprintf('low %.1f V', cells*Vlow),   'Color', [0.93 0.69 0.13], ...
      'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'top');
yline(ax2, cells*Vcrit, '--', sprintf('crit %.1f V', cells*Vcrit), 'Color', [0.85 0.10 0.10], ...
      'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
yline(ax2, cells*4.2,   ':',  'full', 'Color', [0.5 0.5 0.5]);
ylim(ax2, [min(cells*3.0, floor(min(Vp)) - 1), cells*4.2 + 1.5]);
ylabel(ax2, 'Pack voltage (V)');  title(ax2, sprintf('Pack voltage under load (%dS)', cells));

ax3 = subplot(2,2,3);
yyaxis(ax3, 'left');
plot(ax3, tm, I, 'LineWidth', 1.4);  ylabel(ax3, 'Current (A)');
yyaxis(ax3, 'right');
plot(ax3, tm, Pw/1000, 'LineWidth', 1.1);  ylabel(ax3, 'Power out of pack (kW)');
title(ax3, 'Current and power');

ax4 = subplot(2,2,4);
yyaxis(ax4, 'left');
plot(ax4, tm, Wh, 'LineWidth', 1.6);  ylabel(ax4, 'Energy used (Wh)');
yline(ax4, (1-res)*Ah*Vnom, '--', 'usable', 'Color', [0.85 0.33 0.10]);
yyaxis(ax4, 'right');
plot(ax4, tm, mAh, 'LineWidth', 1.1);  ylabel(ax4, 'Charge used (mAh)');
title(ax4, 'Energy and charge used');

for a = [ax1 ax2 ax3 ax4]
    grid(a, 'on');  set(a, 'Layer', 'top');  xlabel(a, 'Time from start of log (min)');  xlim(a, [tm(1) max(tm(end), tm(1) + eps)]);
end
if isstruct(E) && ~isempty(E.t) && exist('swift_mark_events', 'file') == 2
    swift_mark_events(ax1, E, 'scale', 1/60);  swift_mark_events(ax2, E, 'scale', 1/60);
    swift_mark_events(ax3, E, 'scale', 1/60, 'labels', false);
    swift_mark_events(ax4, E, 'scale', 1/60, 'labels', false);
end
sgtitle(f, sprintf('SWIFT battery  |  %.1f -> %.1f %%  |  %.0f Wh used  |  min %.2f V/cell', ...
        R.SOC_start_pct, R.SOC_end_pct, R.Wh_used, R.V_cell_min));

readable(f);   % black text before showing/saving

%% ---- save ---------------------------------------------------------------
if o.save
    here = fileparts(mfilename('fullpath'));
    d = fullfile(fileparts(here), 'data', 'runs', o.tag);
    if ~isfolder(d), mkdir(d); end
    exportgraphics(f, fullfile(d, 'swift_battery.png'), 'Resolution', 150);
    fid = fopen(fullfile(d, 'battery_summary.txt'), 'w');
    for fn = fieldnames(R)'
        fprintf(fid, '%-18s %g\n', fn{1}, R.(fn{1}));
    end
    fclose(fid);
    fprintf('  Saved to %s\n\n', d);
end
end

% =========================================================================
function shade(ax, tm, st, cWarn, cCrit)
% Background bands: amber for reserve/low, red for critical/overload/empty.
hold(ax, 'on');
for lev = 1:2
    if lev == 1, m = (st == 1 | st == 2); c = cWarn; else, m = (st >= 3); c = cCrit; end
    if ~any(m), continue, end
    d  = diff([0; m(:); 0]);
    i0 = find(d == 1);  i1 = find(d == -1) - 1;
    for k = 1:numel(i0)
        xs = [tm(i0(k)) tm(i1(k)) tm(i1(k)) tm(i0(k))];
        patch(ax, xs, [-1e4 -1e4 1e4 1e4], c, 'EdgeColor', 'none', 'FaceAlpha', 0.35, ...
              'HandleVisibility', 'off');
    end
end
end

function [t, y] = sig(o, name)
% Same reader as swift_sitl_build: Timeseries, Structure with time, or Array.
t = [];  y = [];
try
    if isstruct(o) && isfield(o, name), s = o.(name); else, s = o.get(name); end
catch
    return
end
if isempty(s), return, end
if isa(s, 'timeseries')
    t = s.Time;  y = squeeze(s.Data);
elseif isstruct(s)
    t = s.time;  y = squeeze(s.signals.values);
else                                     % Array format: no time saved with it
    y = s;
    if size(y, 1) == 10 && size(y, 2) ~= 10, y = y.'; end
    try, tend = o.tout(end); catch, tend = size(y, 1) - 1; end
    t = linspace(0, tend, size(y, 1))';
end
if size(y, 1) ~= numel(t), y = y.'; end
[t, iu] = unique(t);  y = y(iu, :);
end

% =========================================================================
function readable(f)
%READABLE  Black, easy-to-read text on a white figure, whatever MATLAB theme is on.
%  MATLAB R2025a+ in dark theme draws light-grey text that almost vanishes on
%  the white background these figures are saved with. This forces the light
%  theme, black text, a white plot area and slightly larger fonts so the
%  saved PNGs read well on slides. Coloured yyaxis sides keep their colours.
if ~isgraphics(f), return; end
try, theme(f, 'light'); catch, end            % R2025a+; older releases skip this
k = [0 0 0];
set(f, 'Color', 'w');
for a = findall(f, 'Type', 'axes')'
    set(a, 'Color', 'w', 'XColor', k, 'ZColor', k, 'GridColor', [0.15 0.15 0.15], ...
           'FontSize', max(a.FontSize, 11));
    if numel(a.YAxis) == 1, a.YColor = k; end
    a.Title.Color = k;   a.Title.FontSize = max(a.Title.FontSize, 12);
end
set(findall(f, 'Type', 'legend'), 'TextColor', k, 'Color', 'w', 'EdgeColor', [0.6 0.6 0.6]);
set(findall(f, 'Type', 'colorbar'), 'Color', k);
set(findall(f, 'Type', 'subplottext'), 'Color', k);   % sgtitle
end
