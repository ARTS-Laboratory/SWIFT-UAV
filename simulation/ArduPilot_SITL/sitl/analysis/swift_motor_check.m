function M = swift_motor_check(out, varargin)
%SWIFT_MOTOR_CHECK  Compare one swift_6dof_sitl flight with the MAD X7215 motor table.
%
%   swift_motor_check                     use 'out' from the base workspace
%   swift_motor_check(out)                use a SimulationOutput you kept
%   M = swift_motor_check(out, Name, Value, ...)
%
%   OPTIONS
%     'tag'    auto   run folder name (use the same tag as swift_plot_flight)
%     'save'   true   write PNGs, motor_steady_points.csv and motor_summary.txt
%                     to data/runs/<tag>/
%     'show'   true   false = build and save the figures without showing them
%     'win'    8      s, window used to decide a leg is steady
%     'min_s'  10     s, shortest steady stretch that counts as a data point
%     'wp'     auto   .waypoints file for the numbered event markers
%                     (default S.wp_file when S is in the workspace)
%     'events' true   numbered markers on the time-history figure, and the
%                     events list image if swift_plot_flight hasn't saved one
%
%   WHAT IT DOES
%     1  Finds the steady stretches of the flight: throttle, airspeed and
%        climb rate all flat, wings nearly level, in the air. In a
%        Session D flight these are the propulsion staircase steps, the
%        slalom legs and the pattern legs.
%     2  For each one it lists throttle, airspeed, pack current and voltage,
%        and the motor's electrical input power, next to
%          bench   the MAD table power at that throttle (static, no airspeed)
%          model   what the thrust table + swift_battery_fcn give at the same
%                  throttle and airspeed (an arithmetic check on the Simulink
%                  build: flight points should sit on it)
%     3  Checks the full-throttle static point at the start of the takeoff roll
%        against the bench row (107.96 A, 5093.6 W).
%     4  Plots motor efficiency (bench output/input) against the single
%        eta_mot = 0.82 the battery model uses at every throttle.
%
%   HOW TO READ IT
%     The bench table is a static test. In flight the airstream unloads the
%     propeller, so the same throttle draws LESS power than the bench row; a
%     flight point under the bench curve is expected. What matters is
%       (a) flight points on the model curve  -> the Simulink build is right;
%       (b) the static model curve against the bench curve -> how good the
%           efficiency chain is. Real in-flight agreement can only come from
%           a logged flight of the real aircraft.
%
%   Needs the run's out_batt (swift_battery_add), out_delta, out_V, out_pos
%   and out_euler. Plain MATLAB, no toolboxes beyond core.
%
%   The steady stretches are labelled S1, S2, ... so they don't clash with the
%   numbered event markers (1, 2, ...) shared by every plot script.
%
%   Ryan Bilock, ARTS Lab | SITL session D | v0.4
%     v0.4  works on any log: short logs (smoothing windows capped at the log
%           length), flat signals (time-history axes keep a minimum span),
%           and 'out' may be a struct of saved signals (swift_plot_selftest)
%     v0.3  time-history figure in minutes from the start of the log, with
%           the same event badges as the other figures
%     v0.2  event markers on the time history; steady stretches renamed S1..Sn

%% ---- inputs ---------------------------------------------------------------
if nargin < 1 || isempty(out)
    if evalin('base', 'exist(''out'',''var'')'), out = evalin('base', 'out'); else, out = []; end
end
here = fileparts(mfilename('fullpath'));
o.save = true;  o.tag = ['run_' datestr(now, 'yyyymmdd_HHMM')];
o.show = true;  o.win = 8;  o.min_s = 10;  o.wp = '';  o.events = true;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
P = evalin('base', 'P');
nm = {};
if evalin('base', 'exist(''S'',''var'')')
    Sb = evalin('base', 'S');
    if isempty(o.wp) && isstruct(Sb) && isfield(Sb, 'wp_file'), o.wp = Sb.wp_file; end
    if isstruct(Sb) && isfield(Sb, 'seg') && isfield(Sb, 'wp_file') && strcmp(Sb.wp_file, o.wp), nm = Sb.seg(:, 1); end
end
E = [];
if o.events && exist('swift_flight_events', 'file') == 2
    try, E = swift_flight_events(out, P, 'wp', o.wp, 'names', nm); catch, E = []; end
end
hasE = ~isempty(E) && ~isempty(E.t);

[t, yb] = sig(out, 'out_batt');
if isempty(t) || size(yb, 2) ~= 10
    error('swift_motor_check:nobatt', 'No 10-column out_batt in this run (run swift_battery_add swift_6dof_sitl).');
end
[tp, pos] = sig(out, 'out_pos');
if isempty(tp), error('swift_motor_check:nopos', 'No out_pos found - run the model first.'); end
dl  = resample_to(out, 'out_delta', t, 4);
V   = resample_to(out, 'out_V', t, 1);
eul = resample_to(out, 'out_euler', t, 3);
hp  = resample_to(out, 'out_pos', t, 3);
h   = -hp(:,3);
SOC = 100 * yb(:,1);  Vpk = yb(:,2);  Ipk = yb(:,3);  Pout = yb(:,4);  Psh = yb(:,8);
thr = dl(:,4);                                   % 0..1, the Thrust table's throttle input
Phot = P.elec.P_hotel / P.elec.pdb.eta;          % W, avionics load on the same pack
Pmot = Pout - Phot;                              % W, into the motor and ESC

dt = median(diff(t));
ns = max(1, min(round(1 / dt), floor(numel(t) / 2)));   % v0.4: short logs
sm = @(x, n) movmean(x, n);
thr_s = sm(thr, ns);  V_s = sm(V, ns);  Ip = sm(Ipk, ns);  Vps = sm(Vpk, ns);
Pm_s = sm(Pmot, ns);  Ps_s = sm(Psh, ns);
vz = sm(gradient(sm(h, ns), t), 2 * ns);         % m/s, climb positive
phi = abs(rad2deg(eul(:,1)));
if all(isnan(phi)), phi = zeros(size(t)); end    % no out_euler: skip the wings-level test

%% ---- steady stretches --------------------------------------------------------
Nw   = max(3, min(round(o.win / dt), numel(t)));        % v0.4: short logs
h0   = h(1);
air  = h > h0 + 8;
stdy = air & thr_s > 0.05 & phi < 12 & ...
       movstd(thr_s, Nw) < 0.025 & movstd(V_s, Nw) < 0.35 & movstd(vz, Nw) < 0.25;
d  = diff([0; stdy(:); 0]);
i0 = find(d == 1);  i1 = find(d == -1) - 1;
keep = (t(i1) - t(i0)) >= o.min_s;
i0 = i0(keep);  i1 = i1(keep);
n  = numel(i0);

bench = P.motor.bench;                           % [thr, gf, Nm, rpm, W]
S = struct('t0', zeros(n,1), 'dur', zeros(n,1), 'V', zeros(n,1), 'vz', zeros(n,1), ...
           'thr', zeros(n,1), 'I', zeros(n,1), 'Vpack', zeros(n,1), 'Pmot', zeros(n,1), ...
           'Pbench', zeros(n,1), 'Pmodel0', zeros(n,1), 'PmodelV', zeros(n,1), 'soc', zeros(n,1));
for k = 1:n
    w = i0(k):i1(k);
    S.t0(k) = t(i0(k));  S.dur(k) = t(i1(k)) - t(i0(k));
    S.V(k)  = mean(V_s(w));  S.vz(k) = mean(vz(w));  S.thr(k) = mean(thr_s(w));
    S.I(k)  = mean(Ip(w));   S.Vpack(k) = mean(Vps(w));
    S.Pmot(k) = mean(Pm_s(w));  S.soc(k) = mean(SOC(w));
    S.Pbench(k)  = interp1(bench(:,1), bench(:,5), S.thr(k));      % NaN outside 30-100 %
    S.Pmodel0(k) = model_pmot(P, S.thr(k), 0);
    S.PmodelV(k) = model_pmot(P, S.thr(k), S.V(k));
end

%% ---- full-throttle static point (start of the takeoff roll) ---------------------
iS = find(thr > 0.95 & V < 3 & h < h0 + 1);
ST = struct('ok', false, 'I', NaN, 'Vpack', NaN, 'Pmot', NaN);
if numel(iS) >= 5
    ST.ok = true;  ST.I = mean(Ipk(iS));  ST.Vpack = mean(Vpk(iS));  ST.Pmot = mean(Pmot(iS));
end

%% ---- print -------------------------------------------------------------------------
hr = repmat('-', 1, 112);
fprintf('\n%s\n SWIFT MOTOR CHECK  -  %s   (%s, 12S)\n%s\n', hr, o.tag, P.motor.model, hr);
fprintf(' %3s %7s %5s %5s %6s %5s %7s %7s %9s %9s %9s %7s\n', 'S#', 't s', 'dur s', 'V m/s', 'vz m/s', ...
        'thr %', 'I A', 'Vpack V', 'P_mot W', 'bench W', 'model(V)', 'fl/mod');
for k = 1:n
    fprintf(' S%-2d %7.0f %5.0f %5.1f %6.2f %5.1f %7.1f %7.1f %9.0f %9s %9.0f %6.1f%%\n', k, S.t0(k), S.dur(k), ...
        S.V(k), S.vz(k), 100*S.thr(k), S.I(k), S.Vpack(k), S.Pmot(k), nstr(S.Pbench(k)), S.PmodelV(k), ...
        100 * S.Pmot(k) / S.PmodelV(k));
end
if n == 0
    fprintf(' No steady stretch of %g s or more found (throttle, speed and climb rate flat, bank < 12 deg).\n', o.min_s);
end
fprintf('%s\n', hr);
if ST.ok
    fprintf(' Full-throttle static (start of roll): %.1f A, %.1f V, %.0f W motor input\n', ST.I, ST.Vpack, ST.Pmot);
    fprintf('   bench 100 %%: %.2f A, 47.18 V, %.0f W   ->  model %+.1f %% on power\n', P.motor.I_max, P.motor.P_max, ...
            100 * (ST.Pmot / P.motor.P_max - 1));
else
    fprintf(' Full-throttle static point not found (no samples above 95 %% throttle under 3 m/s on the ground).\n');
end
fprintf(' Peak pack current %.1f A  (motor %.0f A at 100 %%, ESC %.0f A rating)\n', max(Ipk), P.motor.I_max, P.esc.I_max);
fprintf(' Static model vs bench, motor input power: ');
tt = bench(:,1);  pm0 = arrayfun(@(a) model_pmot(P, a, 0), tt);  er = 100 * (pm0 ./ bench(:,5) - 1);
fprintf('%+.0f %% at 30 %%, %+.0f %% at 50 %%, %+.0f %% at 70 %%, %+.0f %% at 100 %%\n', ...
        er(tt == 0.30), er(tt == 0.50), er(tt == 0.70), er(tt == 1.00));
fprintf(' (a single eta_mot = %.2f is exact near 50 %% throttle; the bench efficiency runs %.0f-%.0f %%)\n', ...
        P.prop.eta_mot, 100 * min(bench_eff(bench)), 100 * max(bench_eff(bench)));
fprintf('%s\n\n', hr);

%% ---- figures -------------------------------------------------------------------------
C.blue = [0.13 0.40 0.75];  C.orange = [0.90 0.45 0.10];  C.red = [0.80 0.15 0.15];
C.grey = [0.45 0.45 0.45];  C.green = [0.18 0.60 0.30];
vis = 'on';  if ~o.show, vis = 'off'; end
try, figs = gobjects(0); catch, figs = []; end   % gobjects: MATLAB; [] keeps Octave happy

% 1  power against throttle
f = newfig('SWIFT motor check - power vs throttle', vis);  figs(end+1) = f;
ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
tg = 0.30:0.01:1;
hB  = plot(ax, 100*bench(:,1), bench(:,5), 's-', 'Color', 'k', 'MarkerFaceColor', 'k', 'LineWidth', 1.6);
hM0 = plot(ax, 100*tg, arrayfun(@(a) model_pmot(P, a, 0), tg), '--', 'Color', C.blue, 'LineWidth', 1.4);
hM1 = plot(ax, 100*tg, arrayfun(@(a) model_pmot(P, a, 22), tg), ':', 'Color', C.grey, 'LineWidth', 1.3);
plot(ax, 100*tg, arrayfun(@(a) model_pmot(P, a, 27.5), tg), ':', 'Color', C.grey, 'LineWidth', 1.3);
leg = {'MAD bench (static)', 'model, 0 m/s', 'model, 22 and 27.5 m/s'};  hh = [hB hM0 hM1];
if n > 0
    hF = scatter(ax, 100*S.thr, S.Pmot, 70, S.V, 'filled', 'MarkerEdgeColor', 'k');
    colormap(ax, parula);  cb = colorbar(ax);  ylabel(cb, 'Airspeed (m/s)');
    for k = 1:n, text(ax, 100*S.thr(k) + 0.8, S.Pmot(k), sprintf('S%d', k), 'Color', 'k', 'FontWeight', 'bold'); end
    hh(end+1) = hF;  leg{end+1} = 'flight, steady stretches';
end
if ST.ok
    hS = plot(ax, 100, ST.Pmot, 'p', 'MarkerSize', 13, 'MarkerFaceColor', C.orange, 'MarkerEdgeColor', 'k');
    hh(end+1) = hS;  leg{end+1} = 'flight, full-throttle static';
end
legend(ax, hh, leg, 'Location', 'northwest');
xlabel(ax, 'Throttle (%)');  ylabel(ax, 'Motor electrical input (W)');
title(ax, 'Motor input power vs throttle: bench is static, flight points are unloaded by airspeed');

% 2  efficiency and static model error
f = newfig('SWIFT motor check - efficiency and model error', vis);  figs(end+1) = f;
ax = subplot(2, 1, 1);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, 100*bench(:,1), 100*bench_eff(bench), 's-', 'Color', 'k', 'MarkerFaceColor', 'k', 'LineWidth', 1.5);
plot(ax, [30 100], 100*P.prop.eta_mot*[1 1], '--', 'Color', C.red, 'LineWidth', 1.5);
ylabel(ax, 'Motor + ESC efficiency (%)');  ylim(ax, [60 90]);
legend(ax, {'MAD bench (shaft out / electrical in)', sprintf('battery model, eta_{mot} = %.2f', P.prop.eta_mot)}, ...
       'Location', 'south');
title(ax, 'Efficiency: bench against the single value the battery model uses');
ax = subplot(2, 1, 2);  hold(ax, 'on');  grid(ax, 'on');
bar(ax, 100*tt, er, 0.7, 'FaceColor', C.blue);
plot(ax, [27 103], [0 0], '-', 'Color', 'k');  xlim(ax, [27 103]);
xlabel(ax, 'Throttle (%)');  ylabel(ax, 'Model - bench, input power (%)');
title(ax, 'Static (0 m/s) model error against the bench table');

% 3  time history
f = newfig('SWIFT motor check - time history', vis);  figs(end+1) = f;
rows = {100*thr_s, 'Throttle (%)'; Ip, 'Pack current (A)'; V_s, 'Airspeed (m/s)'};
for r = 1:3
    ax = subplot(3, 1, r);  hold(ax, 'on');  grid(ax, 'on');
    y = rows{r, 1};  plot(ax, t/60, y, 'Color', C.blue, 'LineWidth', 1.1);
    ylabel(ax, rows{r, 2});
    yl = [min(y) max(y)];  if any(~isfinite(yl)), yl = [0 1]; end
    if r < 3, yl(1) = min(yl(1), 0); end                 % throttle and current from zero
    span = [10 2 2];                                     % v0.4: smallest span, so flat logs don't zoom to noise
    if diff(yl) < span(r)
        if r < 3, yl(2) = yl(1) + span(r); else, yl = max(0, mean(yl) - span(r)/2) + [0 span(r)]; end
    end
    yl = yl + 0.08 * diff(yl) * [-1 1];
    ylim(ax, yl);
    for k = 1:n
        try
            patch(ax, [t(i0(k)) t(i1(k)) t(i1(k)) t(i0(k))] / 60, yl([1 1 2 2]), C.green, ...
                  'FaceAlpha', 0.20, 'EdgeColor', 'none');
        catch
            plot(ax, t([i0(k) i1(k)]) / 60, yl(1) * [1 1], '-', 'Color', C.green, 'LineWidth', 4);
        end
        if r == 3, text(ax, t(i0(k)) / 60, yl(1), sprintf(' S%d', k), 'VerticalAlignment', 'bottom', 'FontWeight', 'bold', 'Color', C.green); end
    end
    if hasE, swift_mark_events(ax, E, 'scale', 1/60, 'labels', r == 1); end
end
xlabel(ax, 'Time from start of log (min)');
title(subplot(3, 1, 1), 'Steady stretches (green, S1..Sn as in the table) and numbered flight points');
fl = [];
if hasE && exist('swift_events_figure', 'file') == 2, fl = swift_events_figure(E, o.tag, vis); end

for k = 1:numel(figs), readable(figs(k)); end

%% ---- output + save -----------------------------------------------------------------------
M = struct('tag', o.tag, 'steady', S, 'static', ST, 'n', n, 'static_model_err_pct', er, ...
           'bench_thr', tt, 'I_peak', max(Ipk), 'events', E);
if o.save
    d = fullfile(fileparts(fileparts(here)), 'data', 'runs', o.tag);   % sitl/analysis -> ArduPilot_SITL/data/runs
    if ~isfolder(d), mkdir(d); end
    for k = 1:numel(figs)
        nm = regexprep(lower(get(figs(k), 'Name')), '[^a-z0-9]+', '_');
        try
            exportgraphics(figs(k), fullfile(d, [nm '.png']), 'Resolution', 200);
        catch
            saveas(figs(k), fullfile(d, [nm '.png']));
        end
    end
    %  events list: only if swift_plot_flight hasn't already saved one for this run
    for k = 1:numel(fl)
        nmf = fullfile(d, [regexprep(lower(get(fl(k), 'Name')), '[^a-z0-9]+', '_') '.png']);
        if isfile(nmf), continue, end
        try, exportgraphics(fl(k), nmf, 'Resolution', 200); catch, saveas(fl(k), nmf); end
    end
    fid = fopen(fullfile(d, 'motor_steady_points.csv'), 'w');
    fprintf(fid, 't_s,dur_s,V_ms,vz_ms,throttle_pct,I_A,Vpack_V,Pmotor_W,Pbench_static_W,Pmodel_V_W,SOC_pct\n');
    for k = 1:n
        fprintf(fid, '%.1f,%.1f,%.2f,%.2f,%.1f,%.2f,%.2f,%.0f,%.0f,%.0f,%.1f\n', S.t0(k), S.dur(k), S.V(k), ...
                S.vz(k), 100*S.thr(k), S.I(k), S.Vpack(k), S.Pmot(k), S.Pbench(k), S.PmodelV(k), S.soc(k));
    end
    fclose(fid);
    fid = fopen(fullfile(d, 'motor_summary.txt'), 'w');
    fprintf(fid, 'steady stretches: %d\nfull-throttle static: I %.1f A, Pmotor %.0f W (bench %.2f A, %.0f W)\n', ...
            n, ST.I, ST.Pmot, P.motor.I_max, P.motor.P_max);
    fprintf(fid, 'peak pack current: %.1f A\n', max(Ipk));
    fclose(fid);
    fprintf(' Saved to %s\n\n', d);
end
end

% =========================================================================
function Pm = model_pmot(P, thr, V)
%  Motor electrical input (W) the model gives at throttle thr (0-1) and airspeed V:
%  thrust table, momentum theory, eta_prop and eta_mot - the same chain as swift_battery_fcn.
rho = P.const.rho0;
thr = min(max(thr, 0), 1);                       % a throttle of 1.0000001 must not fall off the table
T   = interp1(P.prop.throttle_bp(:), P.prop.T_static(:), thr, 'linear') * max(0, 1 - V / P.prop.V_pitch);
vi  = -V/2 + sqrt(V^2/4 + T / (2 * rho * P.prop.A_disk));
Pm  = T * (V + vi) / P.prop.eta_prop / P.prop.eta_mot;
end

function e = bench_eff(bench)
%  Motor + ESC efficiency from the bench table: shaft power (torque x speed) / electrical input.
e = bench(:,3) .* 2*pi .* bench(:,4) / 60 ./ bench(:,5);
end

function s = nstr(v)
if isnan(v), s = '-'; else, s = sprintf('%.0f', v); end
end

function f = newfig(name, vis)
f = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', 'Visible', vis, 'Position', [80 80 1100 760]);
try, theme(f, 'light'); catch, end
end

function y = resample_to(out, name, t, n)
[ts, ys] = sig(out, name);
if isempty(ts)
    y = nan(numel(t), n);  return
end
if size(ys, 2) < n, ys(:, end+1:n) = NaN; end
y = interp1(ts, ys(:, 1:n), t, 'linear', 'extrap');
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
%READABLE  Black, easy-to-read text on a white figure, whatever MATLAB theme is on.
if ~isgraphics(f), return; end
try, theme(f, 'light'); catch, end
k = [0 0 0];
set(f, 'Color', 'w');
for a = findall(f, 'Type', 'axes')'
    set(a, 'Color', 'w', 'XColor', k, 'YColor', k, 'ZColor', k, 'GridColor', [0.15 0.15 0.15], ...
           'FontSize', max(get(a, 'FontSize'), 11));
    ti = get(a, 'Title');  set(ti, 'Color', k, 'FontSize', max(get(ti, 'FontSize'), 12));
end
set(findall(f, 'Type', 'legend'), 'TextColor', k, 'Color', 'w', 'EdgeColor', [0.6 0.6 0.6]);
set(findall(f, 'Type', 'colorbar'), 'Color', k);
end
