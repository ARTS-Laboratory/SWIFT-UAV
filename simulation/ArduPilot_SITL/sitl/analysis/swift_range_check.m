function RC = swift_range_check(out, varargin)
%SWIFT_RANGE_CHECK  Range and efficiency of one swift_6dof_sitl flight (Session E range test).
%
%   swift_range_check                     'out' from the base workspace
%   RC = swift_range_check(out, Name, Value, ...)
%
%   OPTIONS
%     'tag'      auto    run folder data/runs/<tag>/ (same as swift_plot_flight)
%     'save'     true    write swift_range_check.png, range_summary.txt, range_legs.csv
%     'show'     true    false = build and save without showing
%     'wp'       auto    .waypoints file for the event markers (default S.wp_file)
%     'events'   true    numbered event markers (same numbers as every figure)
%     'reserve'  2000    mAh left at ArduPlane's low-battery RTL (BATT_LOW_MAH)
%     'min_leg'  60      s, shortest steady stretch that counts as a cruise leg
%
%   WHAT IT MEASURES
%     Cruise legs are found from the log alone: airborne, wings level
%     (|roll| < 4 deg), level (|climb| < 0.4 m/s, 5 s average), for at least
%     'min_leg' seconds. On each leg:
%         charge per distance   mAh/km  =  delta mAh used / delta ground km
%         range per charge      km/Ah   =  1000 / (mAh/km)
%         energy per distance   Wh/km
%     plus mean airspeed, groundspeed, throttle, current and power. With S
%     from swift_sitl_cub_e in the workspace the legs are named "out" (on the
%     runway centreline) and "back" (on the return lane), the flown points are
%     drawn on the model best-range curve S.best_range, and the charge used is
%     compared with the plan S.batt_est leg by leg.
%
%     Projected still-air range to the reserve:
%         R_res = s_flown + (mAh_left - reserve) / (mAh/km of the cruise legs)
%
%   FIGURE  swift_range_check
%     1  Charge used against ground distance flown: flight, plan, low-battery
%        RTL line, cruise legs shaded, event badges
%     2  Airspeed and the commanded best-range speed against distance
%     3  Throttle against distance, planned throttle per leg
%     4  Pack current against distance
%     5  Model Wh/km against airspeed (S.best_range) with each flown leg.
%        Energy per km is used here because mAh per km creeps up as the pack
%        voltage falls (same power, more current), so the out leg always reads
%        a few percent better than the back leg in mAh/km but not in Wh/km.
%   Key underneath: one line per leg and the whole-flight numbers.
%
%   Ryan Bilock, ARTS Lab | SITL session E | v0.1

%% ---- inputs -----------------------------------------------------------------
if nargin < 1 || isempty(out)
    if evalin('base', 'exist(''out'',''var'')'), out = evalin('base', 'out'); else, out = []; end
end
here = fileparts(mfilename('fullpath'));
o.tag = ['run_' datestr(now, 'yyyymmdd_HHMM')];  o.save = true;  o.show = true;  o.wp = '';
o.events = true;  o.reserve = 2000;  o.min_leg = 60;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
Sb = [];
if evalin('base', 'exist(''S'',''var'')'), Sb = evalin('base', 'S'); end
if isempty(o.wp) && isstruct(Sb) && isfield(Sb, 'wp_file') && isfile(Sb.wp_file), o.wp = Sb.wp_file; end
useS = isstruct(Sb) && isfield(Sb, 'wp_file') && strcmp(Sb.wp_file, o.wp) && isfield(Sb, 'best_range');
if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
P = evalin('base', 'P');

%% ---- signals --------------------------------------------------------------------
[t, pos] = sig(out, 'out_pos');
if numel(t) < 10, error('swift_range_check:nolog', 'No out_pos found - run the model first.'); end
Va  = rs(out, 'out_V', t, 1);  eul = rs(out, 'out_euler', t, 3);  dl = rs(out, 'out_delta', t, 4);
[tb, yb] = sig(out, 'out_batt');
if isempty(tb) || size(yb, 2) < 10
    error('swift_range_check:nobatt', 'No out_batt in this run - the range check needs the battery log.');
end
bt  = interp1(tb, yb, t, 'linear', 'extrap');
h   = -pos(:,3);  ha = h - h(1);
dt  = median(diff(t));
ns  = max(1, round(5 / dt));
vz  = movmean(gradient(h, t), ns);
ds  = [0; hypot(diff(pos(:,1)), diff(pos(:,2)))];
s   = cumsum(ds) / 1000;                                   % km of ground track
gs  = movmean([0; ds(2:end) ./ diff(t)], ns);
thr = 100 * dl(:,4);  I = bt(:,3);  Pw = bt(:,4);  mAh = bt(:,5);  Wh = bt(:,6);  soc = 100 * bt(:,1);
air = ha > 15 & Va > 0.8 * P.perf.V_stall;
hdg = 0;  if isstruct(Sb) && isfield(Sb, 'airport'), hdg = Sb.airport.hdg;
elseif isfield(P, 'sitl') && isfield(P.sitl, 'hdg0'), hdg = P.sitl.hdg0; end
xr  =  pos(:,1) * cosd(hdg) + pos(:,2) * sind(hdg);
yr  = -pos(:,1) * sind(hdg) + pos(:,2) * cosd(hdg);

%% ---- cruise legs ------------------------------------------------------------------
st = air & abs(rad2deg(eul(:,1))) < 4 & abs(vz) < 0.4;
st = movmean(double(st), max(1, round(3 / dt))) > 0.7;       % bridge short gaps
d  = diff([0; st; 0]);  i0 = find(d == 1);  i1 = find(d == -1) - 1;
keep = (t(i1) - t(i0)) >= o.min_leg;  i0 = i0(keep);  i1 = i1(keep);
cut = max(1, round(10 / dt));                               % drop 10 s at each end (settling)
i0 = i0 + cut;  i1 = i1 - cut;  ok = i1 > i0;  i0 = i0(ok);  i1 = i1(ok);
nL = numel(i0);
Lg = struct('name', {}, 'i0', {}, 'i1', {}, 't_s', {}, 'km', {}, 'V', {}, 'gs', {}, 'thr', {}, ...
            'I', {}, 'P', {}, 'mAh', {}, 'mAh_km', {}, 'km_Ah', {}, 'Wh_km', {}, 'plan_thr', {});
for k = 1:nL
    j = i0(k):i1(k);
    L.name = sprintf('leg %d', k);
    if useS && isfield(Sb, 'y_lane')
        ym = median(yr(j));
        if abs(ym) < 150, L.name = 'out'; elseif abs(ym - Sb.y_lane) < 150, L.name = 'back'; end
    end
    L.i0 = i0(k);  L.i1 = i1(k);  L.t_s = t(i1(k)) - t(i0(k));
    L.km = s(i1(k)) - s(i0(k));
    L.V = mean(Va(j));  L.gs = mean(gs(j));  L.thr = mean(thr(j));  L.I = mean(I(j));  L.P = mean(Pw(j));
    L.mAh = mAh(i1(k)) - mAh(i0(k));
    L.mAh_km = L.mAh / max(L.km, 1e-3);  L.km_Ah = 1000 / max(L.mAh_km, eps);
    L.Wh_km = (Wh(i1(k)) - Wh(i0(k))) / max(L.km, 1e-3);
    L.plan_thr = NaN;
    if useS, L.plan_thr = Sb.best_range.thr_fly; end
    Lg(end+1) = L; %#ok<AGROW>
end
nmL = {Lg.name};                                            % "back", "back 2", ... when a name repeats
for k = 1:nL
    same = find(strcmp(nmL, nmL{k}));
    if numel(same) > 1 && same(1) ~= k, Lg(k).name = sprintf('%s %d', nmL{k}, find(same == k)); end
end

%% ---- whole flight -------------------------------------------------------------------
iA = find(air, 1, 'first');  iZ = find(air, 1, 'last');
RC.tag = o.tag;  RC.legs = Lg;
RC.km_flown = s(end);  RC.km_air = 0;  if ~isempty(iA), RC.km_air = s(iZ) - s(iA); end
RC.mAh_used = mAh(end);  RC.Wh_used = Wh(end);  RC.SOC_end = soc(end);
RC.duration_min = (t(end) - t(1)) / 60;
RC.mAh_left = 1000 * P.batt.Ah * P.batt.SOC0 - mAh(end);
RC.km_per_Ah_all = RC.km_flown / max(RC.mAh_used / 1000, eps);
RC.cruise_mAh_km = NaN;  RC.cruise_km_Ah = NaN;  RC.cruise_Wh_km = NaN;  RC.range_to_reserve_km = NaN;
if nL
    RC.cruise_mAh_km = sum([Lg.mAh]) / sum([Lg.km]);
    RC.cruise_km_Ah  = 1000 / RC.cruise_mAh_km;
    RC.cruise_Wh_km  = sum([Lg.Wh_km] .* [Lg.km]) / sum([Lg.km]);
    RC.range_to_reserve_km = RC.km_flown + max(RC.mAh_left - o.reserve, 0) / RC.cruise_mAh_km;
end
RC.plan = struct();
if useS && isfield(Sb, 'batt_est') && isfield(Sb.batt_est, 'mAh')
    RC.plan.mAh = Sb.batt_est.mAh;  RC.plan.km = Sb.dist_m / 1000;
    RC.plan.err_pct = 100 * (RC.mAh_used - Sb.batt_est.mAh) / Sb.batt_est.mAh;
    RC.plan.mAh_here = Sb.batt_est.mAh;  RC.plan.partial = false;
    if isfield(Sb.batt_est, 'dist_leg') && RC.km_flown < 0.97 * RC.plan.km   % stopped early: same distance
        sp = [0; cumsum(Sb.batt_est.dist_leg(:))] / 1000;  mp = fillprev([0; Sb.batt_est.mAh_leg(:)]);
        [sp, iu] = unique(sp);  mp = mp(iu);
        RC.plan.mAh_here = interp1(sp, mp, min(RC.km_flown, sp(end)), 'linear');
        RC.plan.err_pct = 100 * (RC.mAh_used - RC.plan.mAh_here) / max(RC.plan.mAh_here, 1);
        RC.plan.partial = true;
    end
    RC.plan.V_fly = Sb.best_range.V_fly;  RC.plan.mAh_km = Sb.best_range.mAh_km_fly;
    RC.plan.Wh_km = Sb.best_range.P_fly / Sb.best_range.V_fly / 3.6;
end

%% ---- events ---------------------------------------------------------------------------
E = [];
if o.events && exist('swift_flight_events', 'file') == 2
    nm = {};  if useS && isfield(Sb, 'seg'), nm = Sb.seg(:, 1); end
    try, E = swift_flight_events(out, P, 'wp', o.wp, 'names', nm); catch, E = []; end
end

%% ---- figure ------------------------------------------------------------------------------
vis = 'on';  if ~o.show, vis = 'off'; end
C.fl = [0.00 0.36 0.70];  C.pl = [0.85 0.40 0.05];  C.rd = [0.80 0.10 0.10];  C.gr = [0.45 0.45 0.45];
C.leg = [0.30 0.70 0.35];  C.bk = [0.55 0.35 0.75];
f = figure('Name', 'swift_range_check', 'NumberTitle', 'off', 'Color', 'w', 'Visible', vis, ...
           'Position', [60 30 1200 1060]);
try, theme(f, 'light'); catch, end
ttl = sprintf('SWIFT range check  |  %s  |  %.1f km flown, %.0f mAh used, %.0f%% left', ...
              strrep(o.tag, '_', '\_'), RC.km_flown, RC.mAh_used, RC.SOC_end);
annotation(f, 'textbox', [0 0.958 1 0.035], 'String', ttl, 'EdgeColor', 'none', ...
           'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

% 1 charge vs distance
a1 = axes('Parent', f, 'Position', [0.07 0.715 0.88 0.195]);  hold(a1, 'on');
shade(a1, Lg, s, [0 1.12 * max([mAh; 1000 * P.batt.Ah - o.reserve])], C);
hp = [];
if useS && isfield(Sb, 'batt_est') && isfield(Sb.batt_est, 'dist_leg')
    sp = [0; cumsum(Sb.batt_est.dist_leg(:))] / 1000;
    mp = [0; Sb.batt_est.mAh_leg(:)];
    okp = ~isnan(mp);  okp(1) = true;
    mp = fillprev(mp);
    hp = plot(a1, sp(okp), mp(okp), '--o', 'Color', C.pl, 'LineWidth', 1.4, 'MarkerSize', 4, ...
              'MarkerFaceColor', C.pl, 'DisplayName', 'plan (swift\_sitl\_cub\_e)');
end
hf = plot(a1, s, mAh, '-', 'Color', C.fl, 'LineWidth', 2, 'DisplayName', 'flight');
fsx = 1000 * P.batt.Ah - o.reserve;
hr_ = plot(a1, [0 max(s(end), 1)], fsx * [1 1], ':', 'Color', C.rd, 'LineWidth', 1.6, ...
           'DisplayName', sprintf('low-battery RTL (%g mAh left)', o.reserve));
ylabel(a1, 'Charge used (mAh)');  grid(a1, 'on');  title(a1, 'Charge used against ground distance flown');
xlim(a1, [0 max(s(end), 0.1) * 1.01]);
legend(a1, [hf hp hr_], 'Location', 'southeast');
xlabel(a1, 'Ground distance flown (km)');

% 2 airspeed
a2 = axes('Parent', f, 'Position', [0.07 0.455 0.40 0.165]);  hold(a2, 'on');
shade(a2, Lg, s, [floor(min(Va(air | ~any(air))) - 1), ceil(max(Va) + 1)], C);
plot(a2, s, Va, '-', 'Color', C.fl, 'LineWidth', 1.2);
if useS
    plot(a2, [0 s(end)], Sb.best_range.V_fly * [1 1], '--', 'Color', C.pl, 'LineWidth', 1.3);
    text(a2, 0.01 * s(end), Sb.best_range.V_fly, sprintf(' best range %.1f m/s', Sb.best_range.V_fly), ...
         'Color', C.pl, 'VerticalAlignment', 'bottom', 'FontSize', 10);
end
ylabel(a2, 'Airspeed (m/s)');  grid(a2, 'on');  title(a2, 'Airspeed');
ylim(a2, [max(0, min(Va(air | ~any(air))) - 1), max(Va) + 1.5]);  xlim(a2, [0 max(s(end), 0.1)]);

% 3 throttle
a3 = axes('Parent', f, 'Position', [0.55 0.455 0.40 0.165]);  hold(a3, 'on');
shade(a3, Lg, s, [0 105], C);
plot(a3, s, thr, '-', 'Color', C.fl, 'LineWidth', 1.0);
for k = 1:nL
    plot(a3, s([Lg(k).i0 Lg(k).i1]), Lg(k).thr * [1 1], '-', 'Color', 'k', 'LineWidth', 2.2);
    if ~isnan(Lg(k).plan_thr)
        plot(a3, s([Lg(k).i0 Lg(k).i1]), Lg(k).plan_thr * [1 1], '--', 'Color', C.pl, 'LineWidth', 1.6);
    end
    if Lg(k).km > 0.12 * s(end)
        text(a3, mean(s([Lg(k).i0 Lg(k).i1])), Lg(k).thr + 7, sprintf('%s %.1f%%', Lg(k).name, Lg(k).thr), ...
             'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    end
end
ylabel(a3, 'Throttle (%)');  grid(a3, 'on');  ylim(a3, [0 105]);  xlim(a3, [0 max(s(end), 0.1)]);
title(a3, 'Throttle (black = leg mean, orange dashed = plan)');

% 4 current
a4 = axes('Parent', f, 'Position', [0.07 0.215 0.40 0.165]);  hold(a4, 'on');
Itop = max(5, min(max(I) * 1.05, 4 * max([Lg.I 10])));
shade(a4, Lg, s, [0 Itop], C);
plot(a4, s, I, '-', 'Color', C.fl, 'LineWidth', 1.0);
for k = 1:nL
    plot(a4, s([Lg(k).i0 Lg(k).i1]), Lg(k).I * [1 1], '-', 'Color', 'k', 'LineWidth', 2.2);
end
ylabel(a4, 'Pack current (A)');  xlabel(a4, 'Ground distance flown (km)');  grid(a4, 'on');
ylim(a4, [0 Itop]);  xlim(a4, [0 max(s(end), 0.1)]);
title(a4, sprintf('Pack current (axis cut at %.0f A; peak %.0f A at takeoff)', Itop, max(I)));

% 5 model curve with flown points (energy per km: does not drift with pack voltage, mAh/km does)
a5 = axes('Parent', f, 'Position', [0.55 0.215 0.40 0.165]);  hold(a5, 'on');
yy = [Lg.Wh_km];
if useS
    BR = Sb.best_range;
    whk = BR.P(:) ./ BR.V(:) / 3.6;                      % W / (m/s) = J/m -> Wh/km
    [~, jf] = min(abs(BR.V - BR.V_fly));
    plot(a5, BR.V, whk, '-', 'Color', C.gr, 'LineWidth', 1.8, 'DisplayName', 'model (level flight)');
    plot(a5, BR.V_fly, whk(jf), 'p', 'MarkerSize', 13, 'MarkerFaceColor', C.pl, ...
         'MarkerEdgeColor', 'k', 'DisplayName', sprintf('planned %.1f m/s', BR.V_fly));
    yy = [yy whk(:)'];
end
mk = {'o', 's', 'd', '^', 'v'};
for k = 1:nL
    c = C.fl;  if strncmp(Lg(k).name, 'back', 4), c = C.bk; end
    plot(a5, Lg(k).V, Lg(k).Wh_km, mk{min(k, 5)}, 'MarkerSize', 9, 'MarkerFaceColor', c, ...
         'MarkerEdgeColor', 'k', 'DisplayName', sprintf('flown: %s', Lg(k).name));
end
xlabel(a5, 'Airspeed (m/s)');  ylabel(a5, 'Energy per distance (Wh/km)');  grid(a5, 'on');
title(a5, 'Best-range curve and the flown legs');
if useS || nL, legend(a5, 'Location', 'southeast'); end
if ~isempty(yy)
    ylim(a5, [min(yy) - 0.25 * range_(yy) - 0.1, max(yy) + 0.15 * range_(yy) + 0.1]);
end
if useS, xlim(a5, [min(BR.V) - 0.5, max(BR.V) + 0.5]); end

% key
ln = {sprintf('%-6s %7s %7s %7s %7s %6s %7s %8s %7s %7s', 'leg', 'time s', 'km', 'V m/s', 'GS m/s', ...
              'thr %', 'I A', 'mAh/km', 'km/Ah', 'Wh/km')};
for k = 1:nL
    L = Lg(k);
    ln{end+1} = sprintf('%-6s %7.0f %7.2f %7.2f %7.2f %6.1f %7.2f %8.1f %7.2f %7.2f', L.name, L.t_s, ...
                        L.km, L.V, L.gs, L.thr, L.I, L.mAh_km, L.km_Ah, L.Wh_km); %#ok<AGROW>
end
if ~nL, ln{end+1} = 'No steady cruise leg of 60 s or more in this log.'; end
ln{end+1} = sprintf('Whole flight: %.2f km in %.1f min, %.0f mAh / %.0f Wh used, %.2f km/Ah overall', ...
                    RC.km_flown, RC.duration_min, RC.mAh_used, RC.Wh_used, RC.km_per_Ah_all);
if nL
    ln{end+1} = sprintf('Cruise legs: %.2f km/Ah, %.2f Wh/km   |   still-air range to the %g mAh reserve ~%.1f km', ...
                        RC.cruise_km_Ah, RC.cruise_Wh_km, o.reserve, RC.range_to_reserve_km);
end
if isfield(RC.plan, 'mAh')
    if RC.plan.partial
        ln{end+1} = sprintf(['Plan: %.0f mAh by %.1f km (flight stopped early; full plan %.0f mAh over %.1f km)' ...
                             '  ->  flight used %+.1f %%  |  model cruise %.2f Wh/km'], RC.plan.mAh_here, ...
                            RC.km_flown, RC.plan.mAh, RC.plan.km, RC.plan.err_pct, RC.plan.Wh_km);
    else
        ln{end+1} = sprintf('Plan: %.0f mAh over %.1f km  ->  flight used %+.1f %% vs plan  |  model cruise %.2f Wh/km at %.1f m/s', ...
                            RC.plan.mAh, RC.plan.km, RC.plan.err_pct, RC.plan.Wh_km, RC.plan.V_fly);
    end
end
ak = axes('Parent', f, 'Position', [0.07 0.005 0.88 0.135], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
text(ak, 0, 1, ln, 'FontName', 'Consolas', 'FontSize', 10, 'VerticalAlignment', 'top', 'Color', 'k', ...
     'Interpreter', 'none');

% events
if ~isempty(E) && isfield(E, 't') && ~isempty(E.t) && exist('swift_mark_events', 'file') == 2
    w = 1:numel(t);
    swift_mark_events(a1, E, 'x', s, 'win', w, 'labels', true);
    for ax = [a2 a3 a4], swift_mark_events(ax, E, 'x', s, 'win', w, 'labels', false); end
end
readable(f);

%% ---- save / print ------------------------------------------------------------------------
fprintf('\n  SWIFT range check  (%s)\n', o.tag);
fprintf('  %s\n', ln{:});
if o.save
    dd = fullfile(fileparts(fileparts(here)), 'data', 'runs', o.tag);   % sitl/analysis -> ArduPilot_SITL/data/runs
    if ~isfolder(dd), mkdir(dd); end
    try
        exportgraphics(f, fullfile(dd, 'swift_range_check.png'), 'Resolution', 200);
    catch
        saveas(f, fullfile(dd, 'swift_range_check.png'));
    end
    fid = fopen(fullfile(dd, 'range_summary.txt'), 'w');
    if fid > 0, fprintf(fid, '%s\n', ln{:});  fclose(fid); end
    fid = fopen(fullfile(dd, 'range_legs.csv'), 'w');
    if fid > 0
        fprintf(fid, 'leg,t_start_s,dur_s,km,V_ms,GS_ms,throttle_pct,I_A,P_W,mAh,mAh_per_km,km_per_Ah,Wh_per_km\n');
        for k = 1:nL
            L = Lg(k);
            fprintf(fid, '%s,%.1f,%.1f,%.3f,%.3f,%.3f,%.2f,%.3f,%.1f,%.1f,%.2f,%.3f,%.3f\n', L.name, ...
                    t(L.i0), L.t_s, L.km, L.V, L.gs, L.thr, L.I, L.P, L.mAh, L.mAh_km, L.km_Ah, L.Wh_km);
        end
        fclose(fid);
    end
    fprintf('  Saved to %s\n\n', dd);
end
if ~o.show, close(f); end
if nargout == 0, clear RC; end
end

% =========================================================================
function shade(ax, Lg, s, yl, C)
for k = 1:numel(Lg)
    c = C.leg;  if strncmp(Lg(k).name, 'back', 4), c = C.bk; end
    x = s([Lg(k).i0 Lg(k).i1]);
    patch(ax, [x(1) x(2) x(2) x(1)], yl([1 1 2 2]), c, 'FaceAlpha', 0.10, 'EdgeColor', 'none', ...
          'HandleVisibility', 'off');
end
end

function r = range_(v)
r = max(v) - min(v);
end

function v = fillprev(v)
for k = 2:numel(v), if isnan(v(k)), v(k) = v(k-1); end, end
end

function y = rs(out, name, t, n)
[ts, ys] = sig(out, name);
if isempty(ts), y = nan(numel(t), n);  return, end
if size(ys, 2) < n, ys(:, end+1:n) = NaN; end
y = interp1(ts, ys(:, 1:n), t, 'linear', 'extrap');
end

function [t, y] = sig(o, name)
t = [];  y = [];  s = [];
if isa(o, 'Simulink.SimulationOutput')
    try, s = o.get(name); catch, s = []; end
elseif isstruct(o) && isfield(o, name)
    s = o.(name);
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
%  Black text on white, whatever MATLAB theme is on (same as swift_plot_flight).
if ~isgraphics(f), return; end
try, theme(f, 'light'); catch, end
k = [0 0 0];
set(f, 'Color', 'w');
for a = findall(f, 'Type', 'axes')'
    if strcmp(get(a, 'Visible'), 'off'), continue, end
    if ~(ischar(get(a, 'Color')) && strcmp(get(a, 'Color'), 'none')), set(a, 'Color', 'w'); end
    set(a, 'XColor', k, 'YColor', k, 'ZColor', k, 'GridColor', [0.15 0.15 0.15], 'FontSize', max(get(a, 'FontSize'), 11));
    set(get(a, 'Title'), 'Color', k, 'FontSize', 12);
end
set(findall(f, 'Type', 'legend'), 'TextColor', k, 'Color', 'w', 'EdgeColor', [0.6 0.6 0.6]);
end
