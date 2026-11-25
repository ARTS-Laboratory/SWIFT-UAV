function VT = swift_vtail_check(out, varargin)
%SWIFT_VTAIL_CHECK  Check the inverted-V tail in one swift_6dof_sitl (or swift_6dof) run.
%
%   swift_vtail_check                     'out' from the base workspace
%   VT = swift_vtail_check(out, Name, Value, ...)
%
%   OPTIONS
%     'tag'     auto    run folder data/runs/<tag>/ (same as swift_plot_flight)
%     'save'    true    write swift_vtail_check.png and vtail_summary.txt
%     'show'    true    false = build and save without showing
%     'wp'      auto    .waypoints file for the event markers (default S.wp_file)
%     'events'  true    numbered event markers (same numbers as every figure)
%
%   WHAT IT CHECKS  (PASS / WARN / FAIL, printed and drawn under the figure)
%     1 Logged       out_rv = [left; right] ruddervator angles is in the run
%                    (swift_vtail_add puts the logger in the V-tail limit block).
%     2 Mixing       the elevator and rudder the 6-DOF flew on (out_delta) are the
%                    un-mix of the two logged ruddervators:
%                        delta_e = (rv_L + rv_R)/2,   delta_r = (rv_L - rv_R)/2
%                    PASS below 0.05 deg everywhere. A mismatch means the V-tail
%                    limit block is not between the commands and the aero.
%     3 Travel       share of airborne time a ruddervator sits on its stop
%                    (+/- P.tail.drv_max). PASS < 1 %, WARN < 5 %.
%     4 Pitch sign   at takeoff rotation both ruddervators go trailing-edge UP
%                    (delta_e < 0, nose-up for Cm_de < 0) while the nose comes up.
%     5 Yaw sign     ArduPlane mixes rudder with aileron (KFF_RDDRMIX). The yaw
%                    the ruddervators make (Cn_dr * delta_r) must have the same
%                    sign as the roll the ailerons make (Cl_da * delta_a): rudder
%                    INTO the turn. Opposite sign = left/right swapped:
%                    swap SERVO2_FUNCTION 79 and SERVO4_FUNCTION 80.
%     6 Straight     on wings-level stretches the two ruddervators sit together
%                    (|delta_r| < 1 deg): no built-in yaw from the mixing.
%     Also reported: peak sideslip in turns, peak deflections, most travel used.
%
%   FIGURE  swift_vtail_check
%     1  Left and right ruddervator against time, travel stops, turns shaded
%     2  delta_e and delta_r flown (solid) over the un-mix of out_rv (dashed)
%     3  V-tail travel envelope: delta_e against delta_r with the diamond
%        |delta_e +/- delta_r| <= drv_max the two surfaces allow
%     4  Yaw coordination: Cn_dr*delta_r against Cl_da*delta_a, fitted line
%     5  Pitch trim: delta_e against angle of attack on steady flight, model
%        trim line  delta_e = -(Cm0 + Cm_alpha*alpha)/Cm_de
%
%   Sign convention (as the model): positive ruddervator = trailing edge DOWN.
%
%   Ryan Bilock, ARTS Lab | V-tail | v0.1

%% ---- inputs -----------------------------------------------------------------
if nargin < 1 || isempty(out)
    if evalin('base', 'exist(''out'',''var'')'), out = evalin('base', 'out'); else, out = []; end
end
here = fileparts(mfilename('fullpath'));
o.tag = ['run_' datestr(now, 'yyyymmdd_HHMM')];  o.save = true;  o.show = true;  o.wp = '';  o.events = true;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
Sb = [];
if evalin('base', 'exist(''S'',''var'')'), Sb = evalin('base', 'S'); end
if isempty(o.wp) && isstruct(Sb) && isfield(Sb, 'wp_file') && isfile(Sb.wp_file), o.wp = Sb.wp_file; end
if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
P = evalin('base', 'P');
lim = deg2rad(25);  if isfield(P, 'tail') && isfield(P.tail, 'drv_max'), lim = P.tail.drv_max; end
limd = rad2deg(lim);

%% ---- signals --------------------------------------------------------------------
[t, pos] = sig(out, 'out_pos');
if numel(t) < 10, error('swift_vtail_check:nolog', 'No out_pos found - run the model first.'); end
dl  = rs(out, 'out_delta', t, 4);  om = rs(out, 'out_om', t, 3);  eul = rs(out, 'out_euler', t, 3);
al  = rs(out, 'out_alpha', t, 1);  be = rs(out, 'out_beta', t, 1);  Va = rs(out, 'out_V', t, 1);
[trv, yrv] = sig(out, 'out_rv');
hasRV = ~isempty(trv) && size(yrv, 2) >= 2;
if hasRV
    rv = interp1(trv, yrv(:, 1:2), t, 'linear', 'extrap');
else
    rv = [dl(:,2) + dl(:,3), dl(:,2) - dl(:,3)];          % what the mixing would give
    rv = min(max(rv, -lim), lim);
end
D   = @(x) rad2deg(x);
tm  = t / 60;
h   = -pos(:,3);  ha = h - h(1);
dt  = median(diff(t));
air = ha > 3 & Va > 0.8 * P.perf.V_stall;
phi = D(eul(:,1));
turn = air & abs(phi) > 8;
vz  = movmean(gradient(h, t), max(1, round(3 / dt)));
lvl = air & abs(phi) < 3 & abs(vz) < 0.4;
de  = dl(:,2);  dr = dl(:,3);  da = dl(:,1);
deU = (rv(:,1) + rv(:,2)) / 2;  drU = (rv(:,1) - rv(:,2)) / 2;

%% ---- checks ---------------------------------------------------------------------------
C = struct('name', {}, 'res', {}, 'txt', {});
add = @(C, n, r, s) [C, struct('name', n, 'res', r, 'txt', s)];

% 1 logged
if hasRV
    C = add(C, 'Logged', 'PASS', sprintf('out_rv in the run (%d samples)', numel(trv)));
else
    C = add(C, 'Logged', 'FAIL', 'no out_rv - run swift_vtail_add, then fly again (plots below use the mix of out_delta)');
end

% 2 mixing identity
if hasRV
    eM = max(abs(D([deU - de; drU - dr])));
    if eM < 0.05, r = 'PASS'; elseif eM < 0.5, r = 'WARN'; else, r = 'FAIL'; end
    C = add(C, 'Mixing', r, sprintf('delta_e, delta_r flown = un-mix of the ruddervators (worst %.3f deg)', eM));
else
    eM = NaN;
    C = add(C, 'Mixing', 'n/a', 'needs out_rv');
end

% 3 travel
onstop = any(abs(rv) >= lim - deg2rad(0.2), 2);
pS = 100 * sum(onstop & air) / max(sum(air), 1);
mx = max(abs(D(rv)), [], 1);                           % whole log, ground included
if pS < 1, r = 'PASS'; elseif pS < 5, r = 'WARN'; else, r = 'FAIL'; end
C = add(C, 'Travel', r, sprintf('on a stop %.2f %% of airborne time; peak L %.1f / R %.1f of %.0f deg (whole log)', pS, mx(1), mx(2), limd));

% 4 pitch sign at rotation
q = D(om(:,2));
gnd = ha < 2;
iR = find(gnd & q > 2 & Va > 0.7 * P.perf.V_stall, 1, 'first');
if ~isempty(iR)
    w = max(1, iR - round(1 / dt)):min(numel(t), iR + round(1 / dt));
    deR = D(mean(de(w)));
    if deR < 0
        C = add(C, 'Pitch sign', 'PASS', sprintf('takeoff rotation at %.2f min: both surfaces TE up, delta_e %.1f deg, nose up %.1f deg/s', tm(iR), deR, max(q(w))));
    else
        C = add(C, 'Pitch sign', 'FAIL', sprintf('rotation with delta_e %+.1f deg (TE down) - pitch sign reversed (P.sitl.sgn(2))', deR));
    end
else
    deR = NaN;
    C = add(C, 'Pitch sign', 'n/a', 'no takeoff rotation in this log');
end

% 5 yaw sign: rudder with aileron
Cnd = P.aero.Cndr;  Cla = P.aero.Clda;
ny = Cnd * dr;  rl = Cla * da;
m5 = air & abs(D(da)) > 0.3 & abs(D(dr)) > 0.05;
slope = NaN;  agree = NaN;
if sum(m5) > 50
    slope = (rl(m5)' * ny(m5)) / max(rl(m5)' * rl(m5), eps);
    agree = 100 * mean(sign(ny(m5)) == sign(rl(m5)));
    if slope > 0 && agree > 60
        C = add(C, 'Yaw sign', 'PASS', sprintf('rudder into the turn: yaw/roll slope %+.2f, same sign %.0f %% of %d samples', slope, agree, sum(m5)));
    elseif slope > 0
        C = add(C, 'Yaw sign', 'WARN', sprintf('slope %+.2f but same sign only %.0f %% - weak rudder mixing', slope, agree));
    else
        C = add(C, 'Yaw sign', 'FAIL', sprintf('rudder AGAINST the turn (slope %+.2f): swap SERVO2_FUNCTION 79 and SERVO4_FUNCTION 80', slope));
    end
else
    C = add(C, 'Yaw sign', 'n/a', 'too little aileron and rudder together to judge (KFF_RDDRMIX = 0?)');
end

% 6 straight flight
if sum(lvl) > 50
    mdr = D(mean(dr(lvl)));  mdiff = D(mean(rv(lvl,1) - rv(lvl,2)));
    if abs(mdr) < 1, r = 'PASS'; elseif abs(mdr) < 3, r = 'WARN'; else, r = 'FAIL'; end
    C = add(C, 'Straight', r, sprintf('wings level: delta_r %+.2f deg mean, left - right %+.2f deg, delta_e %+.2f deg', ...
                                      mdr, mdiff, D(mean(de(lvl)))));
else
    mdr = NaN;
    C = add(C, 'Straight', 'n/a', 'no steady wings-level flight in this log');
end
bT = NaN;  if any(turn), bT = max(abs(D(be(turn)))); end
VT.checks = C;  VT.has_rv = hasRV;  VT.mix_err_deg = eM;  VT.pct_on_stop = pS;  VT.peak_deg = mx;
VT.de_rotation_deg = deR;  VT.yaw_slope = slope;  VT.yaw_agree_pct = agree;  VT.dr_level_deg = mdr;
VT.beta_turn_max_deg = bT;  VT.travel_used_pct = 100 * max(abs(rv(:))) / lim;
VT.pass = ~any(strcmp({C.res}, 'FAIL'));

%% ---- events ---------------------------------------------------------------------------
E = [];
if o.events && exist('swift_flight_events', 'file') == 2
    nm = {};  if isstruct(Sb) && isfield(Sb, 'seg') && isfield(Sb, 'wp_file') && strcmp(Sb.wp_file, o.wp), nm = Sb.seg(:, 1); end
    try, E = swift_flight_events(out, P, 'wp', o.wp, 'names', nm); catch, E = []; end
end

%% ---- figure ------------------------------------------------------------------------------
vis = 'on';  if ~o.show, vis = 'off'; end
cL = [0.00 0.40 0.75];  cR = [0.85 0.33 0.10];  cG = [0.45 0.45 0.45];  cS = [0.80 0.10 0.10];
f = figure('Name', 'swift_vtail_check', 'NumberTitle', 'off', 'Color', 'w', 'Visible', vis, ...
           'Position', [60 30 1200 1060]);
try, theme(f, 'light'); catch, end
verdict = 'ALL CHECKS PASS';  if ~VT.pass, verdict = 'CHECK FAILED - see the key'; end
annotation(f, 'textbox', [0 0.958 1 0.035], 'String', ...
           sprintf('SWIFT inverted-V tail check  |  %s  |  %s', strrep(o.tag, '_', '\_'), verdict), ...
           'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

% 1 ruddervators against time
a1 = axes('Parent', f, 'Position', [0.07 0.745 0.88 0.165]);  hold(a1, 'on');
pk1 = max([abs(D(rv(:))); 1]);
zoomed = 1.3 * pk1 + 2 < 1.1 * limd;                   % small deflections: zoom in, say where the stops are
if zoomed, yl = (1.3 * pk1 + 2) * [-1 1]; else, yl = [-1.15 1.15] * limd; end
shade_mask(a1, tm, turn, yl, [0.85 0.85 0.85]);
plot(a1, tm, D(rv(:,1)), '-', 'Color', cL, 'LineWidth', 1.1, 'DisplayName', 'left ruddervator');
plot(a1, tm, D(rv(:,2)), '-', 'Color', cR, 'LineWidth', 1.1, 'DisplayName', 'right ruddervator');
if zoomed
    text(a1, tm(end), yl(1), sprintf('axis zoomed: travel stops at \\pm%.0f%s are off scale  ', limd, '^{\circ}'), ...
         'Color', cS, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 9);
else
    plot(a1, tm([1 end]), [limd limd], '--', 'Color', cS, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(a1, tm([1 end]), -[limd limd], '--', 'Color', cS, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    text(a1, tm(1), limd, sprintf(' stop +%.0f%s (TE down)', limd, '^{\circ}'), 'Color', cS, 'VerticalAlignment', 'bottom', 'FontSize', 9);
    text(a1, tm(1), -limd, sprintf(' stop -%.0f%s (TE up)', limd, '^{\circ}'), 'Color', cS, 'VerticalAlignment', 'top', 'FontSize', 9);
end
ylim(a1, yl);  xlim(a1, [tm(1) max(tm(end), tm(1) + 0.1)]);  grid(a1, 'on');
ylabel(a1, sprintf('Deflection (%s)', '^{\circ}'));  xlabel(a1, 'Time from start of log (min)');
ttl1 = 'Ruddervators  (grey = turning, bank > 8\circ)';
if ~hasRV, ttl1 = [ttl1 '  -  NO out\_rv: shown from the mix of out\_delta']; end
title(a1, ttl1);  legend(a1, 'Location', 'southeast', 'Orientation', 'horizontal');

% 2 identity
a2 = axes('Parent', f, 'Position', [0.07 0.485 0.50 0.165]);  hold(a2, 'on');
plot(a2, tm, D(de), '-', 'Color', cL, 'LineWidth', 1.4, 'DisplayName', '\delta_e flown');
plot(a2, tm, D(dr), '-', 'Color', cR, 'LineWidth', 1.4, 'DisplayName', '\delta_r flown');
plot(a2, tm, D(deU), '--', 'Color', 'k', 'LineWidth', 0.8, 'DisplayName', '(L+R)/2');
plot(a2, tm, D(drU), ':', 'Color', 'k', 'LineWidth', 1.0, 'DisplayName', '(L-R)/2');
grid(a2, 'on');  xlim(a2, [tm(1) max(tm(end), tm(1) + 0.1)]);
ylabel(a2, sprintf('Angle (%s)', '^{\circ}'));  xlabel(a2, 'Time from start of log (min)');
ttl2 = 'Elevator / rudder flown vs un-mixed ruddervators';
if ~isnan(eM), ttl2 = sprintf('%s  (worst %.3f%s)', ttl2, eM, '^{\circ}'); end
title(a2, ttl2);  legend(a2, 'Location', 'southeast', 'Orientation', 'horizontal');

% 3 envelope
a3 = axes('Parent', f, 'Position', [0.665 0.465 0.24 0.20]);  hold(a3, 'on');
patch(a3, [limd 0 -limd 0], [0 limd 0 -limd], [0.92 0.96 0.92], 'EdgeColor', [0.2 0.55 0.2], 'LineWidth', 1.4);
jj = find(air);  if isempty(jj), jj = 1:numel(t); end
jj = jj(1:max(1, floor(numel(jj) / 4000)):end);
scatter(a3, D(dr(jj)), D(de(jj)), 6, tm(jj), 'filled');
colormap(a3, parula_or_jet());
plot(a3, 0, 0, '+', 'Color', 'k', 'MarkerSize', 8);
axis(a3, 'equal');  xlim(a3, 1.1 * [-limd limd]);  ylim(a3, 1.1 * [-limd limd]);  grid(a3, 'on');
xlabel(a3, sprintf('\\delta_r (%s)', '^{\circ}'));  ylabel(a3, sprintf('\\delta_e (%s)', '^{\circ}'));
title(a3, sprintf('Travel envelope (%.0f %% used)', VT.travel_used_pct));
text(a3, 0, 0.62 * limd, '|\delta_e \pm \delta_r| \leq stop', 'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.2 0.45 0.2]);
cb = colorbar(a3);  try, cb.Label.String = 'min'; catch, ylabel(cb, 'min'); end

% 4 yaw coordination
a4 = axes('Parent', f, 'Position', [0.07 0.225 0.40 0.165]);  hold(a4, 'on');
mm = find(air);  if isempty(mm), mm = 1:numel(t); end
mm = mm(1:max(1, floor(numel(mm) / 4000)):end);
xr = 1000 * rl(mm);  yr = 1000 * ny(mm);
xm = max([abs(xr); 1e-3]);  ym = max([abs(yr); 1e-3]);
patch(a4, [0 xm xm 0] * 1.1, [0 0 ym ym] * 1.1, [0.92 0.96 0.92], 'EdgeColor', 'none', 'HandleVisibility', 'off');
patch(a4, -[0 xm xm 0] * 1.1, -[0 0 ym ym] * 1.1, [0.92 0.96 0.92], 'EdgeColor', 'none', 'HandleVisibility', 'off');
scatter(a4, xr, yr, 6, cG, 'filled', 'DisplayName', 'airborne samples');
if ~isnan(slope)
    xx = [-xm xm] * 1.05;
    plot(a4, xx, slope * xx, '-', 'Color', 'k', 'LineWidth', 1.6, 'DisplayName', sprintf('fit, slope %+.2f', slope));
end
xlim(a4, 1.1 * [-xm xm]);  ylim(a4, 1.1 * [-ym ym]);  grid(a4, 'on');
xlabel(a4, 'Roll from ailerons  C_{l\delta a}\delta_a  (\times 10^{-3})');
ylabel(a4, 'Yaw from V-tail  C_{n\delta r}\delta_r  (\times 10^{-3})');
title(a4, 'Yaw coordination (green = rudder into the turn)');
if ~isnan(slope), legend(a4, 'Location', 'northwest'); end

% 5 pitch trim
a5 = axes('Parent', f, 'Position', [0.57 0.225 0.38 0.165]);  hold(a5, 'on');
lv = find(lvl);  if isempty(lv), lv = find(air); end
if ~isempty(lv)
    lv = lv(1:max(1, floor(numel(lv) / 3000)):end);
    scatter(a5, D(al(lv)), D(de(lv)), 7, [0.00 0.40 0.75], 'filled', 'DisplayName', 'steady level flight');
    aa = linspace(min(al(lv)) - deg2rad(1), max(al(lv)) + deg2rad(1), 50);
    if isfield(P.aero, 'Cm0') && isfield(P.aero, 'Cmalpha') && isfield(P.aero, 'Cmde')
        plot(a5, D(aa), D(-(P.aero.Cm0 + P.aero.Cmalpha * aa) / P.aero.Cmde), '--', 'Color', 'k', ...
             'LineWidth', 1.4, 'DisplayName', 'model trim line (C_m = 0)');
    end
    legend(a5, 'Location', 'northeast');
end
grid(a5, 'on');  xlabel(a5, sprintf('Angle of attack (%s)', '^{\circ}'));  ylabel(a5, sprintf('\\delta_e (%s)', '^{\circ}'));
title(a5, 'Pitch trim from the V-tail');

% key
ak = axes('Parent', f, 'Position', [0.07 0.005 0.88 0.15], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
hold(ak, 'on');
nC = numel(C);
for k = 1:nC
    y = 1 - (k - 0.5) / (nC + 1);
    switch C(k).res
        case 'PASS', c = [0.10 0.55 0.15];
        case 'WARN', c = [0.85 0.55 0.00];
        case 'FAIL', c = [0.80 0.10 0.10];
        otherwise,   c = [0.45 0.45 0.45];
    end
    text(ak, 0.000, y, sprintf('%-4s', C(k).res), 'Color', c, 'FontWeight', 'bold', 'FontName', 'Consolas', 'FontSize', 10.5);
    text(ak, 0.055, y, sprintf('%-11s %s', C(k).name, C(k).txt), 'Color', 'k', 'FontName', 'Consolas', ...
         'FontSize', 10, 'Interpreter', 'none');
end
bTs = '-';  if ~isnan(bT), bTs = sprintf('%.1f deg', bT); end
extra = sprintf(['Peak sideslip in turns %s  |  peak delta_e %.1f deg, delta_r %.1f deg (whole log)  |  ' ...
                 'mixing gain %.2g, stops +/- %.0f deg'], bTs, max(abs(D(de))), max(abs(D(dr))), ...
                 getfield_or(P, {'tail', 'mix_gain'}, NaN), limd);
text(ak, 0.055, 1 - (nC + 0.5) / (nC + 1), extra, 'Color', [0.25 0.25 0.25], 'FontName', 'Consolas', ...
     'FontSize', 9.5, 'Interpreter', 'none');

if ~isempty(E) && isfield(E, 't') && ~isempty(E.t) && exist('swift_mark_events', 'file') == 2
    swift_mark_events(a1, E, 'scale', 1/60, 'labels', true);
    swift_mark_events(a2, E, 'scale', 1/60, 'labels', false);
end
readable(f);

%% ---- print / save ------------------------------------------------------------------------
fprintf('\n  SWIFT V-tail check  (%s)\n', o.tag);
for k = 1:nC, fprintf('  %-4s  %-11s %s\n', C(k).res, C(k).name, C(k).txt); end
fprintf('        %s\n', extra);
if o.save
    dd = fullfile(fileparts(fileparts(here)), 'data', 'runs', o.tag);   % sitl/analysis -> ArduPilot_SITL/data/runs
    if ~isfolder(dd), mkdir(dd); end
    try
        exportgraphics(f, fullfile(dd, 'swift_vtail_check.png'), 'Resolution', 200);
    catch
        saveas(f, fullfile(dd, 'swift_vtail_check.png'));
    end
    fid = fopen(fullfile(dd, 'vtail_summary.txt'), 'w');
    if fid > 0
        for k = 1:nC, fprintf(fid, '%-4s  %-11s %s\n', C(k).res, C(k).name, C(k).txt); end
        fprintf(fid, '%s\n', extra);  fclose(fid);
    end
    fprintf('  Saved to %s\n\n', dd);
end
if ~o.show, close(f); end
if nargout == 0, clear VT; end
end

% =========================================================================
function shade_mask(ax, x, m, yl, c)
d = diff([0; m(:); 0]);  i0 = find(d == 1);  i1 = find(d == -1) - 1;
for k = 1:numel(i0)
    patch(ax, x([i0(k) i1(k) i1(k) i0(k)]), yl([1 1 2 2]), c, 'FaceAlpha', 0.45, 'EdgeColor', 'none', ...
          'HandleVisibility', 'off');
end
end

function v = getfield_or(S, p, d)
v = d;
try, v = S.(p{1}).(p{2}); catch, end
end

function m = parula_or_jet()
try, m = parula(256); catch, m = jet(256); end
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
set(findall(f, 'Type', 'colorbar'), 'Color', k);
end
