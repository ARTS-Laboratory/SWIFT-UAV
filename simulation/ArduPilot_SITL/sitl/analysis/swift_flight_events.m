function E = swift_flight_events(out, P, varargin)
%SWIFT_FLIGHT_EVENTS  Numbered list of what happened, and when, in one swift_6dof_sitl run.
%
%   E = swift_flight_events(out, P)
%   E = swift_flight_events(out, P, Name, Value, ...)
%
%   Used by swift_plot_flight, swift_battery_plot and swift_motor_check to put
%   the same numbered markers on every graph, and by swift_events_figure to
%   print the list. Works for any flight: the mission items need a .waypoints
%   file; the flight events (throttle up, liftoff, touchdown, ...) come from the
%   logged signals alone.
%
%   OPTIONS
%     'wp'         ''     .waypoints file the flight used (QGC WPL 110)
%     'names'      {}     optional item names, names{k} = mission item k
%                         (swift_sitl_cub_d gives them as S.seg(:,1))
%     'wp_radius'  90     m, ArduPlane WP_RADIUS: a waypoint counts as reached
%                         inside this distance
%     'pass_max'   250    m, a waypoint never closer than this is "not reached"
%
%   MISSION ITEMS (from the .waypoints file, timed from the flight)
%     TAKEOFF          time the takeoff altitude is reached
%     WAYPOINT         first time inside wp_radius (or the closest pass)
%     LOITER_TURNS     time the circle is joined
%     LOITER_TO_ALT    time the circle is joined
%     LAND             time the final approach starts (previous item done)
%     DO items (speed change, landing-sequence start) are folded into the
%     row of the item before them, the moment ArduPlane runs them.
%   If fewer than 60 % of the positioned items are reached, the file does not
%   match this flight and the mission rows are dropped (E.note says so).
%
%   FLIGHT EVENTS (always)
%     throttle up, liftoff, highest point, peak pack current, first time in
%     each battery warning state, touchdown, stopped.
%
%   E (one row per event, sorted by time, numbered 1..N)
%     t, idx            s and sample index on the out_pos time base
%     num               marker number
%     kind              'mission' | 'flight' | 'battery'
%     item              mission item text ('#5') or ''
%     what              one-line description
%     n, e, alt         m north / east of home, m above the start point
%     V, thr, I, SOC    m/s, %, A, %  at that moment (NaN when not logged)
%     n_mission, n_flight, note, wp
%     tr_t, tr_alt      s, m: the whole altitude trace (for swift_events_figure)
%
%   Ryan Bilock, ARTS Lab | SITL plotting | v0.3
%     v0.3  timed loiter shown in minutes
%     v0.2  adds altitude trace

o.wp = '';  o.names = {};  o.wp_radius = 90;  o.pass_max = 250;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end

E = empty_E();  E.wp = o.wp;
[t, pos] = sig(out, 'out_pos');
if isempty(t), E.note = 'no out_pos in this run'; return, end
V   = resample_to(out, 'out_V', t, 1);
dl  = resample_to(out, 'out_delta', t, 4);
[tb, yb] = sig(out, 'out_batt');
hasB = ~isempty(tb) && size(yb, 2) == 10;
if hasB
    bt = interp1(tb, yb, t, 'linear', 'extrap');
else
    bt = nan(numel(t), 10);
end
h   = -pos(:,3);  h0 = h(1);  ha = h - h0;
E.tr_t = t;  E.tr_alt = ha;                % altitude trace for swift_events_figure
thr = dl(:,4);
gs  = hypot(gradient(pos(:,1), t), gradient(pos(:,2), t));
dist = [0; cumsum(hypot(diff(pos(:,1)), diff(pos(:,2))))];
Vs  = 18;  if isfield(P, 'perf') && isfield(P.perf, 'V_stall'), Vs = P.perf.V_stall; end

rows = {};                 % {idx, kind, item, what}
add  = @(i, kind, item, what) {i, kind, item, what};

%% ---- flight events ---------------------------------------------------------
onGnd0 = ha(1) < 1 && h0 < 1e3;
iUp = find(thr > 0.3, 1);
air = ha > 0.5;
iLift = [];  iTD = [];  iStop = [];
if onGnd0
    if ~isempty(iUp)
        rows{end+1} = add(iUp, 'flight', '', 'Throttle up: takeoff roll begins');
    end
    iLift = find(air & V > 0.9*Vs, 1);
    if ~isempty(iLift)
        x0 = 0;  if ~isempty(iUp), x0 = dist(iUp); end
        rows{end+1} = add(iLift, 'flight', '', sprintf('Liftoff at %.1f m/s, %.0f m from brake release', ...
                          V(iLift), dist(iLift) - x0));
        iLast = find(air, 1, 'last');
        if ~isempty(iLast) && iLast > iLift && iLast < numel(t)
            iTD = iLast + 1;
            rows{end+1} = add(iTD, 'flight', '', sprintf('Touchdown at %.1f m/s', V(iTD)));
            iStop = find(t > t(iTD) & gs < 0.5, 1);
            if ~isempty(iStop)
                rows{end+1} = add(iStop, 'flight', '', sprintf('Stopped, %.0f m rollout', dist(iStop) - dist(iTD)));
            end
        end
    end
end
[hmax, iMax] = max(ha);
if hmax > 5
    rows{end+1} = add(iMax, 'flight', '', sprintf('Highest point, %.0f m', hmax));
end
if hasB
    [Imax, iI] = max(bt(:,3));
    rows{end+1} = add(iI, 'battery', '', sprintf('Peak pack current, %.0f A', Imax));
    st = round(bt(:,10));
    msg = {'Battery below the reserve line', 'Battery LOW: cell voltage under the low limit', ...
           'Battery CRITICAL: cell voltage under the critical limit', 'Pack overloaded (asked for more than it can give)', ...
           'Battery EMPTY (state of charge 0)'};
    for s = 1:5
        i = find(st == s, 1);
        if ~isempty(i), rows{end+1} = add(i, 'battery', '', msg{s}); end %#ok<AGROW>
    end
end

%% ---- mission items --------------------------------------------------------------
M = read_items(o.wp);
nMis = 0;
if ~isempty(M)
    [mrows, nReach, nPos] = time_items(M, t, pos, ha, V, thr, iUp, iLift, iTD, o);
    if nPos == 0 || nReach / nPos >= 0.6
        rows = [rows, mrows];  nMis = numel(mrows);
    else
        E.note = sprintf('%s: only %d of %d positioned items reached, so it does not match this flight; mission rows left out', ...
                         short(o.wp), nReach, nPos);
    end
    if nReach < nPos && isempty(E.note)
        E.note = sprintf('%d of %d positioned mission items not reached (run stopped early or RTL)', nPos - nReach, nPos);
    end
end

%% ---- names ------------------------------------------------------------------
if ~isempty(o.names)
    for r = 1:numel(rows)
        tok = regexp(rows{r}{3}, '^#(\d+)', 'tokens', 'once');
        if ~isempty(tok)
            k = str2double(tok{1});
            if k >= 1 && k <= numel(o.names) && ~isempty(o.names{k})
                rows{r}{4} = [o.names{k} ' - ' rows{r}{4}];
            end
        end
    end
end

%% ---- assemble, sort, number --------------------------------------------------------
if isempty(rows), return, end
idx = cellfun(@(r) r{1}, rows);
[idx, ord] = sort(idx(:));  rows = rows(ord);
N = numel(idx);
E.idx = idx;  E.t = t(idx);  E.num = (1:N)';
E.kind = cellfun(@(r) r{2}, rows, 'UniformOutput', false)';
E.item = cellfun(@(r) r{3}, rows, 'UniformOutput', false)';
E.what = cellfun(@(r) r{4}, rows, 'UniformOutput', false)';
E.n = pos(idx, 1);  E.e = pos(idx, 2);  E.alt = ha(idx);  E.V = V(idx);
E.thr = 100 * thr(idx);  E.I = bt(idx, 3);  E.SOC = 100 * bt(idx, 1);
E.n_mission = nMis;  E.n_flight = N - nMis;
end

% =========================================================================
function E = empty_E()
E = struct('t', zeros(0,1), 'idx', zeros(0,1), 'num', zeros(0,1), 'kind', {cell(0,1)}, ...
           'item', {cell(0,1)}, 'what', {cell(0,1)}, 'n', zeros(0,1), 'e', zeros(0,1), ...
           'alt', zeros(0,1), 'V', zeros(0,1), 'thr', zeros(0,1), 'I', zeros(0,1), 'SOC', zeros(0,1), ...
           'n_mission', 0, 'n_flight', 0, 'note', '', 'wp', '', 'tr_t', zeros(0,1), 'tr_alt', zeros(0,1));
end

function M = read_items(file)
%  Mission items after HOME: seq, cmd, p1..p4, north, east (m from HOME), alt.
M = [];
if isempty(file) || ~ischar(file) || ~isfile(file), return, end
L = strsplit(fileread(file), {sprintf('\r\n'), sprintf('\n')});
H = [];  A = zeros(0, 9);
for k = 2:numel(L)
    v = sscanf(L{k}, '%f');
    if numel(v) < 11, continue, end
    if v(1) == 0, H = v(9:10)'; continue, end
    A(end+1, :) = [v(1) v(4) v(5:8)' v(9) v(10) v(11)]; %#ok<AGROW>
end
if isempty(H) || isempty(A), return, end
Re = 6378137;
hasLL = any(A(:,7:8) ~= 0, 2);
nn = deg2rad(A(:,7) - H(1)) * Re;
ee = deg2rad(A(:,8) - H(2)) * Re * cosd(H(1));
nn(~hasLL) = NaN;  ee(~hasLL) = NaN;
M = struct('seq', A(:,1), 'cmd', A(:,2), 'p', A(:,3:6), 'n', nn, 'e', ee, 'alt', A(:,9));
end

function [rows, nReach, nPos] = time_items(M, t, pos, ha, V, thr, iUp, iLift, iTD, o)
%  Walk the mission in order. Each item is searched for after the previous one.
rows = {};  nReach = 0;  nPos = 0;
NAV = [16 17 18 19 21 22 31];
nav = find(ismember(M.cmd, NAV));
i0 = 1;  if ~isempty(iUp), i0 = iUp; end
cur = i0;                                   % sample where the search starts
lastRow = 0;                                % row index to fold DO items into
pendDo = {};                                % DO items seen before any row
prevNE = pos(i0, 1:2);
for m = 1:numel(M.seq)
    c = M.cmd(m);  k = M.seq(m);  item = sprintf('#%d', k);
    if ~ismember(c, NAV)
        txt = do_text(M, m);
        if isempty(txt), continue, end
        if lastRow > 0
            rows{lastRow}{4} = [rows{lastRow}{4} '; ' txt];
        else
            pendDo{end+1} = txt; %#ok<AGROW>
        end
        continue
    end
    nxt = next_nav(M, nav, m);
    switch c
        case 22                                             % TAKEOFF
            j = find(ha(cur:end) >= M.alt(m) - 1.5, 1);
            if isempty(j), continue, end
            i = cur + j - 1;
            what = sprintf('Takeoff complete at %g m', M.alt(m));
            prevNE = pos(i, 1:2);
        case {16, 17, 18, 19, 31}
            if isnan(M.n(m)), continue, end
            nPos = nPos + 1;
            d = hypot(pos(cur:end, 1) - M.n(m), pos(cur:end, 2) - M.e(m));
            if c == 16
                j = find(d <= o.wp_radius, 1);
                if isempty(j)
                    [dm, j] = min(d);
                    if dm > o.pass_max, continue, end
                end
            else
                r = abs(M.p(m, 3));  if c == 31, r = abs(M.p(m, 2)); end
                if r == 0, r = 90; end
                j = find(d <= r + 35, 1);
                if isempty(j), continue, end
            end
            i = cur + j - 1;  nReach = nReach + 1;
            what = nav_text(M, m, nxt, prevNE);
            prevNE = [M.n(m) M.e(m)];
            if c == 18 || c == 31                           % loiter: search on after it ends
                iend = loiter_end(M, m, i, t, pos, ha);
                cur = max(i, iend);
            else
                cur = i;
            end
            rows{end+1} = {i, 'mission', item, what}; %#ok<AGROW>
            lastRow = numel(rows);
            if ~isempty(pendDo), rows{lastRow}{4} = [rows{lastRow}{4} '; ' strjoin(pendDo, '; ')]; pendDo = {}; end
            continue
        case 21                                             % LAND
            i = cur;
            if ~isempty(iTD) && iTD < i, i = iTD; end
            what = 'LAND active: final approach, flare, touchdown on the runway';
        otherwise
            continue
    end
    rows{end+1} = {i, 'mission', item, what}; %#ok<AGROW>
    lastRow = numel(rows);
    if ~isempty(pendDo), rows{lastRow}{4} = [rows{lastRow}{4} '; ' strjoin(pendDo, '; ')]; pendDo = {}; end
    cur = i;
end
end

function j = next_nav(M, nav, m)
j = nav(find(nav > m, 1));
end

function txt = do_text(M, m)
switch M.cmd(m)
    case 178, txt = sprintf('speed %g m/s', M.p(m, 2));
    case 189, txt = 'landing sequence start (RTL jumps here)';
    case 177, txt = sprintf('jump to item %g', M.p(m, 1));
    case 183, txt = sprintf('servo %g set to %g', M.p(m, 1), M.p(m, 2));
    otherwise, txt = sprintf('command %d', M.cmd(m));
end
end

function s = nav_text(M, m, nxt, prevNE)
%  What the aircraft does from this point on.
c = M.cmd(m);  a = M.alt(m);
switch c
    case 16,       s = sprintf('Waypoint at %g m', a);
    case 18,       s = sprintf('Joins %g %s circle%s, radius %g m, at %g m', M.p(m,1), lr(M.p(m,3)), ...
                               plural(M.p(m,1)), abs(M.p(m,3)), a);
    case 31,       s = sprintf('Joins %s spiral to %g m, radius %g m', lr(M.p(m,2)), a, abs(M.p(m,2)));
    case 17,       s = sprintf('Loiter %s, radius %g m, at %g m', lr(M.p(m,3)), abs(M.p(m,3)), a);
    case 19,       s = sprintf('Loiter %.2g min at %g m', M.p(m,1) / 60, a);
    otherwise,     s = sprintf('Item %d', M.cmd(m));
end
if isempty(nxt), return, end
if c == 16 && ~isnan(M.n(nxt))
    hin  = atan2d(M.e(m) - prevNE(2), M.n(m) - prevNE(1));
    hout = atan2d(M.e(nxt) - M.e(m), M.n(nxt) - M.n(m));
    dh = mod(hout - hin + 180, 360) - 180;
    if abs(dh) >= 10
        s = [s sprintf(', turns %s %.0f deg', ternary(dh > 0, 'right', 'left'), abs(dh))];
    end
end
if M.cmd(nxt) == 21
    s = [s ', then final approach'];
elseif ~isnan(M.alt(nxt)) && M.cmd(nxt) ~= 22
    dz = M.alt(nxt) - a;
    if dz > 2,      s = [s sprintf(', then climbs to %g m', M.alt(nxt))];
    elseif dz < -2, s = [s sprintf(', then descends to %g m', M.alt(nxt))];
    else,           s = [s ', then level'];
    end
    s = [s sprintf(' (#%d)', M.seq(nxt))];
end
end

function iend = loiter_end(M, m, i, t, pos, ha)
%  End of a loiter: swept angle around the centre (LOITER_TURNS) or altitude (LOITER_TO_ALT).
ang = unwrap(atan2(pos(i:end, 2) - M.e(m), pos(i:end, 1) - M.n(m)));
sw  = abs(ang - ang(1));
if M.cmd(m) == 18
    j = find(sw >= 2*pi*max(M.p(m,1), 0.01), 1);
else
    j = find(abs(ha(i:end) - M.alt(m)) <= 2, 1);
end
if isempty(j), iend = i; else, iend = i + j - 1; end
end

function s = lr(r)
if r < 0, s = 'LEFT'; else, s = 'RIGHT'; end
end
function s = plural(n)
if n == 1, s = ''; else, s = 's'; end
end
function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
function s = short(f)
[~, a, b] = fileparts(f);  s = [a b];
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
