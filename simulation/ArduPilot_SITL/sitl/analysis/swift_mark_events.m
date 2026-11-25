function swift_mark_events(ax, E, varargin)
%SWIFT_MARK_EVENTS  Draw the numbered event markers from swift_flight_events on one axes.
%
%   swift_mark_events(ax, E)                     time axis in seconds, badges on
%   swift_mark_events(ax, E, 'scale', 1/60)      time axis in minutes
%   swift_mark_events(ax, E, 'labels', false)    lines only (lower tiles)
%   swift_mark_events(ax, E, 'x', xv, 'win', w)  x axis is something else: xv(k) is
%                                                the x value of sample k, only events
%                                                with sample index in w are drawn
%   swift_mark_events(ax, E, 'mode', 'track')    ground track (x = east, y = north)
%
%   Time plots: a dotted line at each event, and (with labels on) a filled
%   number badge for every event in a band added above the data. Badges are
%   swept left to right into as few rows as needed, sliding right on a short
%   leader when events are close in time, so they read in order and no two
%   badges ever overlap. Same badge
%   style and colours as swift_events_figure and the battery-on-track figure:
%   blue-grey = mission item, orange = flight event, red = battery.
%   Call it after everything else is plotted on the axes: it fixes the limits.
%
%   Ryan Bilock, ARTS Lab | SITL plotting | v0.2 (filled badges, no overlaps)

o.mode = 'time';  o.scale = 1;  o.labels = true;  o.x = [];  o.win = [];  o.font = 8;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
if isempty(E) || ~isfield(E, 't') || isempty(E.t) || ~isgraphics(ax), return, end

N = numel(E.t);
keep = true(N, 1);
if ~isempty(o.win), keep = E.idx >= min(o.win) & E.idx <= max(o.win); end
if ~any(keep), return, end
held = ishold(ax);  hold(ax, 'on');

kk = find(keep);
if strcmp(o.mode, 'track')
    % points closer than 30 m share one marker, labelled "31,32"
    for g = groups([E.e(kk) E.n(kk)], 30)
        k = kk(g{1});  c = kcol(E.kind{k(1)});
        line(ax, E.e(k(1)), E.n(k(1)), 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 6.5, ...
             'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'LineWidth', 1, 'HandleVisibility', 'off');
        if o.labels
            text(ax, E.e(k(1)), E.n(k(1)), ['  ' numlist(E.num(k))], 'Color', c, 'FontSize', o.font + 1, ...
                 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', 'Clipping', 'on');
        end
    end
    if ~held, hold(ax, 'off'); end
    return
end

% ---- event lines on a time (or mapped) x axis ----------------------------------
if isempty(o.x), xe = E.t * o.scale; else, xe = o.x(E.idx); xe = xe(:); end
yl0 = get(ax, 'YLim');  set(ax, 'YLim', yl0);       % freeze the data limits
xl  = get(ax, 'XLim');  xspan = diff(xl);
in  = kk(xe(kk) >= xl(1) & xe(kk) <= xl(2));
if isempty(in), if ~held, hold(ax, 'off'); end, return, end

if ~o.labels
    for g = groups(xe(in), 0.004 * xspan)           % same moment -> one line
        k = in(g{1});  c = kcol(E.kind{k(1)});
        line(ax, xe(k(1)) * [1 1], yl0, 'Color', 1 - 0.6*(1 - c), 'LineStyle', ':', 'LineWidth', 0.9, ...
             'Marker', 'none', 'HandleVisibility', 'off');
    end
    if ~held, hold(ax, 'off'); end
    return
end

% ---- badges: pack into rows in pixel space -------------------------------------
try, drawnow; catch, end                           % let tiledlayout settle the axes size
pp = getpixelposition(ax);  Wp = max(pp(3), 50);  Hp = max(pp(4), 40);
d  = 20;                                           % badge diameter (px)
for attempt = 1:3
    [cx, row, nrow] = pack(xe(in), xl, Wp, d);
    band = nrow * (d + 4) + 6;                     % px above the data
    if band <= 0.5 * Hp || d <= 14, break, end
    d = max(14, floor(d * 0.82));                  % crowded small tile: smaller badges
end
f   = min(band / Hp, 0.6);
top = yl0(2) + diff(yl0) * f / (1 - f);
set(ax, 'YLim', [yl0(1) top]);
pxy = (top - yl0(1)) / Hp;                         % data units per pixel (y)
ms  = d * 0.75;  fs = max(6, round(d * 0.37 * 2) / 2);

for g = groups(xe(in), 0.004 * xspan)              % dotted line up to the band
    k = in(g{1});  c = kcol(E.kind{k(1)});
    line(ax, xe(k(1)) * [1 1], [yl0(1) yl0(2)], 'Color', 1 - 0.6*(1 - c), 'LineStyle', ':', ...
         'LineWidth', 0.9, 'Marker', 'none', 'HandleVisibility', 'off');
end
for j = 1:numel(in)
    k  = in(j);  c = kcol(E.kind{k});
    xb = xl(1) + cx(j) / Wp * xspan;
    yb = yl0(2) + (3 + (row(j) - 0.5) * (d + 4)) * pxy;
    line(ax, [xe(k) xb], [yl0(2) yb], 'Color', 1 - 0.6*(1 - c), 'LineWidth', 0.8, ...
         'Marker', 'none', 'HandleVisibility', 'off');
    line(ax, xb, yb, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', ms, 'MarkerFaceColor', c, ...
         'MarkerEdgeColor', 'w', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    text(ax, xb, yb, sprintf('%d', E.num(k)), 'Color', 'w', 'FontSize', fs - (E.num(k) > 99), ...
         'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
         'Clipping', 'off', 'HandleVisibility', 'off');
end
if ~held, hold(ax, 'off'); end
end

% =========================================================================
function [cx, row, nrow] = pack(x, xl, Wp, d)
%  Left-to-right sweep: each badge goes in the lowest row where it fits at or
%  just right of its own line (at most 4 diameters along, on a leader), so
%  the numbers always read in order and no two badges ever overlap.
px  = (x(:) - xl(1)) / diff(xl) * Wp;
[~, ord] = sort(px);
cx = zeros(size(px));  row = zeros(size(px));
last = [];  gap = d + 2;  lo = d/2 + 1;  hi = Wp - d/2 - 1;
for j = ord'
    r = 0;
    while true
        r = r + 1;
        if r > numel(last), last(r) = -inf; end %#ok<AGROW>
        c = max(min(max(px(j), lo), hi), last(r) + gap);
        if (c - px(j) <= 4 * d && c <= hi) || r > 40
            cx(j) = min(c, hi);  row(j) = r;  last(r) = cx(j);  break
        end
    end
end
nrow = max([row; 1]);
end

function G = groups(X, tol)
%  Runs of consecutive points within tol of the first point of the run.
G = {};  n = size(X, 1);  i = 1;
while i <= n
    j = i;
    while j < n && norm(X(j+1, :) - X(i, :)) < tol, j = j + 1; end
    G{end+1} = i:j; %#ok<AGROW>
    i = j + 1;
end
end

function s = numlist(v)
s = sprintf('%d,', v);  s = s(1:end-1);
end

function c = kcol(kind)
switch kind
    case 'mission', c = [0.20 0.33 0.55];
    case 'battery', c = [0.80 0.15 0.15];
    otherwise,      c = [0.90 0.45 0.10];
end
end
