function figs = swift_events_figure(E, tag, vis)
%SWIFT_EVENTS_FIGURE  Altitude profile with every numbered point marked, plus a key.
%
%   figs = swift_events_figure(E)              E from swift_flight_events
%   figs = swift_events_figure(E, tag, vis)    title tag, 'on' or 'off'
%
%   Page 1: altitude above the start point against time, with the numbered
%   markers placed on the trace (leader lines where labels had to move to stay
%   clear of each other), then a two-column key: number, time from the start of
%   the log (min:s), what is happening, and airspeed and state of charge at that
%   moment. If the key is too long for one page it carries on to extra pages
%   ("SWIFT SITL - flight events (2 of 2)"). The caller saves the figures.
%
%   The trace comes from E.tr_t / E.tr_alt (swift_flight_events v0.2). With an
%   older E it falls back to joining the event altitudes.
%
%   Ryan Bilock, ARTS Lab | SITL plotting | v0.3
%     v0.3  key times in minutes (2 decimals), same units as the time axis
%     v0.2  annotated altitude profile

if nargin < 2 || isempty(tag), tag = ''; end
if nargin < 3 || isempty(vis), vis = 'on'; end
figs = [];
if isempty(E) || ~isfield(E, 't') || isempty(E.t), return, end

N = numel(E.t);
W  = 1600;                         % figure width, px
rh = 26;                           % key row height, px
colW = (W - 2*40) / 2;             % key column width
plotH = 470;                       % altitude panel height, px
head  = 112;                       % title + subtitle + legend
rows1 = 20;  rowsN = 30;           % key rows per column: page 1 / later pages
k0 = [0 0 0];  grey = [0.35 0.35 0.35];

% ---- split the key into pages ------------------------------------------------
pg = {};  i = 1;  first = true;
while i <= N
    per = 2 * (first * rows1 + ~first * rowsN);
    pg{end+1} = i : min(i + per - 1, N); %#ok<AGROW>
    i = i + per;  first = false;
end
np = numel(pg);

for p = 1:np
    rr  = pg{p};
    nr  = ceil(numel(rr) / 2);                         % rows per column on this page
    top = head + (p == 1) * (plotH + 112);              % y of the key header
    H   = top + 34 + nr * rh + 64;
    nm = 'SWIFT SITL - flight events';
    if np > 1, nm = sprintf('%s (%d of %d)', nm, p, np); end
    f = figure('Name', nm, 'NumberTitle', 'off', 'Color', 'w', 'Visible', vis, ...
               'Position', [40 30 W H], 'InvertHardcopy', 'off');
    try, theme(f, 'light'); catch, end
    % overlay for all page text, pixel coordinates, y down
    ov = axes('Parent', f, 'Units', 'normalized', 'Position', [0 0 1 1], 'XLim', [0 W], ...
              'YLim', [0 H], 'YDir', 'reverse', 'Visible', 'off');
    hold(ov, 'on');

    % ---- title block
    ttl = 'Flight events on the altitude profile';
    if ~isempty(tag), ttl = [ttl '  |  ' tag]; end
    if p > 1, ttl = [ttl sprintf('  (key, page %d of %d)', p, np)]; end
    text(ov, 40, 40, ttl, 'FontSize', 18, 'FontWeight', 'bold', 'Color', k0, 'Interpreter', 'none');
    sub = sprintf(['%d points: %d mission items, %d flight and battery events. ' ...
                   'The same numbers mark every graph in this run folder.'], N, E.n_mission, E.n_flight);
    text(ov, 40, 70, sub, 'FontSize', 11.5, 'Color', grey, 'Interpreter', 'none');
    lx = 40;
    for kk = {'mission', 'Mission item'; 'flight', 'Flight event'; 'battery', 'Battery event'}'
        badge(ov, lx + 9, 96, '', kcol(kk{1}), 11);
        text(ov, lx + 24, 96, kk{2}, 'FontSize', 11, 'Color', k0, 'VerticalAlignment', 'middle');
        lx = lx + 165;
    end

    % ---- altitude profile (page 1 only)
    if p == 1
        ax = axes('Parent', f, 'Units', 'pixels', 'Position', [100 H-head-plotH-24 W-150 plotH]);
        profile(ax, E);
    end

    % ---- key
    rule(ov, 40, W-40, top - 6, [0.15 0.20 0.28], 1.5);
    for c = 1:2
        x0 = 40 + (c-1) * colW;
        text(ov, x0 + 14,  top + 12, '#',    'FontSize', 10.5, 'FontWeight', 'bold', 'Color', grey, 'HorizontalAlignment', 'center');
        text(ov, x0 + 36,  top + 12, 'min', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', grey);
        text(ov, x0 + 100, top + 12, 'What is happening', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', grey);
        text(ov, x0 + colW - 84, top + 12, 'm/s', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', grey, 'HorizontalAlignment', 'right');
        text(ov, x0 + colW - 16, top + 12, 'SOC %', 'FontSize', 10.5, 'FontWeight', 'bold', 'Color', grey, 'HorizontalAlignment', 'right');
    end
    rule(ov, 40, W-40, top + 26, [0.75 0.75 0.75], 0.75);
    for j = 1:numel(rr)
        k = rr(j);  c = 1 + (j > nr);  r = j - (c-1)*nr;
        x0 = 40 + (c-1) * colW;  y = top + 34 + (r - 0.5) * rh;
        if mod(r, 2) == 0
            patch(ov, x0 + [0 colW-8 colW-8 0], y + rh/2*[-1 -1 1 1], [0.955 0.962 0.972], 'EdgeColor', 'none');
        end
        badge(ov, x0 + 14, y, sprintf('%d', E.num(k)), kcol(E.kind{k}), 19);
        text(ov, x0 + 36, y, sprintf('%.2f', E.t(k) / 60), 'FontSize', 10.5, 'Color', k0, 'VerticalAlignment', 'middle');
        fw = 'normal';  if ~strcmp(E.kind{k}, 'mission'), fw = 'bold'; end
        text(ov, x0 + 100, y, fit(keytext(E, k), 72), 'FontSize', 10.5, 'Color', k0, 'FontWeight', fw, ...
             'VerticalAlignment', 'middle', 'Interpreter', 'none');
        text(ov, x0 + colW - 84, y, num(E.V(k), '%.1f'), 'FontSize', 10.5, 'Color', k0, ...
             'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
        text(ov, x0 + colW - 16, y, num(E.SOC(k), '%.1f'), 'FontSize', 10.5, 'Color', k0, ...
             'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    end
    yb = top + 34 + nr * rh;
    rule(ov, 40, W-40, yb + 2, [0.75 0.75 0.75], 0.75);
    foot = ['Time is in minutes from the start of the log. Airspeed and state of charge (SOC) are at the moment of each point; ' ...
            'the full values are in events.csv.'];
    if ~isempty(E.note), foot = [foot '  Note: ' E.note]; end
    text(ov, 40, yb + 30, fit(foot, 230), 'FontSize', 9.5, 'Color', grey, 'Interpreter', 'none');
    figs = [figs f]; %#ok<AGROW>
end
end

% =========================================================================
function profile(ax, E)
%  Altitude trace with numbered markers. Labels sit above the trace where they
%  fit; when two would overlap the later one steps further out on a leader line.
hold(ax, 'on');  box(ax, 'off');
if isfield(E, 'tr_t') && ~isempty(E.tr_t)
    tm = E.tr_t(:) / 60;  hm = E.tr_alt(:);
else
    tm = E.t(:) / 60;     hm = E.alt(:);
end
hm(hm < 0 & hm > -1) = 0;
hmax = max([hm; E.alt(:); 10]);
xl = [min(tm) max(tm)];  xl = xl + [-0.01 0.01] * diff(xl);
yl = [-0.06 1.36] * hmax;
set(ax, 'XLim', xl, 'YLim', yl, 'FontSize', 11, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
        'TickDir', 'out', 'Layer', 'top', 'Color', 'w');
grid(ax, 'on');  set(ax, 'GridColor', [0.80 0.80 0.80], 'GridAlpha', 0.6);
fill(ax, [tm; flipud(tm)], [hm; zeros(size(hm))], [0.11 0.62 0.46], 'FaceAlpha', 0.10, 'EdgeColor', 'none');
plot(ax, tm, hm, '-', 'Color', [0.06 0.43 0.34], 'LineWidth', 1.8);
plot(ax, xl, [0 0], '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
xlabel(ax, 'Time from start of log (min)', 'FontSize', 12, 'Color', [0 0 0]);
ylabel(ax, 'Altitude above start (m)', 'FontSize', 12, 'Color', [0 0 0]);

% ---- label placement (greedy, in normalised axes units)
xs = diff(xl);  ys = diff(yl);
tx = E.t(:) / 60;  ty = max(E.alt(:), 0);
bw = 0.0135;  bh = 0.058;                          % label box, fraction of axes
boxes = zeros(0, 4);                               % [x0 y0 x1 y1] normalised
cand = [1.0 2.0 3.0 4.0 5.0 -1.2 -2.2];            % multiples of bh, + = above
for k = 1:numel(tx)
    px = (tx(k) - xl(1)) / xs;  py = (ty(k) - yl(1)) / ys;
    s = sprintf('%d', E.num(k));  w = bw * (0.75 + 0.45 * numel(s));
    best = [];
    for m = cand
        cy = py + m * bh;
        if cy - bh/2 < 0.075 || cy + bh/2 > 0.985, continue, end
        for dx = [0 -1 1] * 0.9 * w                 % try straight, then nudge sideways
            cx = min(max(px + dx, w/2 + 0.003), 1 - w/2 - 0.003);
            b = [cx - w/2, cy - bh/2, cx + w/2, cy + bh/2];
            if ~any(b(1) < boxes(:,3) & b(3) > boxes(:,1) & b(2) < boxes(:,4) & b(4) > boxes(:,2))
                best = [cx cy];  break
            end
        end
        if ~isempty(best), break, end
    end
    if isempty(best), best = [px, min(py + 1.1*bh, 0.96)]; end
    boxes(end+1, :) = [best(1) - w/2, best(2) - bh/2, best(1) + w/2, best(2) + bh/2]; %#ok<AGROW>
    lx = xl(1) + best(1) * xs;  ly = yl(1) + best(2) * ys;
    c = kcol(E.kind{k});
    line(ax, [tx(k) lx], [ty(k) ly], 'Color', 1 - 0.55*(1 - c), 'LineWidth', 0.9);
    line(ax, tx(k), ty(k), 'Marker', 'o', 'MarkerSize', 5, 'MarkerFaceColor', c, ...
         'MarkerEdgeColor', 'w', 'LineStyle', 'none');
    badge(ax, lx, ly, s, c, 19);
end
end

function badge(ax, x, y, s, c, sz)
%  Filled circle with a white number in it.
line(ax, x, y, 'Marker', 'o', 'MarkerSize', sz, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', ...
     'LineWidth', 1, 'LineStyle', 'none');
if ~isempty(s)
    text(ax, x, y, s, 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', 'w', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end
end

function rule(ax, x0, x1, y, c, lw)
line(ax, [x0 x1], [y y], 'Color', c, 'LineWidth', lw);
end

function s = keytext(E, k)
%  Short key text. Mission rows are "Name - generated detail"; keep the name and
%  the useful part of the detail, drop what the graph already shows.
s = E.what{k};
if ~strcmp(E.kind{k}, 'mission'), return, end
parts = regexp(s, ' - ', 'split', 'once');
if numel(parts) == 2, nm = parts{1};  d = parts{2}; else, nm = '';  d = s; end
d = regexprep(d, '\s*\(#\d+\)', '');
d = regexprep(d, '^Waypoint at [\d.]+ m,?\s*', '');
d = regexprep(d, '^Takeoff complete at [\d.]+ m;?\s*', '');
d = regexprep(d, '(\d+) deg', '$1°');
d = regexprep(d, '^then level;?\s*', '');
d = regexprep(d, '^Joins (LEFT|RIGHT) spiral to [\d.]+ m, ', '');
d = regexprep(d, '^Joins ', 'joins ');
d = regexprep(d, ', at [\d.]+ m', '');
d = regexprep(d, ', then level$', '');
d = regexprep(d, 'LAND active:.*', 'LAND active');
d = regexprep(d, ' \(RTL jumps here\)', '');
d = regexprep(d, '^,\s*', '');
d = strtrim(d);
if isempty(nm), s = d; elseif isempty(d), s = nm; else, s = [nm ' · ' d]; end
end

function s = mmss(t)
t = round(t * 10) / 10;                     % so 59.96 s prints as 1:00.0, not 0:60.0
s = sprintf('%d:%04.1f', floor(t/60), t - 60*floor(t/60));
end

function s = num(v, f)
if isnan(v), s = '-'; else, s = sprintf(f, v); if strcmp(s(1), '-') && str2double(s) == 0, s = s(2:end); end, end
end

function s = fit(s, n)
if numel(s) > n, s = [s(1:n-3) '...']; end
end

function c = kcol(kind)
switch kind
    case 'mission', c = [0.20 0.33 0.55];
    case 'battery', c = [0.80 0.15 0.15];
    otherwise,      c = [0.90 0.45 0.10];
end
end
