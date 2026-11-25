function P = swift_vtail(P)
%SWIFT_VTAIL  Make the aerodynamics and control limits match the SWIFT inverted-V tail.
%
%   P = swift_vtail(P)
%
%   swift_placeholder calls this straight after its section 4 (aerodynamic
%   derivatives), so the trim, the aero matrix and the travel limits further
%   down that file all pick up the V-tail numbers. swift_vtail_add puts that
%   call in for you.
%
%   WHAT IT DOES
%     1. Holds the tail geometry measured from the CAD (section 1).
%     2. Works out what the tail contributes to each derivative from that
%        geometry (section 2) and overwrites those entries in P.aero.
%     3. Defines the ruddervator mixing the Controls subsystem uses, so
%        elevator and rudder share the travel of two real surfaces.
%
%   THE MODEL STILL FLIES ON delta_e AND delta_r
%     A V-tail is two ruddervators. The model keeps an "equivalent"
%     elevator and rudder, which is exact as long as they are defined as
%         delta_e = (rv_L + rv_R)/2        both surfaces together
%         delta_r = (rv_L - rv_R)/2        the two surfaces opposed
%     and the derivatives are per radian of THOSE deflections. Going the
%     other way,  rv_L = delta_e + delta_r,  rv_R = delta_e - delta_r.
%     Each ruddervator stops at +/- P.tail.drv_max, so full elevator and
%     full rudder together cannot both be had - the "V-tail limit" block
%     in Controls enforces that.
%
%   WHAT AN INVERTED V CHANGES (and a conventional tail does not have)
%     - Pitch power scales with cos(Gamma), yaw power with sin(Gamma).
%       At 25 deg the tail is a strong elevator and a weak rudder.
%     - The force on each panel is at right angles to the panel. With the
%       apex UP, those two force lines cross BELOW the aircraft, so rudder
%       rolls the aircraft INTO the turn (Cldr < 0 here; a fin on top of a
%       fuselage gives Cldr > 0), and the tail takes away some dihedral
%       effect (Clbeta becomes less negative).
%
%   TAGS
%     [CAD]       measured from the STEP files in the SWIFT-UAV repository
%                 (ARTS-Laboratory/SWIFT-UAV, Pelican structural_model)
%     [EST->CAD]  estimate; replace from MainAssembly.iam / mass properties
%     [EST->AERO] estimate; replace via AVL/XFLR5 or flight identification
%     [EST]       engineering estimate, basis in the comment
%
%   Ryan Bilock, ARTS Lab | V-tail | v0.1

%% 1 | Settings and geometry ------------------------------------------------
%  scope  'all'       control AND tail-driven stability derivatives (default)
%         'controls'  only CLde, Cmde, CYdr, Cldr, Cndr
scope = 'all';

t.type    = 'inverted V (apex up), twin boom';
t.Gamma   = deg2rad(25);   % rad  panel angle from horizontal                 [CAD]
                           %      root mitre face of Female_Tail_Female_Connector
                           %      (v0.3.1) and the panel faces of Tail.stp (v0.3.0)
t.s       = 0.4535;        % m    one panel, root to tip ALONG the panel      [CAD] v0.3.1
t.c       = 0.2045;        % m    panel chord, ruddervator included           [CAD] v0.3.1
t.c_rv    = 0.0843;        % m    ruddervator chord                           [CAD] v0.3.1
t.b_rv    = 0.296;         % m    ruddervator length (0.300 m bay)            [CAD] v0.3.1
t.boom_sep = 0.765;        % m    boom centre to boom centre                  [CAD] v0.2.4
                           %      MainAssembly.stp. DataSheet.xlsx says 0.62 - out of date.
t.l       = 0.98;          % m    wing 1/4 chord to tail 1/4 chord            [EST->CAD]
                           %      v0.2.4 MainAssembly.stp, CG assumed at the wing 1/4 chord
t.h_apex  = 0.24;          % m    apex above the CG: 0.19 above the booms     [EST->CAD]
                           %      (CAD) + 0.05 booms above the CG (estimate)
t.drv_max = deg2rad(25);   % rad  ruddervator travel, +/- from centre         [EST] measure to retire

t.eta     = 1.00;          % -    dynamic-pressure ratio at the tail. Wing wake
                           %      lowers it, the pusher prop wash raises it.  [EST->AERO]
t.a0      = 5.9;           % 1/rad  NACA 0012 section lift slope, Re ~ 3e5    [EST->AERO]
t.k_visc  = 0.80;          % -    real / thin-airfoil flap effectiveness      [EST->AERO]

%  What the rest of the aircraft (wing + fuselage) adds to the derivatives
%  the tail dominates. Small allowances, not computed.                 [EST->AERO]
wb.Cmq    = -1.0;          % wing pitch damping
wb.CYbeta = -0.08;         % fuselage side force
wb.Cnbeta = -0.015;        % fuselage is destabilising in yaw
wb.Cnr    = -0.015;        % wing yaw damping

%% 2 | Tail contributions from the geometry --------------------------------
%  V-tail theory (Purser & Campbell, NACA Report 823): treat the two panels
%  as one wing of true area S_t, with normal-force slope a_N. A panel force
%  acts at right angles to the panel, so only cos(Gamma) of it is vertical
%  and sin(Gamma) of it is sideways.
S = P.geom.S;   b = P.geom.b;   c = P.geom.c;
cG = cos(t.Gamma);   sG = sin(t.Gamma);

t.S    = 2 * t.s * t.c;                       % m^2  both panels, true area
t.AR   = (2*t.s)^2 / t.S;                     % -    true-span aspect ratio
x      = t.a0 / (pi * t.AR);
t.a_N  = t.a0 / (sqrt(1 + x^2) + x);          % 1/rad  finite-wing slope (Helmbold)

%  Flap effectiveness: thin-airfoil value for the chord ratio, knocked down
%  for viscosity and for the ruddervator covering only part of the panel.
th      = acos(2*t.c_rv/t.c - 1);
t.tau_th = 1 - (th - sin(th)) / pi;           % -    thin-airfoil, full span
t.tau   = t.tau_th * t.k_visc * (t.b_rv / t.s);

t.V_H  = t.S * t.l / (S * c);                 % -    pitch tail volume
t.V_V  = t.S * t.l / (S * b);                 % -    yaw tail volume

%  Height at which the tail's SIDE force acts, measured up from the CG.
%  The panel forces act at mid-panel, at right angles to the panel; their
%  lines cross (s/2)/sin(Gamma) below the apex. Negative = below the CG.
t.z_side = t.h_apex - (t.s/2) / sG;           % m

d.CLde   =  t.eta * (t.S/S) * t.a_N * t.tau * cG;
d.Cmde   = -t.eta * t.V_H   * t.a_N * t.tau * cG;
d.CYdr   =  t.eta * (t.S/S) * t.a_N * t.tau * sG;
d.Cndr   = -t.eta * t.V_V   * t.a_N * t.tau * sG;
d.Cldr   =  d.CYdr * t.z_side / b;

d.Cmq_t    = -2 * t.eta * t.V_H * t.a_N * cG^2 * (t.l / c);
d.CYbeta_t = -t.eta * (t.S/S) * t.a_N * sG^2;
d.Cnbeta_t =  t.eta * t.V_V   * t.a_N * sG^2;
d.Cnr_t    = -2 * t.eta * t.V_V * t.a_N * sG^2 * (t.l / b);
d.Clbeta_t =  d.CYbeta_t * t.z_side / b;

%% 3 | Overwrite the derivatives --------------------------------------------
%  P.tail.base keeps the values that were there before, so calling this
%  twice on the same P gives the same answer.
names = {'CLde', 'Cmde', 'CYdr', 'Cldr', 'Cndr', 'Cmq', 'CYbeta', 'Cnbeta', 'Cnr', 'Clbeta'};
if isfield(P, 'tail') && isfield(P.tail, 'base')
    base = P.tail.base;
else
    base = struct();
    for k = 1:numel(names), base.(names{k}) = P.aero.(names{k}); end
end

new = base;
new.CLde = d.CLde;   new.Cmde = d.Cmde;
new.CYdr = d.CYdr;   new.Cldr = d.Cldr;   new.Cndr = d.Cndr;
if strcmpi(scope, 'all')
    new.Cmq    = d.Cmq_t    + wb.Cmq;
    new.CYbeta = d.CYbeta_t + wb.CYbeta;
    new.Cnbeta = d.Cnbeta_t + wb.Cnbeta;
    new.Cnr    = d.Cnr_t    + wb.Cnr;
    new.Clbeta = base.Clbeta + d.Clbeta_t;    % wing value + what the tail takes away
end
for k = 1:numel(names), P.aero.(names{k}) = new.(names{k}); end

%% 4 | Ruddervator mixing ---------------------------------------------------
%  [rv_L; rv_R] = mix * [delta_e; delta_r],   [delta_e; delta_r] = demix * [rv_L; rv_R]
t.mix   = [1 1; 1 -1];
t.demix = 0.5 * [1 1; 1 -1];
%  ArduPlane's own V-tail mixer gain (MIXING_GAIN). 1.0 = full elevator
%  alone uses full ruddervator travel, as the old elevator channel did.
%  ArduPlane's default is 0.5 (never saturates, half the throw per axis).
t.mix_gain = 1.0;                                                         % [OPS]

t.scope = scope;   t.base = base;   t.wb = wb;   t.derived = d;
P.tail = t;

%  The aero matrix is rebuilt by swift_placeholder section 10. If P already
%  has one (this was called by hand on a finished P), rebuild it now.
if isfield(P.aero, 'A'), P = swift_aero_matrix(P); end

%% 5 | Report ---------------------------------------------------------------
fprintf('\n*** SWIFT V-tail v0.1: %s, %.0f deg, %.3f m^2, arm %.2f m ***\n', ...
        t.type, rad2deg(t.Gamma), t.S, t.l);
fprintf('  a_N %.2f /rad (AR %.2f) | flap effectiveness %.2f | V_H %.2f | V_V %.3f\n', ...
        t.a_N, t.AR, t.tau, t.V_H, t.V_V);
fprintf('  %-8s %10s %10s\n', '', 'before', 'V-tail');
for k = 1:numel(names)
    if abs(new.(names{k}) - base.(names{k})) > 1e-12
        fprintf('  %-8s %10.4f %10.4f\n', names{k}, base.(names{k}), new.(names{k}));
    end
end
fprintf('  Ruddervators +/- %.0f deg each: rv_L = de + dr, rv_R = de - dr\n', rad2deg(t.drv_max));
end
