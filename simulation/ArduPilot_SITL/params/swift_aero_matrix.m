function P = swift_aero_matrix(P)
%SWIFT_AERO_MATRIX  Pack the aerodynamic derivatives into one matrix gain.
%
%   P = swift_aero_matrix(P)
%
%  WHY THIS FILE EXISTS
%    swift_6dof builds a 9-element regressor vector on the canvas (step 14):
%
%        u = [ 1 ; alpha ; beta ; p*b/(2V) ; q*c/(2V) ; r*b/(2V) ; da ; de ; dr ]
%
%    and ONE Gain block in Matrix(K*u) mode (step 15) turns it into five
%    coefficients:
%
%        [CL; -CY; Cl; Cm; Cn] = P.aero.A * u      (-CY: see SIDE-FORCE ROW)
%
%    Writing the matrix out by hand would be unreadable and easy to get
%    wrong. Writing the derivatives out one per line, as swift_placeholder
%    does, reads like a normal aerodynamic dataset. This function is the
%    bridge: derivatives in, one 5x9 matrix out.
%
%    That single matrix IS the aerodynamic database. Refit it from AVL,
%    XFLR5, wind tunnel data, or flight identification and the model picks
%    up the new aerodynamics with no block edited.
%
%  WHAT IT PRODUCES
%    P.aero.A              5x9, the matrix the Gain block reads
%    P.aero.k_ind          induced-drag factor, read by a Gain in step 16
%    P.geom.AR             aspect ratio          (reference / hand checks)
%    P.mass.W              weight in newtons     (reference / hand checks)
%    P.perf.V_stall        1-g stall speed       (reference / hand checks)
%    P.perf.wing_loading   kg per square metre   (reference / hand checks)
%
%    Only A and k_ind are read by the model. The rest are there so you can
%    sanity-check the parameter set without doing arithmetic by hand.
%
%  WHY IT IS A FUNCTION AND NOT A SCRIPT
%    It takes P in and hands P back, so you can re-run it any time you poke
%    a derivative at the command line:
%
%        P.aero.Cmalpha = -0.80;
%        P = swift_aero_matrix(P);      % matrix rebuilds, model picks it up
%
%    A script would have to be edited to do the same thing. You will use
%    this while tuning.
%
%  DRAG IS DELIBERATELY NOT IN HERE
%    CD = CD0 + k*CL^2 is the one genuinely nonlinear term, so it cannot
%    live in a linear matrix. Three blocks on the canvas handle it (step 16),
%    reading P.aero.CD0 and P.aero.k_ind.
%
%  SIDE-FORCE ROW IS STORED WITH ITS SIGN FLIPPED (v0.3)
%    The Aerodynamic Forces and Moments block, set to Wind input axes, treats
%    all three force coefficients as positive AGAINST their axis:
%        F_wind = qbar*S*[-CD; -(row 2); -CL]
%    Drag and lift are meant to work that way. Side force is not: CYbeta and
%    CYdr below are the standard body-y derivatives (positive to the right).
%    So row 2 of P.aero.A holds -CY. Enter CYbeta, CYp, CYr, CYdr in
%    swift_placeholder with their normal textbook signs - this file does the
%    flip. Found 14 NOV 2026 by rebuilding the Session D forces from the
%    logged signals: they matched only with the side force reversed.
%
%  Ryan Bilock, ARTS Lab | v0.3
%    v0.3  row 2 stored as -CY so the Wind-axes aero block applies side
%          force in the correct direction (was reversed in v0.2 and earlier).
%    v0.2  input validation with a named error; header rewritten for the
%          from-scratch manual.

%% ---- check the inputs are all there --------------------------------------
need = { 'CL0','CLalpha','CLq','CLde', ...
         'CD0','e_osw','CLmax', ...
         'CYbeta','CYp','CYr','CYdr', ...
         'Clbeta','Clp','Clr','Clda','Cldr', ...
         'Cm0','Cmalpha','Cmq','Cmde', ...
         'Cnbeta','Cnp','Cnr','Cnda','Cndr' };

if ~isfield(P,'aero')
    error('swift_aero_matrix:noAero', ...
        'P.aero does not exist. Run swift_placeholder (or swift_params) first.');
end
missing = need(~cellfun(@(f) isfield(P.aero,f), need));
if ~isempty(missing)
    error('swift_aero_matrix:missing', ...
        ['These aerodynamic derivatives are missing from P.aero:\n    %s\n' ...
         'Check the spelling in section 4 of your parameter script - the two\n' ...
         'files have to agree on every name.'], strjoin(missing, ', '));
end
for f = {'b','S','c'}
    if ~isfield(P,'geom') || ~isfield(P.geom, f{1})
        error('swift_aero_matrix:noGeom', ...
            'P.geom.%s is missing. Geometry is needed for the aspect ratio.', f{1});
    end
end

a = P.aero;

%% ---- the matrix -----------------------------------------------------------
%  COLUMN ORDER IS THE CONTRACT. It must match the Mux wiring in step 14
%  exactly. Swap two columns here, or two Mux ports there, and the model
%  runs happily with the wrong physics - no error, no warning.
%
%        col 1    col 2      col 3     col 4     col 5     col 6     col 7     col 8     col 9
%        1        alpha      beta      p_hat     q_hat     r_hat     da        de        dr
P.aero.A = [ ...
       a.CL0,   a.CLalpha,  0,        0,        a.CLq,    0,        0,        a.CLde,   0;       % CL
       0,       0,          a.CYbeta, a.CYp,    0,        a.CYr,    0,        0,        a.CYdr;  % CY
       0,       0,          a.Clbeta, a.Clp,    0,        a.Clr,    a.Clda,   0,        a.Cldr;  % Cl
       a.Cm0,   a.Cmalpha,  0,        0,        a.Cmq,    0,        0,        a.Cmde,   0;       % Cm
       0,       0,          a.Cnbeta, a.Cnp,    0,        a.Cnr,    a.Cnda,   0,        a.Cndr]; % Cn

%  Row 2 goes to the aero block as -CY (see SIDE-FORCE ROW in the header).
P.aero.A(2,:) = -P.aero.A(2,:);

%  Every zero above is a modelling statement you can point at in a review.
%  Row CL, column beta is zero because sideslip does not change lift to
%  first order. Row Cl, column alpha is zero because incidence does not roll
%  a symmetric aircraft. Any of them can become a real derivative later
%  without restructuring anything.

%% ---- derived scalars ------------------------------------------------------
P.geom.AR           = P.geom.b^2 / P.geom.S;
P.aero.k_ind        = 1 / (pi * P.aero.e_osw * P.geom.AR);
P.mass.W            = P.mass.m * P.const.g;
P.perf.V_stall      = sqrt(2*P.mass.W / (P.const.rho0 * P.geom.S * P.aero.CLmax));
P.perf.wing_loading = P.mass.m / P.geom.S;

%% ---- shape check ----------------------------------------------------------
if ~isequal(size(P.aero.A), [5 9])
    error('swift_aero_matrix:badShape', ...
        'P.aero.A came out %dx%d, not 5x9. A derivative is probably a vector.', ...
        size(P.aero.A,1), size(P.aero.A,2));
end
end
