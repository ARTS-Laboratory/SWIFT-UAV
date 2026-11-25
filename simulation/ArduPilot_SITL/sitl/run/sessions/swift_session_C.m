function C = swift_session_C
%SWIFT_SESSION_C  Settings for SITL Session C - first ArduPlane flight from KCUB.
%   Fly it with:  swift_run C        (see swift_session_template.m for every field)

C.name      = 'C';
C.title     = 'KCUB runway 13 - first ArduPlane circuit';
C.tag       = 'sessionC_cub';                  % data\runs\<tag>\
C.planner   = @swift_sitl_cub;                 % writes swift_sitl_cub.waypoints/.param
C.plan_args = {};
C.params    = {'swift_sitl.param', 'swift_sitl_cub.param', ...
               'swift_sitl_batt.param', 'swift_sitl_vtail.param'};   % load order
C.max_min   = 30;                              % warn if airborne longer than this
C.analysis  = {
    'flight plots', @(out, S, tag) swift_plot_flight(out, 'tag', tag, 'wp', S.wp_file)
    };
end
