function C = swift_session_E
%SWIFT_SESSION_E  Settings for SITL Session E - out-and-back range test from KCUB.
%   Fly it with:  swift_run E
%   Planner options pass through, e.g.  swift_run('E', 'plan', {'land_mAh', 4000})
%   Skip the slow video:                swift_run('E', 'skip', {'video'})

C.name      = 'E';
C.title     = 'KCUB out-and-back range test at best-range speed';
C.tag       = 'sessionE_cub';
C.planner   = @swift_sitl_cub_e;               % writes swift_sitl_cub_e.waypoints/.param
C.plan_args = {};
C.params    = {'swift_sitl.param', 'swift_sitl_cub_e.param', ...
               'swift_sitl_batt.param', 'swift_sitl_vtail.param'};
C.max_min   = 40;
C.analysis  = {
    'flight plots', @(out, S, tag) swift_plot_flight(out, 'tag', tag, 'wp', S.wp_file)
    'range check',  @(out, S, tag) swift_range_check(out, 'tag', tag)
    'vtail check',  @(out, S, tag) swift_vtail_check(out, 'tag', tag)
    'video',        @(out, S, tag) swift_flight_video(out, 'tag', tag, 'speed', 20)
    };
end
