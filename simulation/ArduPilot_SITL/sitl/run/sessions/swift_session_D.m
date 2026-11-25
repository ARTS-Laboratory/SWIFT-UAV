function C = swift_session_D
%SWIFT_SESSION_D  Settings for SITL Session D - manoeuvre set and motor check from KCUB.
%   Fly it with:  swift_run D        (see swift_session_template.m for every field)

C.name      = 'D';
C.title     = 'KCUB staircase, figure-8, spirals, slalom + motor check';
C.tag       = 'sessionD_cub';
C.planner   = @swift_sitl_cub_d;               % writes swift_sitl_cub_d.waypoints/.param
C.plan_args = {};
C.params    = {'swift_sitl.param', 'swift_sitl_cub_d.param', ...
               'swift_sitl_batt.param', 'swift_sitl_vtail.param'};
C.max_min   = 45;
C.analysis  = {
    'flight plots', @(out, S, tag) swift_plot_flight(out, 'tag', tag, 'wp', S.wp_file)
    'motor check',  @(out, S, tag) swift_motor_check(out, 'tag', tag)
    };
end
