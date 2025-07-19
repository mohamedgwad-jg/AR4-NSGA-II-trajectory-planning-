function [theta, dtheta, ddtheta, spline, Q, knots] = rebuild_spline_dt(joint_angles, t_waypoints)
    n = length(joint_angles);
    order = 6;  % Quintic B-spline
    num_ctrl_pts = n + 4;

    knots = augknt(t_waypoints, order);

    % Constraint matrix
    A = zeros(n + 4, num_ctrl_pts);
    rhs = zeros(n + 4, 1);

    for i = 1:n
        A(i,:) = spcol(knots, order, t_waypoints(i));
        rhs(i) = joint_angles(i);
    end

    % Velocity & acceleration constraints at start/end
    d1 = fnder(spmak(knots, eye(num_ctrl_pts)), 1);
    d2 = fnder(spmak(knots, eye(num_ctrl_pts)), 2);

    A(n+1,:) = fnval(d1, t_waypoints(1));      rhs(n+1) = 0;
    A(n+2,:) = fnval(d1, t_waypoints(end));    rhs(n+2) = 0;
    A(n+3,:) = fnval(d2, t_waypoints(1));      rhs(n+3) = 0;
    A(n+4,:) = fnval(d2, t_waypoints(end));    rhs(n+4) = 0;

    % Solve
    Q = A \ rhs;

    % Build spline
    spline = spmak(knots, Q');
    t_eval = linspace(t_waypoints(1), t_waypoints(end), 300);
    theta    = fnval(spline, t_eval)';
    dtheta   = fnval(fnder(spline, 1), t_eval)';
    ddtheta  = fnval(fnder(spline, 2), t_eval)';
end
