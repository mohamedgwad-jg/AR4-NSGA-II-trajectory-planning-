function J = evaluate_objectives_constrained(dts, joint_angles_deg, kin_table)
    % Inputs:
    %   dts             — 1×7 delta times for 8 waypoints
    %   joint_angles_deg — 8×6 matrix of joint angles
    %   kin_table       — 6×5 matrix [min max vel acc jerk] per joint
    % Output:
    %   J = [S1, S2, S3] (objectives: time, accel energy, jerk energy)

    t_wp = [0; cumsum(dts(:))];
    T_total = t_wp(end);
    t_eval = linspace(0, T_total, 300);
    N = numel(t_eval);
    dt = mean(diff(t_eval));
    penalty = 0;

    total_accel_energy = 0;
    total_jerk_energy  = 0;

    for j = 1:6
        q = joint_angles_deg(:,j);
        kin = kin_table(j,:);

        [theta, dtheta, ddtheta, ~, ~, ~] = rebuild_spline_dt(q, t_wp);

        % Compute numerical jerk
        dddtheta = gradient(ddtheta, dt);

        % Objective contributions
        total_accel_energy = total_accel_energy + trapz(t_eval, ddtheta.^2);
        total_jerk_energy  = total_jerk_energy  + trapz(t_eval, dddtheta.^2);

        % === Check kinematic constraints ===
        if any(theta < kin(1)) || any(theta > kin(2))
            penalty = penalty + 1e5;  % joint limit
        end
        if any(abs(dtheta) > kin(3))
            penalty = penalty + 1e5;  % velocity
        end
        if any(abs(ddtheta) > kin(4))
            penalty = penalty + 1e5;  % acceleration
        end
        if any(abs(dddtheta) > kin(5))
            penalty = penalty + 1e5;  % jerk
        end
    end

    % === Final objectives ===
    S1 = T_total;
    S2 = total_accel_energy / 6;
    S3 = total_jerk_energy / 6;

    % Add penalty to all (to push them out of Pareto set)
    J = [S1 + penalty, S2 + penalty, S3 + penalty];
end
