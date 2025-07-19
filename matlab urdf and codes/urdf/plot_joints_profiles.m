clc; clear; close all;

%% === Load Both Trajectories ===
for j = 1:6
    best(j)  = load(sprintf('Joint%d_Trajectory_Smooth.mat', j));
    worst(j) = load(sprintf('Joint%d_Trajectory_Worst.mat', j));
end

%% === Joint Waypoints (8×6) ===
joint_angles_deg = [
     0     0     0     0     0     0;
    30   -10    15    30    20    25;
    60   -25    35    60    40    50;
    90   -35    20    90    70    75;
   120   -20     0   120    90   100;
   150     0   -20   150    60   125;
   170    30   -40   165    30   150;
   140    60   -60   130     0   120
];

%% === Plot θ, θ̇, θ̈, θ⁗ for Each Joint Individually ===
for j = 1:6
    % === Load profiles and time vectors
    t_b  = best(j).TrajectoryData.time;
    t_w  = worst(j).TrajectoryData.time;
    N_b = numel(t_b);
    N_w = numel(t_w);

    q_b    = best(j).TrajectoryData.theta;
    dq_b   = best(j).TrajectoryData.dtheta;
    ddq_b  = best(j).TrajectoryData.ddtheta;
    dddq_b = gradient(dq_b, t_b);  % θ⁗ for smooth

    q_w    = worst(j).TrajectoryData.theta;
    dq_w   = worst(j).TrajectoryData.dtheta;
    ddq_w  = worst(j).TrajectoryData.ddtheta;
    dddq_w = gradient(dq_w, t_w);  % θ⁗ for worst

    % Waypoint time vectors from Δt (accumulated)
    wp_t_b = [0; cumsum(best(j).TrajectoryData.dt_vec(:))];
    wp_t_w = [0; cumsum(worst(j).TrajectoryData.dt_vec(:))];

    %% === Plot
    figure('Name', sprintf('Joint %d Profiles (Individual Time)', j), 'Color','w');

    % θ(t)
    subplot(4,1,1);
    plot(t_b, q_b, 'g-', 'LineWidth', 2); hold on;
    plot(t_w, q_w, 'r--', 'LineWidth', 2);
    plot(wp_t_b, joint_angles_deg(:,j), 'ko', 'MarkerFaceColor','y');
    plot(wp_t_w, joint_angles_deg(:,j), 'ks', 'MarkerFaceColor','c');
    ylabel('\theta (deg)'); title(sprintf('Joint %d — Position', j));
    legend('Smooth','Rough','Waypoints (Smooth)', 'Waypoints (Worst)'); grid on;

    % θ̇(t)
    subplot(4,1,2);
    plot(t_b, dq_b, 'g-', 'LineWidth', 2); hold on;
    plot(t_w, dq_w, 'r--', 'LineWidth', 2);
    plot(wp_t_b, interp1(t_b, dq_b, wp_t_b), 'ko', 'MarkerFaceColor','y');
    plot(wp_t_w, interp1(t_w, dq_w, wp_t_w), 'ks', 'MarkerFaceColor','c');
    ylabel('\thetȧ (deg/s)'); title('Velocity'); grid on;

    % θ̈(t)
    subplot(4,1,3);
    plot(t_b, ddq_b, 'g-', 'LineWidth', 2); hold on;
    plot(t_w, ddq_w, 'r--', 'LineWidth', 2);
    plot(wp_t_b, interp1(t_b, ddq_b, wp_t_b), 'ko', 'MarkerFaceColor','y');
    plot(wp_t_w, interp1(t_w, ddq_w, wp_t_w), 'ks', 'MarkerFaceColor','c');
    ylabel('\thetä (deg/s²)'); title('Acceleration'); grid on;

    % θ⁗(t)
    subplot(4,1,4);
    plot(t_b, dddq_b, 'g-', 'LineWidth', 2); hold on;
    plot(t_w, dddq_w, 'r--', 'LineWidth', 2);
    plot(wp_t_b, interp1(t_b, dddq_b, wp_t_b), 'ko', 'MarkerFaceColor','y');
    plot(wp_t_w, interp1(t_w, dddq_w, wp_t_w), 'ks', 'MarkerFaceColor','c');
    ylabel('\theta⁗ (deg/s³)'); title('Jerk'); xlabel('Time (s)');
    grid on;
end
