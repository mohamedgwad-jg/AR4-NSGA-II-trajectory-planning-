clc; clear; close all;

%% === Load Robot Model ===
robot = importrobot('AR4_robotv5.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
eeName = robot.BodyNames{end};

%% === Load Trajectory Data ===
for j = 1:6
    best(j)  = load(sprintf('Joint%d_Trajectory_Smooth.mat', j));
    worst(j) = load(sprintf('Joint%d_Trajectory_Worst.mat', j));
end

t_best  = best(1).TrajectoryData.time;
t_worst = worst(1).TrajectoryData.time;

N_best  = numel(t_best);
N_worst = numel(t_worst);

theta_best  = zeros(N_best,6);
theta_worst = zeros(N_worst,6);
for j = 1:6
    theta_best(:,j)  = best(j).TrajectoryData.theta;
    theta_worst(:,j) = worst(j).TrajectoryData.theta;
end

%% === Compute EE Cartesian Positions ===
xyz_best  = zeros(N_best,3);
xyz_worst = zeros(N_worst,3);
for i = 1:N_best
    T = getTransform(robot, deg2rad(theta_best(i,:)), eeName);
    xyz_best(i,:) = T(1:3,4)';
end
for i = 1:N_worst
    T = getTransform(robot, deg2rad(theta_worst(i,:)), eeName);
    xyz_worst(i,:) = T(1:3,4)';
end

%% === Define Joint Waypoints ===
joint_waypoints = [
     0     0     0     0     0     0;
    30   -10    15    30    20    25;
    60   -25    35    60    40    50;
    90   -35    20    90    70    75;
   120   -20     0   120    90   100;
   150     0   -20   150    60   125;
   170    30   -40   165    30   150;
   140    60   -60   130     0   120
];

nWP = size(joint_waypoints, 1);

% Times at waypoints
t_wp_best  = [0; cumsum(best(1).TrajectoryData.dt_vec(:))];
t_wp_worst = [0; cumsum(worst(1).TrajectoryData.dt_vec(:))];

% EE positions at waypoints
xyz_wp_best  = zeros(nWP, 3);
xyz_wp_worst = zeros(nWP, 3);
for i = 1:nWP
    T1 = getTransform(robot, deg2rad(joint_waypoints(i,:)), eeName);
    xyz_wp_best(i,:)  = T1(1:3,4)';
    xyz_wp_worst(i,:) = T1(1:3,4)';
end

%% === Plot All (X, Y, Z) in Same Figure ===
labels = {'X (m)', 'Y (m)', 'Z (m)'};
colors = {'r', 'g', 'b'};

figure('Name', 'End-Effector XYZ vs Time', 'Color','w');

for k = 1:3
    subplot(3,1,k);
    hold on;

    % Smoothest solution
    plot(t_best, xyz_best(:,k), '-', 'Color', [0 0.6 0], 'LineWidth', 2);
    plot(t_wp_best, xyz_wp_best(:,k), 'o', 'MarkerSize', 6, ...
        'MarkerEdgeColor','k', 'MarkerFaceColor','y');

    % Worst solution
    plot(t_worst, xyz_worst(:,k), '--', 'Color', [0.8 0 0], 'LineWidth', 2);
    plot(t_wp_worst, xyz_wp_worst(:,k), 's', 'MarkerSize', 6, ...
        'MarkerEdgeColor','k', 'MarkerFaceColor','c');

    ylabel(labels{k});
    title(['EE ' labels{k} ' vs Time']);
    grid on;

    if k == 1
        legend('Smooth', 'Smooth WP', 'Rough', 'Rough WP', ...
            'Location', 'best');
    end
end

xlabel('Time (s)');
