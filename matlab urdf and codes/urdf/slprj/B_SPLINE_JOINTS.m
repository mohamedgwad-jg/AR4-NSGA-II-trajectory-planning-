clc; clear; close all;
%% === 1. Define Waypoints for All 6 Joints ===
joint_angles = [
0     0     0     0     0     0;
30   -10    15    30    20    25;
60   -25    35    60    40    50;
90   -35    20    90    70    75;
120   -20     0   120    90   100;
150     0   -20   150    60   125;
170    30   -40   165    30   150;
140    60   -60   130     0   120
];  % 8 × 6
num_joints = 6;
num_waypoints = size(joint_angles,1);
t_waypoints = linspace(0, 19.809436182955068, num_waypoints);  % 1×8
t_eval = linspace(0, 19.809436182955068, 300);  % 1×300
%% === 2. B-Spline Parameters ===
order = 6;  % Quintic (degree 5)
num_ctrl_pts = num_waypoints + 4;  % 12
knots = augknt(linspace(0, 19.809436182955068, num_ctrl_pts - order + 2), order);
greville = aveknt(knots, order);  % 1×12
%% === 3. Preallocate Trajectories ===
theta_all   = zeros(300, num_joints);
theta_d_all = zeros(300, num_joints);
theta_dd_all= zeros(300, num_joints);
Q_all = zeros(num_ctrl_pts, num_joints);
%% === 4. Loop Over Each Joint and Generate B-Spline ===
for j = 1:num_joints
joint_j_angles = joint_angles(:,j);
% === Build Full Constraint Matrix (positions + velocity/accel constraints)
A = zeros(12, num_ctrl_pts);
rhs = zeros(12,1);
for i = 1:num_waypoints
A(i,:) = spcol(knots, order, t_waypoints(i));
rhs(i) = joint_j_angles(i);
end
% Add velocity constraints at t=0, t=7
d1 = fnder(spmak(knots, eye(num_ctrl_pts)), 1);
A(9,:)  = fnval(d1, 0);
A(10,:) = fnval(d1, 2.29);
% Add acceleration constraints at t=0, t=7
d2 = fnder(spmak(knots, eye(num_ctrl_pts)), 2);
A(11,:) = fnval(d2, 0);
A(12,:) = fnval(d2, 2.29);
% RHS remains zero (0 vel & accel)
rhs(9:12) = 0;
% === Solve for control points
Q = A \ rhs;
Q_all(:,j) = Q;
% === Construct spline and its derivatives
spline_pos = spmak(knots, Q');
spline_vel = fnder(spline_pos, 1);
spline_acc = fnder(spline_pos, 2);
% === Evaluate
theta_all(:,j)    = fnval(spline_pos, t_eval)';
theta_d_all(:,j)  = fnval(spline_vel, t_eval)';
theta_dd_all(:,j) = fnval(spline_acc, t_eval)';
end
%% === 5. Plot All θ(t), θ̇(t), θ̈(t) with Waypoints ===
joint_labels = {'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'};
for j = 1:num_joints
figure('Name', ['Joint ' num2str(j) ' B-Spline']);
subplot(3,1,1);
plot(t_eval, theta_all(:,j), 'b', 'LineWidth', 2); hold on;
plot(t_waypoints, joint_angles(:,j), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(greville, Q_all(:,j), 'r*--', 'MarkerSize', 6, 'LineWidth', 1.2);
title(['\theta(t) for ' joint_labels{j}]);
ylabel('\theta (deg)');
legend('\theta(t)', 'Waypoints', 'Control Points');
grid on;
subplot(3,1,2);
plot(t_eval, theta_d_all(:,j), 'g', 'LineWidth', 2);
title(['\thetȧ(t) for ' joint_labels{j}]);
ylabel('\thetȧ (deg/s)');
grid on;
subplot(3,1,3);
plot(t_eval, theta_dd_all(:,j), 'm', 'LineWidth', 2);
title(['\thetä(t) for ' joint_labels{j}]);
ylabel('\thetä (deg/s²)');
xlabel('Time (s)');
grid on;
end
%% === 6. Compute and Plot End-Effector 3D Path ===

% --- Load Robot Model ---
robot = importrobot('AR4_robotv5.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
eeName = robot.BodyNames{end};

N = size(theta_all, 1);
xyz = zeros(N, 3);  % EE path

% --- Compute EE position using FK at each interpolated time ---
for i = 1:N
    config_rad = deg2rad(theta_all(i,:));  % convert to radians
    T = getTransform(robot, config_rad, eeName);
    xyz(i,:) = T(1:3,4)' * 1000;  % convert to millimeters
end

% --- Compute EE positions at the original waypoints ---
num_wp = size(joint_angles,1);
xyz_wp = zeros(num_wp, 3);
for i = 1:num_wp
    config_wp_rad = deg2rad(joint_angles(i,:));
    T_wp = getTransform(robot, config_wp_rad, eeName);
    xyz_wp(i,:) = T_wp(1:3,4)' * 1000;
end

% --- Plot EE path in 3D ---
figure('Name','End-Effector Path in 3D');
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(xyz_wp(:,1), xyz_wp(:,2), xyz_wp(:,3), 'ro--', ...
    'MarkerFaceColor','y','LineWidth',1.5,'MarkerSize',7);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('End-Effector Cartesian Path from B-spline Interpolation');
legend('Interpolated Path','Waypoints','Location','best');
grid on; axis equal; view(3);
%% === 7. Plot EE X, Y, Z Positions on One Graph vs Time ===
figure('Name','End-Effector Position vs Time (X, Y, Z)');
plot(t_eval, xyz(:,1), 'r-', 'LineWidth', 2); hold on;
plot(t_eval, xyz(:,2), 'g-', 'LineWidth', 2);
plot(t_eval, xyz(:,3), 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (mm)');
title('End-Effector Cartesian Position vs Time');
legend('X (mm)', 'Y (mm)', 'Z (mm)', 'Location', 'Best');
grid on;
%% === 8. Save B-Spline Interpolation Data for Comparison ===

for j = 1:num_joints
    data.time     = t_eval(:);
    data.theta    = theta_all(:,j);
    data.dtheta   = theta_d_all(:,j);
    data.ddtheta  = theta_dd_all(:,j);
    data.ctrl_pts = Q_all(:,j);
    data.joint_id = j;
    data.type     = 'bspline_no_optimization';
    
    filename = sprintf('Joint%d_BSpline_NoOpt.mat', j);
    save(filename, 'data');
end

% Save task-space EE path as well
EE_data.time = t_eval(:);
EE_data.xyz  = xyz;
EE_data.xyz_wp = xyz_wp;
EE_data.source = 'bspline_no_optimization';

save('EE_BSpline_NoOpt.mat', 'EE_data');

disp('✅ B-spline interpolation data saved for all joints.');
