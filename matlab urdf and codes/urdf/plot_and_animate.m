clc; clear; close all;

%% === Load Both Smoothest and Worst Trajectories ===
for j = 1:6
    best(j)  = load(sprintf('Joint%d_Trajectory_Smooth.mat', j));
    worst(j) = load(sprintf('Joint%d_Trajectory_Worst.mat', j));
end

t_best  = best(1).TrajectoryData.time;
t_worst = worst(1).TrajectoryData.time;
N = min(numel(t_best), numel(t_worst));
dt = min(diff(t_best(1:2)), diff(t_worst(1:2)));

theta_best   = zeros(N,6);
dtheta_best  = zeros(N,6);
ddtheta_best = zeros(N,6);

theta_worst   = zeros(N,6);
dtheta_worst  = zeros(N,6);
ddtheta_worst = zeros(N,6);

for j = 1:6
    theta_best(:,j)   = best(j).TrajectoryData.theta(1:N);
    dtheta_best(:,j)  = best(j).TrajectoryData.dtheta(1:N);
    ddtheta_best(:,j) = best(j).TrajectoryData.ddtheta(1:N);

    theta_worst(:,j)   = worst(j).TrajectoryData.theta(1:N);
    dtheta_worst(:,j)  = worst(j).TrajectoryData.dtheta(1:N);
    ddtheta_worst(:,j) = worst(j).TrajectoryData.ddtheta(1:N);
end

%% === Load URDF Robot ===
robot = importrobot('AR4_robotv5.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
eeName = robot.BodyNames{end};

xyz_best  = zeros(N,3);
xyz_worst = zeros(N,3);
for i = 1:N
T_best = getTransform(robot, deg2rad(theta_best(i,:)), eeName);
xyz_best(i,:) = T_best(1:3,4)';
T_worst = getTransform(robot, deg2rad(theta_worst(i,:)), eeName);
xyz_worst(i,:) = T_worst(1:3,4)';
end

%% === Waypoints in Task Space ===
waypoints_deg = [
     0     0     0     0     0     0;
    30   -10    15    30    20    25;
    60   -25    35    60    40    50;
    90   -35    20    90    70    75;
   120   -20     0   120    90   100;
   150     0   -20   150    60   125;
   170    30   -40   165    30   150;
   140    60   -60   130     0   120
];
xyz_wp = zeros(size(waypoints_deg,1), 3);
for i = 1:size(waypoints_deg,1)
    T_wp = getTransform(robot, deg2rad(waypoints_deg(i,:)), eeName);
    xyz_wp(i,:) = T_wp(1:3,4)';
end
%% === Figure: Combined 3D Plot of Both Solutions and Waypoints ===
fig3 = figure('Name','Comparison of Smooth and Rough Trajectories','Color','w','Position',[600 100 900 600]);
ax3 = axes(fig3); hold(ax3, 'on');
view(ax3, 3); grid(ax3, 'on'); axis(ax3, 'equal');
xlabel(ax3, 'X (m)'); ylabel(ax3, 'Y (m)'); zlabel(ax3, 'Z (m)');
title(ax3, 'End-Effector Paths: Smooth vs Rough Trajectories');

% Plot both paths
plot3(ax3, xyz_best(:,1), xyz_best(:,2), xyz_best(:,3), 'g-', 'LineWidth', 2);
plot3(ax3, xyz_worst(:,1), xyz_worst(:,2), xyz_worst(:,3), 'r-', 'LineWidth', 2);

% Plot waypoints
plot3(ax3, xyz_wp(:,1), xyz_wp(:,2), xyz_wp(:,3), 'ko--', ...
    'MarkerFaceColor', 'y', 'LineWidth', 1.5, 'MarkerSize', 7);

legend(ax3, {'Smooth Path', 'Rough Path', 'Waypoints'}, 'Location', 'best');

%% === Figure: Smoothest ===
fig1 = figure('Name','Smooth Trajectory','Color','w','Position',[100 100 1000 600]);
ax1 = axes(fig1, 'Position',[0.05 0.1 0.65 0.8]);
show(robot, deg2rad(theta_best(1,:)), 'Parent', ax1, 'PreservePlot', false); hold on;
plot3(xyz_best(:,1), xyz_best(:,2), xyz_best(:,3), 'b-', 'LineWidth', 2);
plot3(xyz_wp(:,1), xyz_wp(:,2), xyz_wp(:,3), 'ko--', 'MarkerFaceColor','y');
hEE1 = plot3(xyz_best(1,1), xyz_best(1,2), xyz_best(1,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');
view(3); axis equal; grid on; xlabel('X'); ylabel('Y'); zlabel('Z');

hPanel1 = uipanel(fig1,'Title','Smooth State','FontSize',10,'Position',[0.72 0.1 0.26 0.8]);
hText1 = uicontrol(hPanel1,'Style','text','Units','normalized','Position',[0 0 1 1], ...
    'FontName','Courier','FontSize',10,'BackgroundColor','w','HorizontalAlignment','left');

%% === Figure: Worst ===
fig2 = figure('Name','Rough Trajectory','Color','w','Position',[1150 100 1000 600]);
ax2 = axes(fig2, 'Position',[0.05 0.1 0.65 0.8]);
show(robot, deg2rad(theta_worst(1,:)), 'Parent', ax2, 'PreservePlot', false); hold on;
plot3(xyz_worst(:,1), xyz_worst(:,2), xyz_worst(:,3), 'b-', 'LineWidth', 2);
plot3(xyz_wp(:,1), xyz_wp(:,2), xyz_wp(:,3), 'ko--', 'MarkerFaceColor','y');
hEE2 = plot3(xyz_worst(1,1), xyz_worst(1,2), xyz_worst(1,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');
view(3); axis equal; grid on; xlabel('X'); ylabel('Y'); zlabel('Z');

hPanel2 = uipanel(fig2,'Title','Rough State','FontSize',10,'Position',[0.72 0.1 0.26 0.8]);
hText2 = uicontrol(hPanel2,'Style','text','Units','normalized','Position',[0 0 1 1], ...
    'FontName','Courier','FontSize',10,'BackgroundColor','w','HorizontalAlignment','left');
% === Create Video Writer ===
v = VideoWriter('AR4_Trajectory_Comparison.mp4', 'MPEG-4');
v.FrameRate = round(1/dt);  % Match animation speed
open(v);

%% === Animate Both ===
for i = 1:N
    cfg_best = deg2rad(theta_best(i,:));
    cfg_worst = deg2rad(theta_worst(i,:));

    show(robot, cfg_best, 'Parent', ax1, 'PreservePlot', false, 'Frames','off');
    show(robot, cfg_worst, 'Parent', ax2, 'PreservePlot', false, 'Frames','off');

    set(hEE1, 'XData', xyz_best(i,1), 'YData', xyz_best(i,2), 'ZData', xyz_best(i,3));
    set(hEE2, 'XData', xyz_worst(i,1), 'YData', xyz_worst(i,2), 'ZData', xyz_worst(i,3));

    txt1 = sprintf('t = %.2f s\n\nEE (mm): X=%.1f\n Y=%.1f\n Z=%.1f\n\n', ...
        t_best(i), xyz_best(i,1)*1000, xyz_best(i,2)*1000, xyz_best(i,3)*1000);
    txt2 = sprintf('t = %.2f s\n\nEE (mm): X=%.1f\n Y=%.1f\n Z=%.1f\n\n', ...
        t_worst(i), xyz_worst(i,1)*1000, xyz_worst(i,2)*1000, xyz_worst(i,3)*1000);

    for j = 1:6
        txt1 = [txt1, sprintf('J%d: θ=%.1f°\n     ω=%.1f°/s\n     α=%.1f°/s²\n\n', ...
            j, theta_best(i,j), dtheta_best(i,j), ddtheta_best(i,j))];
        txt2 = [txt2, sprintf('J%d: θ=%.1f°\n     ω=%.1f°/s\n     α=%.1f°/s²\n\n', ...
            j, theta_worst(i,j), dtheta_worst(i,j), ddtheta_worst(i,j))];
    end

    hText1.String = txt1;
    hText2.String = txt2;

    drawnow;
    % Capture combined frame (only one of the figures, or create a combined one)
frame1 = getframe(fig1);
frame2 = getframe(fig2);

% Optional: stitch side-by-side (figures must be same size!)
combined = [frame1.cdata, frame2.cdata];
writeVideo(v, combined);  % Save combined frame to video

    pause(dt);
end
close(v);
disp("🎥 Video saved as AR4_Trajectory_Comparison.mp4");
