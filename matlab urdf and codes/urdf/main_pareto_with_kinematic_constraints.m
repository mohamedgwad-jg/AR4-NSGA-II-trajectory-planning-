clc; clear; close all;

%% === Waypoints (8x6) ===
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
Ndts = size(joint_angles_deg,1) - 1;  % number of segments

%% === Kinematic Constraints Table (joint limits, velocities, accelerations, jerks) ===
% Format: [min_angle max_angle max_vel max_accel max_jerk]
kinematic_table = [
    -170  170  360  3000  8000;
    -42    90  360  3000  8000;
    -89    52  360  3000  8000;
    -165 165  360  3000  8000;
    -105 105  360  3000  8000;
    -155 155  360  3000  8000
];

%% === Bounds for delta t
lb = 0.5 * ones(1, Ndts);
ub = 3.0 * ones(1, Ndts);

%% === Objective function
objfun = @(dts) evaluate_objectives_constrained(dts, joint_angles_deg, kinematic_table);

%% === NSGA-II settings
opts = optimoptions('gamultiobj', ...
    'Display', 'iter', ...
    'PopulationSize', 120, ...
    'MaxGenerations', 50, ...
    'UseParallel', false);

%% === Run optimization
[dts_opt, fval] = gamultiobj(objfun, Ndts, [], [], [], [], lb, ub, opts);

%% === Extract both Smoothest (min S2) and Worst (max S2) Solutions

[~, best_idx] = min(fval(:,2));
[~, worst_idx] = max(fval(:,2));

dts_best = dts_opt(best_idx,:);
dts_worst = dts_opt(worst_idx,:);

t_waypoints_best  = [0; cumsum(dts_best(:))];
t_waypoints_worst = [0; cumsum(dts_worst(:))];

T_total_best  = t_waypoints_best(end);
T_total_worst = t_waypoints_worst(end);

%% === Rebuild and Save Smoothest Trajectories
for j = 1:6
    qj = joint_angles_deg(:,j);
    kin = kinematic_table(j,:);

    [theta, dtheta, ddtheta, ~, Q, ~] = rebuild_spline_dt(qj, t_waypoints_best);

    TrajectoryData.time     = linspace(0, T_total_best, 300)';
    TrajectoryData.theta    = theta(:);
    TrajectoryData.dtheta   = dtheta(:);
    TrajectoryData.ddtheta  = ddtheta(:);
    TrajectoryData.dt_vec   = dts_best(:);
    TrajectoryData.ctrl_pts = Q;
    TrajectoryData.KinLimits= kin;

    save(sprintf('Joint%d_Trajectory_Smooth.mat', j), 'TrajectoryData');
end

%% === Rebuild and Save Worst Trajectories
for j = 1:6
    qj = joint_angles_deg(:,j);
    kin = kinematic_table(j,:);

    [theta, dtheta, ddtheta, ~, Q, ~] = rebuild_spline_dt(qj, t_waypoints_worst);

    TrajectoryData.time     = linspace(0, T_total_worst, 300)';
    TrajectoryData.theta    = theta(:);
    TrajectoryData.dtheta   = dtheta(:);
    TrajectoryData.ddtheta  = ddtheta(:);
    TrajectoryData.dt_vec   = dts_worst(:);
    TrajectoryData.ctrl_pts = Q;
    TrajectoryData.KinLimits= kin;

    save(sprintf('Joint%d_Trajectory_Worst.mat', j), 'TrajectoryData');
end

%% === Plot Pareto Front and Mark Solutions
figure('Name','Pareto Front (All Solutions)','Color','w');
scatter3(fval(:,1), fval(:,2), fval(:,3), 40, 'b', 'filled'); hold on;
xlabel('S1: Total Time (s)');
ylabel('S2: Accel Energy');
zlabel('S3: Jerk Energy');
title('Pareto Front of Multi-Objective Optimization');
grid on; view(135,30);

% Mark Smoothest
S_best = fval(best_idx,:);
scatter3(S_best(1), S_best(2), S_best(3), 100, 'g', 'filled', 'Marker','*');
text(S_best(1), S_best(2), S_best(3), sprintf(' Smoothest\n #%d', best_idx), 'Color','g');

% Mark Worst
S_worst = fval(worst_idx,:);
scatter3(S_worst(1), S_worst(2), S_worst(3), 100, 'r', 'filled', 'Marker','p');
text(S_worst(1), S_worst(2), S_worst(3), sprintf(' Worst\n #%d', worst_idx), 'Color','r');

legend('Pareto Solutions', 'Smoothest', 'Worst', 'Location','best');

%% === Save Solutions as .mat for reference
save('Smoothest_Solution.mat', 'dts_best', 'S_best', 'best_idx');
save('Worst_Solution.mat', 'dts_worst', 'S_worst', 'worst_idx');

disp("✅ Both smoothest and worst trajectories saved.");
%% === Export All Objective Values to Table and Save ===
% Round objective values to 4 digits for clarity
S1 = round(fval(:,1), 4);   % Total Time
S2 = round(fval(:,2), 4);   % Accel Energy
S3 = round(fval(:,3), 4);   % Jerk Energy

% Create a table
ParetoTable = table((1:length(S1))', S1, S2, S3, ...
    'VariableNames', {'Index','TotalTime_S1','AccelEnergy_S2','JerkEnergy_S3'});

% Add logical flags for best and worst
ParetoTable.Smoothest = (1:length(S1))' == best_idx;
ParetoTable.Worst     = (1:length(S1))' == worst_idx;

% Save as .mat
save('Pareto_Objective_Table.mat', 'ParetoTable');

% Save as .csv (formatted for Excel)
writetable(ParetoTable, 'Pareto_Objective_Table.csv', 'Delimiter', ',');

disp('✅ Readable Pareto table exported to .mat and .csv');
