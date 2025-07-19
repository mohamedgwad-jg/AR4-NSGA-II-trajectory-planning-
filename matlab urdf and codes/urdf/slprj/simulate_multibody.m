clc; clear;

% === Load Smooth Trajectories for All 6 Joints ===
for j = 1:6
    data = load(sprintf('Joint%d_Trajectory_Smooth.mat', j));
    t = data.TrajectoryData.time;
    theta = data.TrajectoryData.theta;

    % Create timeseries for each joint
    joint_ts{j} = timeseries(theta, t);
end

% === Export to Workspace Variables ===
q1 = joint_ts{1}; q2 = joint_ts{2}; q3 = joint_ts{3};
q4 = joint_ts{4}; q5 = joint_ts{5}; q6 = joint_ts{6};

% === Optionally save to .mat file for Simulink to load ===
save('Smooth_JointInputs.mat', 'q1','q2','q3','q4','q5','q6');
disp("✅ Smooth trajectory timeseries saved.");
set_param('AR4_robotv5', 'StopTime', num2str(q1.Time(end)));
