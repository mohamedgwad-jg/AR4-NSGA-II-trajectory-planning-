%% extract_xyz_from_mat.m
clc; clear;

% 1. Load Joint Trajectories
nJoints = 6;
for j = 1:nJoints
    data(j) = load(sprintf('Joint%d_Trajectory_TaskOpt.mat', j));
    % expects data(j).TrajectoryData.theta (Nx1 deg) and .time (Nx1)
end

% 2. Build theta matrix and time vector
t       = data(1).TrajectoryData.time;  % Nx1
thetaMat = zeros(numel(t), nJoints);

for j = 1:nJoints
    thetaMat(:,j) = data(j).TrajectoryData.theta;
end

% 3. Import the URDF robot model
robot = importrobot('AR4_robotv5.urdf');
robot.DataFormat = 'row';

% 4. Compute end‑effector positions
eeName = robot.BodyNames{end};
N      = size(thetaMat,1);
xyz    = zeros(N,3);

for i = 1:N
    cfg = deg2rad(thetaMat(i,:));           % convert to radians
    Tfm = getTransform(robot, cfg, eeName);
    xyz(i,:) = Tfm(1:3,4)';                  % extract x,y,z
end

% 5. Save the result
save('EE_Path.mat', 'xyz', 't');

fprintf('Extracted end‑effector path (Nx3) and time vector saved to EE_Path.mat\n');
