clc; clear; close all;

%% === Step 1: Define True Pareto Front (ZDT3-like synthetic in 3D) ===
nPoints = 100;
f1 = linspace(0, 1, nPoints)';
f2 = 1 - sqrt(f1);
f3 = 1 - f1.^2;
truePF = [f1 f2 f3];

%% === Step 2: Simulate Approximated Front (MOPSO / NSGA-II Result) ===
noise = 0.05 * randn(nPoints, 3);  % add some noise
approxPF = truePF + noise;
approxPF = max(approxPF, 0);  % keep values ≥ 0

%% === Step 3: Compute GD (Generational Distance) ===
distances = zeros(nPoints, 1);
for i = 1:nPoints
    d = vecnorm(truePF - approxPF(i,:), 2, 2);  % Euclidean to all truePF
    distances(i) = min(d);  % distance to closest true point
end
GD = sqrt(mean(distances.^2));

%% === Step 4: Compute SP (Spacing) ===
nearest_dists = zeros(nPoints, 1);
for i = 1:nPoints
    d = vecnorm(approxPF - approxPF(i,:), 2, 2);
    d(i) = inf;  % exclude self
    nearest_dists(i) = min(d);
end
mean_d = mean(nearest_dists);
SP = sqrt(mean((nearest_dists - mean_d).^2));

%% === Step 5: Compute HV (Hypervolume) ===
refPoint = [1.5 1.5 1.5];  % must be dominated by all points
valid_idx = all(bsxfun(@le, approxPF, refPoint), 2);  % only dominated
dominated = approxPF(valid_idx, :);
HV = 0;

% Use a basic hypervolume approximation (grid-based)
nGrid = 100;
vol = 0;
[X,Y,Z] = ndgrid(linspace(0,refPoint(1),nGrid), ...
                 linspace(0,refPoint(2),nGrid), ...
                 linspace(0,refPoint(3),nGrid));
allGrid = [X(:) Y(:) Z(:)];

is_dominated = false(size(allGrid,1),1);
for i = 1:size(dominated,1)
    is_dominated = is_dominated | all(bsxfun(@le, dominated(i,:), allGrid),2);
end
cellVol = prod(refPoint) / numel(is_dominated);
HV = sum(is_dominated) * cellVol;

%% === Step 6: Report ===
fprintf('🧮 GD  = %.4f\n', GD);
fprintf('📏 SP  = %.4f\n', SP);
fprintf('📦 HV  = %.4f\n', HV);

%% === Step 7: Plot Fronts and Metrics ===
figure('Name','3D Pareto Front Comparison','Color','w');
plot3(truePF(:,1), truePF(:,2), truePF(:,3), 'g-', 'LineWidth', 2); hold on;
scatter3(approxPF(:,1), approxPF(:,2), approxPF(:,3), 40, 'r', 'filled');
legend('True Pareto Front','Approximated Front');
xlabel('f1'); ylabel('f2'); zlabel('f3');
title('True vs Approximated Pareto Fronts');
grid on; view(135,30);

figure('Name','Metrics Summary','Color','w');
bar([GD, SP, HV]);
set(gca, 'XTickLabel', {'GD', 'SP', 'HV'});
ylabel('Value'); title('Multi-Objective Evaluation Metrics');
grid on;
% === Simulated Metrics for 3-objective Pareto Evaluation over 50 Generations ===
iterations = 1:50;
GD_vals = exp(-0.1 * iterations) + 0.01 * rand(1, 50);
SP_vals = 0.1 + 0.05 * exp(-0.05 * iterations) + 0.005 * rand(1, 50);
HV_vals = 1 - exp(-0.05 * iterations) + 0.01 * rand(1, 50);

% === Plot ===
figure('Color','w');
plot(iterations, GD_vals, 'r-o', 'LineWidth', 1.5); hold on;
plot(iterations, SP_vals, 'g-s', 'LineWidth', 1.5);
plot(iterations, HV_vals, 'b-^', 'LineWidth', 1.5);
xlabel('Iterations'); ylabel('Metric Value');
title('GD, SP, and HV vs Iterations');
legend('Generational Distance (GD)', 'Spread (SP)', 'Hypervolume (HV)', 'Location','best');
grid on;
