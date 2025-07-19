clc; clear;

%% === 1. Problem Definition: ZDT1 ===
nvars = 30;
lb = zeros(1, nvars);
ub = ones(1, nvars);
objfun = @(x) zdt1(x);

maxGen = 100;
populationSize = 500;

% === Global variable for metrics logging ===
global logbook;
logbook = struct('fvals', cell(maxGen, 1));

%% === 2. Set Options for gamultiobj ===
opts = optimoptions('gamultiobj', ...
    'PopulationSize', populationSize, ...
    'MaxGenerations', maxGen, ...
    'Display', 'iter', ...
    'OutputFcn', @logMetrics);

%% === 3. Run Optimization (NSGA-II) ===
[x_opt, fval] = gamultiobj(objfun, nvars, [], [], [], [], lb, ub, opts);

%% === 4. True Pareto Front (ZDT1)
f1_true = linspace(0, 1, 100);
f2_true = 1 - sqrt(f1_true);
true_front = [f1_true' f2_true'];

%% === 5. Compute & Plot Metrics ===
GD = zeros(maxGen, 1);
SP = zeros(maxGen, 1);
HV = zeros(maxGen, 1);

for gen = 1:maxGen
    P = logbook(gen).fvals;
    if isempty(P), continue; end

    GD(gen) = generationalDistance(P, true_front);
    SP(gen) = spacing(P);
    HV(gen) = hypervolume(P, [2, 2]); % Reference point
end

%% === 6. Plot Metrics ===
figure;
subplot(3,1,1);
plot(1:maxGen, GD, 'b-', 'LineWidth', 2);
title('GD (Generational Distance)'); xlabel( 'Generation'); ylabel('GD'); grid on;

subplot(3,1,2);
plot(1:maxGen, SP, 'g-', 'LineWidth', 2);
title('SP (Spacing)'); xlabel('Generation'); ylabel('SP'); grid on;

subplot(3,1,3);
plot(1:maxGen, HV, 'm-', 'LineWidth', 2);
title('HV (Hypervolume)'); xlabel('Generation'); ylabel('HV'); grid on;

%% === 7. Final Pareto Plot ===
figure;
plot(fval(:,1), fval(:,2), 'ro', 'LineWidth', 1.5); hold on;
plot(f1_true, f2_true, 'b-', 'LineWidth', 2);
xlabel('f_1'); ylabel('f_2');
legend('Computed Pareto Front', 'True ZDT1 Front', 'Location', 'Best');
title('Final Pareto Front'); grid on;
