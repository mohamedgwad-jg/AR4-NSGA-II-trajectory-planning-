clc; clear;

%% === ZDT Problem Setup ===
zdt_funcs = {@zdt1, @zdt2, @zdt3, @zdt4, @zdt6};  % Skip ZDT5
zdt_names = {'ZDT1','ZDT2','ZDT3','ZDT4','ZDT6'};
true_refs = {[2 2], [2 2], [2 2], [4 600], [2  2]};
nvars = 500;
maxGen = 500;

for idx = 1:length(zdt_funcs)
    fprintf('\n=== Solving %s ===\n', zdt_names{idx});

    objfun = zdt_funcs{idx};
    ref    = true_refs{idx};
    title_label = zdt_names{idx};

    % Bounds
    if strcmp(zdt_names{idx}, 'ZDT4')
        lb = -5 * ones(1,nvars); lb(1) = 0;
        ub =  5 * ones(1,nvars); ub(1) = 1;
    else
        lb = zeros(1, nvars);
        ub = ones(1, nvars);
    end

    % Setup output tracking
    global logbook
    logbook = struct('fvals', cell(maxGen,1));
    opts = optimoptions('gamultiobj', ...
        'Display','none', ...
        'PopulationSize',100, ...
        'MaxGenerations',maxGen, ...
        'OutputFcn',@logMetrics);

    % Run NSGA-II
    [x_opt, fval] = gamultiobj(objfun, nvars, [], [], [], [], lb, ub, opts);

    % === Load true front (only for ZDT1,2,3,6) ===
    f1_true = linspace(0, 1, 100);
    switch title_label
        case 'ZDT1'
            f2_true = 1 - sqrt(f1_true);
        case 'ZDT2'
            f2_true = 1 - (f1_true).^2;
        case 'ZDT3'
            f2_true = 1 - sqrt(f1_true) - f1_true .* sin(10*pi*f1_true);
        case 'ZDT6'
            f2_true = 1 - f1_true.^2;
        otherwise
            f2_true = [];
    end

    %% === Metrics Computation ===
    GD = zeros(maxGen,1); SP = GD; HV = GD;
    for gen = 1:maxGen
        P = logbook(gen).fvals;
        if isempty(P), continue; end
        if ~isempty(f2_true)
            true_front = [f1_true' f2_true'];
            GD(gen) = generationalDistance(P, true_front);
        else
            GD(gen) = NaN;
        end
        SP(gen) = spacing(P);
        HV(gen) = hypervolume(P, ref);
    end

    %% === Plot Pareto Front ===
    figure('Name',['Pareto Front - ' title_label]);
    plot(fval(:,1), fval(:,2), 'ro'); hold on;
   if ~isempty(f2_true)
    plot(f1_true, f2_true, 'b-', 'LineWidth', 2);
    legend('Computed', 'True Front', 'Location', 'best');
else
    legend('Computed', 'Location', 'best');
end

    xlabel('f1'); ylabel('f2');
    title(['Pareto Front - ' title_label]);
    legend('Computed','True Front','Location','best');
    grid on;

    %% === Plot Metrics ===
    figure('Name',['Metrics - ' title_label]);
    subplot(3,1,1); plot(GD, 'b', 'LineWidth', 2); title('GD (Convergence)'); grid on;
    subplot(3,1,2); plot(SP, 'g', 'LineWidth', 2); title('SP (Spacing)'); grid on;
    subplot(3,1,3); plot(HV, 'm', 'LineWidth', 2); title('HV (Hypervolume)'); grid on;
end
