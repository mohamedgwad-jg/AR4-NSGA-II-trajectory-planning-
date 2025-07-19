clc; clear;

% Initialize storage
S1_time     = zeros(6,1);
S2_accel    = zeros(6,1);
S3_jerk     = zeros(6,1);
BestIndex   = zeros(6,1);

fprintf('📊 Extracting Best Objective Values (Smoothest) from Pareto:\n');
fprintf('------------------------------------------------------------\n');

for j = 1:6
    file = sprintf('Joint%d_Trajectory_Smooth.mat', j);
    
    if isfile(file)
        data = load(file);
        
        if isfield(data, 'Pareto Table') && all(isfield(data.Pareto, {'S1_time', 'S2_accel', 'S3_jerk'}))
            % Get best solution (minimum acceleration energy = smoothest)
            [~, best_idx] = min(data.Pareto.S2_accel);
            BestIndex(j) = best_idx;
            S1_time(j)  = data.Pareto.S1_time(best_idx);
            S2_accel(j) = data.Pareto.S2_accel(best_idx);
            S3_jerk(j)  = data.Pareto.S3_jerk(best_idx);
            
            fprintf('Joint %d: S1 = %.3f s,\tS2 = %.3e,\tS3 = %.3e\t(Index = %d)\n', ...
                j, S1_time(j), S2_accel(j), S3_jerk(j), best_idx);
        else
            fprintf('Joint %d: ⚠️ Missing Pareto fields in file: %s\n', j, file);
        end
    else
        fprintf('Joint %d: ❌ File not found: %s\n', j, file);
    end
end

% Optionally: Save to a table
ObjTable = table((1:6)', S1_time, S2_accel, S3_jerk, BestIndex, ...
    'VariableNames', {'Joint', 'S1_TotalTime_sec', 'S2_AccelEnergy', 'S3_JerkEnergy', 'BestIndex'});
writetable(ObjTable, 'Objective_Values_Smoothest.csv');  % Optional
