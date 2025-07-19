function [state,options,optchanged] = logMetrics(options,state,flag)
    global logbook
    optchanged = false;

    if strcmp(flag, 'iter')
        gen = state.Generation + 1;  % 0-indexed
        logbook(gen).fvals = state.Score;
    end
end
