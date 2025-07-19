function hv = hypervolume(P, ref)
    P = P(all(P <= ref, 2), :);  % Only points within reference box
    if isempty(P)
        hv = 0;
        return;
    end
    [~, idx] = sortrows(P, 1);
    P = P(idx, :);
    hv = 0;
    for i = 1:size(P,1)
        if i == 1
            dx = ref(1) - P(i,1);
        else
            dx = P(i-1,1) - P(i,1);
        end
        dy = ref(2) - P(i,2);
        hv = hv + dx * dy;
    end
end
