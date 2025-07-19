function sp = spacing(P)
    D = pdist2(P, P);
    D(D == 0) = inf;
    minD = min(D, [], 2);
    sp = std(minD);
end
