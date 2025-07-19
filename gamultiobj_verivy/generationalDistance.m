function gd = generationalDistance(P, trueFront)
    D = pdist2(P, trueFront);
    minD = min(D, [], 2);
    gd = sqrt(mean(minD.^2));
end
