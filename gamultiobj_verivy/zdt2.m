function f = zdt2(x)
    x = x(:);
    f1 = x(1);
    g = 1 + 9 * sum(x(2:end)) / (length(x)-1);
    f2 = g * (1 - (f1 / g)^2);
    f = [f1, f2];
end
