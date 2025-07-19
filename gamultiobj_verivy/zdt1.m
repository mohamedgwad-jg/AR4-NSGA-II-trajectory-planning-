function f = zdt1(x)
    x = x(:);
    f1 = x(1);
    g = 1 + 9 * sum(x(2:end)) / (length(x)-1);
    f2 = g * (1 - sqrt(f1 / g));
    f = [f1, f2];
end
