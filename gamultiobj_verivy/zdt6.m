function f = zdt6(x)
    x = x(:);
    f1 = 1 - exp(-4 * x(1)) * sin(6 * pi * x(1))^6;
    g = 1 + 9 * (sum(x(2:end)) / (length(x)-1))^0.25;
    f2 = g * (1 - (f1 / g)^2);
    f = [f1, f2];
end
