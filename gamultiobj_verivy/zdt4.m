function f = zdt4(x)
    x = x(:);
    f1 = x(1);
    g = 1 + 10*(length(x)-1) + sum(x(2:end).^2 - 10 * cos(4 * pi * x(2:end)));
    f2 = g * (1 - sqrt(f1 / g));
    f = [f1, f2];
end
