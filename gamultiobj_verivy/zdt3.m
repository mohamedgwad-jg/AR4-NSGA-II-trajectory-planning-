function f = zdt3(x)
    x = x(:);
    f1 = x(1);
    g = 1 + 9 * sum(x(2:end)) / (length(x)-1);
    f2 = g * (1 - sqrt(f1 / g) - (f1 / g) * sin(10 * pi * f1));
    f = [f1, f2];
end
