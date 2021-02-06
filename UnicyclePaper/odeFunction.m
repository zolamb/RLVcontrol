function dx = odeFunction(x)
    dx(1,1) = x(1)*(1 - x(1));
    dx(2,1) = sin(x(2)/2)^2;
end