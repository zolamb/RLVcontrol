function dx = odeFunction(t, x, u, w) % first parameter must be to be t
    dx(1,1) = u*cos(x(3));
    dx(2,1) = u*sin(x(3));
    dx(3,1) = w;
end