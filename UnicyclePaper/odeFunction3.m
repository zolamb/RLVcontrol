function dx = fun(x, u, w)
    dx(1,1) = u*cos(x(3));
    dx(2,1) = u*sin(x(3));
    dx(3,1) = w;
end