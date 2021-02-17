function dx = odeFunction2(x, gamma, h, k)
    dx(1,1) = -gamma*x(1)*cos(x(2))^2;
    dx(2,1) = -k*x(2) - gamma*h*cos(x(2))*sin(x(2))/x(2);
    dx(3,1) = gamma*cos(x(2))*sin(x(2));
end