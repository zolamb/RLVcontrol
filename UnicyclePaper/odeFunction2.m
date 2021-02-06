function dx = odeFunction2(x)
    dx(1,1) = -3*x(1)*cos(x(2))^2;
    dx(2,1) = -6*x(2) - 3*1*cos(x(2))*sin(x(2))/x(2);
    dx(3,1) = 3*cos(x(2))*sin(x(2));
end