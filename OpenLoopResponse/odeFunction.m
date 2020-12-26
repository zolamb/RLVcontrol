function dx = odeFunction(x, w, L, bL, m, Fw, I, u)
    % Control inputs
    F1=u(1);
    F2=u(2);
    Ft=u(3);
    Psi=u(4);
    
    % System of 1st Order ODE's
    dx(1,1) = x(4,1);
    dx(2,1) = x(5,1);
    dx(3,1) = x(6,1);
    dx(4,1) = -(F1+F2)/m*sin(x(3)) - (Ft/m)*sin(x(3) + Psi);
    dx(5,1) =  (F1+F2)/m*cos(x(3)) + (Ft/m)*cos(x(3) + Psi) - Fw/m;
    dx(6,1) =  (w/(2*I))*(F1-F2) - (Ft/I)*(L/2 - bL)*sin(Psi);
end