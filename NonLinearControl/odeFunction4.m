function dx = odeFunction4(x, w, L, bL, m, Fw, I, u) % first parameter must be to be t
    % Control inputs
    F1=u(1);
    F2=u(2);
    Ft=u(3);
    Psi=u(4);

    % Newtonian set of ODEs
    ddotX = (F1+F2)/m*cos(x(2)) + (Ft/m)*cos(x(2) + Psi);
    ddotY = (F1+F2)/m*sin(x(2)) + (Ft/m)*sin(x(2) + Psi) - Fw/m;
    ddotTheta = (w/(2*I))*(F1-F2) - (Ft/I)*(L/2 - bL)*sin(Psi);

    % Solve ODEs
    dx(1,1) = x(3);
    dx(2,1) = x(4);
    dx(3,1) = sqrt(ddotX^2 + ddotY^2);
    dx(4,1) = ddotTheta;
    dx(5,1) = x(7);
    dx(6,1) = x(8);
    dx(7,1) = ddotX;
    dx(8,1) = ddotY;
end