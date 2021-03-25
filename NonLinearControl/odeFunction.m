function dx = odeFunction(x, w, L, bL, m, Fw, I, u) % first parameter must be to be t
    % Control inputs
    F1=u(1);
    F2=u(2);
    Ft=u(3);
    Psi=u(4);

    % Newtonian set of ODEs
    ddotX = (F1+F2)/m*cos(x(3)) + (Ft/m)*cos(x(3) + Psi);
    ddotY = (F1+F2)/m*sin(x(3)) + (Ft/m)*sin(x(3) + Psi) - Fw/m;
    ddotTheta = (w/(2*I))*(F1-F2) - (Ft/I)*(L/2 - bL)*sin(Psi);

    % Solve ODEs
    dx(1,1) = sqrt(ddotX^2 + ddotY^2);
    dx(2,1) = ddotTheta;
    dx(3,1) = x(2); % Solving for theta
end