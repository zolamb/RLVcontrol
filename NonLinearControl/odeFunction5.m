function dx = odeFunction5(x, width, L, bL, m, Fw, I, u) % first parameter must be to be t
    % Control inputs
    F1=u(1);
    F2=u(2);
    Ft=u(3);
    Psi=u(4);
    
    % Newtonian set of ODEs
    xC=x(1,1);        %<- x positions
    yC=x(2,1);        %<- y positions
    vx=x(3,1);        %<- vx velocity
    vy=x(4,1);        %<- vy velocity
    theta=x(5,1);     %<- body angle 
    dotTheta=x(6,1);  %<- body turning rate
    
    ddotX = (F1+F2)/m*cos(theta) + (Ft/m)*cos(theta + Psi);
    ddotY = (F1+F2)/m*sin(theta) + (Ft/m)*sin(theta + Psi);
    ddotTheta = (width/(2*I))*(F1-F2) - (Ft/I)*(L/2 - bL)*sin(Psi);
    
    dx(1,1) = vx;
    dx(2,1) = vy;
    dx(3,1) = ddotX;
    dx(4,1) = ddotY;
    dx(5,1) = dotTheta; 
    dx(6,1) = ddotTheta;   
end