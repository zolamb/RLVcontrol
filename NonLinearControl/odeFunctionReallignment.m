function dx = odeFunctionReallignment(x, width, L, bL, m, d, I, u) % first parameter must be to be t
    % Control inputs
    F3=u(1);
    F4=u(2);
    
    % Newtonian set of ODEs
    xC=x(1,1);        %<- x positions
    yC=x(2,1);        %<- y positions
    vx=x(3,1);        %<- vx velocity
    vy=x(4,1);        %<- vy velocity
    theta=x(5,1);     %<- body angle 
    dotTheta=x(6,1);  %<- body turning rate
    
    ddotX = F3*sin(theta)/m - F4*sin(theta)/m;
    ddotY = F4*cos(theta)/m - F3*cos(theta)/m;
    ddotTheta = (d/I)*F4 - (d/I)*F3;
    
    dx(1,1) = vx;
    dx(2,1) = vy;
    dx(3,1) = ddotX;
    dx(4,1) = ddotY;
    dx(5,1) = dotTheta; 
    dx(6,1) = ddotTheta;   
end