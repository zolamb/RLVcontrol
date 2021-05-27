function dx = ode2DRocketDyn(t, x) 
    global F1;
    global F2;
    global FT;
    global psi; 
    
    % Initialize Vehicle Constants
    % RLV Physical Constants
    width = 3.7;            % width of rocket (m)
    L = 47.7;               % length of rocket (m)
    bL = 15.0;              % distance from center of rocket to center of mass (m)
    m = 250000.0;           % mass of rocket (kg)
    g = 9.81;               % acceleration due to gravity (m/s^2)
    Fw = m*g;               % weight of rocket (N)
    I = 0.5*m*(width/2)^2;  % inertia for a cylinder (1/2*m*r^2) (kg*m^2)
  

   % Cartesian system simulations
    % Newtonian set of ODEs
    xC=x(1,1);        %<- x positions
    yC=x(2,1);        %<- y positions
    vx=x(3,1);        %<- vx velocity
    vy=x(4,1);        %<- vy velocity
    phi=x(5,1);     %<- body angle 
    dotphi=x(6,1);  %<- body turning rate
    
    %ddotX = (F1+F2)/m*cos(theta) + (Ft/m)*cos(theta + Psi);
    ddotX = (F1+F2)/m*cos(phi) + (FT/m)*cos(phi + psi);
    ddotY = (F1+F2)/m*sin(phi) + (FT/m)*sin(phi + psi) - Fw/m;
    ddotphi = (width/(2*I))*(F1-F2) - (FT/I)*(L/2 - bL)*sin(psi);
    
    dx(1,1) = vx;
    dx(2,1) = vy;
    dx(3,1) = ddotX;
    dx(4,1) = ddotY;
    dx(5,1) = dotphi; 
    dx(6,1) = ddotphi;   
end