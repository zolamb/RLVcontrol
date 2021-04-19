% Clear previous results
clear;
clc;

% Target parking pose:
xP=1000; yP=-1000; thetaP=0; % we will use it in formulas for e and alpha

%Initial condition:
x=0; y=0; phi=3*pi/4;

% Control vars
u = 0;      % Speed
w = 0;      % Angular speed

% Gains
gamma = 3;
h = 1;
k = 6;

% Timestep
dt = 0.01;

yrec = [];
urec = [];
wrec = [];
trec=[];

disp("here")
for i=1:500
    % Compute e, alpha, and theta
    % phi - robot heading, theta - heading of the parking pose
    e=sqrt((xP-x)^2+(yP-y)^2); %distance between x,y and xP=0,yP=0
    
    theta=atan2(yP-y,xP-x)-thetaP; % ThetaP is acting as an offset because the paper specifies equations where theta->0
%     theta=atan2(yP-y,xP-x);
    alpha=theta-(phi-thetaP);
%     alpha=theta-phi;
    alpha=atan2(sin(alpha),cos(alpha));

    % Update controls
    u = gamma*e*cos(alpha)
    if alpha <= 1e-50
      % lim alpha->0 (cos(alpha)*sin(alpha)/alpha) = 1
      w = k*alpha + gamma*cos(alpha)*sin(alpha)+...
          gamma*h*theta      
    else
      w = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha
    end
    
    % Update initial conditions
    y2Init=[x;y;phi];
    
    % Solve cartesian ODE for small timestep
    [t, y2] = ode45(@(t,y2)odeFunction32(t, y2, u, w), [0 dt], y2Init);
    
    % Record results
    yrec = [yrec, y2'];
    trec=[trec;t+(i-1)*dt*ones(1,length(t))'];
    x=y2(end,1);
    y=y2(end,2);
    phi=y2(end,3);
    urec=[urec u*ones(1,length(y2))];
    wrec=[wrec w*ones(1,length(y2))];
end

% Plot cartesian results
figure(1)
plot(yrec(1,:), yrec(2,:))
axis equal
figure(2)
plot(trec,yrec(3,:)*180/pi)
figure(3)
plot(trec,urec)
figure(4)
plot(trec,wrec)