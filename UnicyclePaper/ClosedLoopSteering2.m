% Clear previous results
clear;
clc;

% These are not initial conditions, you can remove lines 5-8
% Initial conditions 
%y0Polar = [sqrt(2) -pi/4 -pi];
%y0Cartesian = [-1 1 3*pi/4];

% Let us assume that the target parking pose is 
xP=0; yP=0; thetaP=0; % we will use it in formulas for e and alpha

%Initial condition is (x,y,theta)
x=-1; y=1; phi=3*pi/4; % this is an example

% Control vars
u = 0;
w = 0;

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
for i=1:300,
    % Remove this
    % Solve polar ODE for small timestep 
    %[t, y] = ode45(@(t,y)odeFunction2(y, gamma, h, k), [0 dt], y0Polar);
    %y0Polar = y(end,:)'; 
    
    % Compute e and alpha for (x,y,phi) and (xP,yP,theta)
    % phi - robot heading, theta - heading of the parking pose
    e=sqrt((xP-x)^2+(yP-y)^2); %distance between x,y and xP=0,yP=0
    theta=atan2(yP-y,xP-x)-thetaP;
    alpha=theta-(phi-thetaP);
    alpha=atan2(sin(alpha),cos(alpha));
    %Control update
    % Find new u,w using the above values
    u = gamma*e*cos(alpha);
%     if alpha <= 1e-50
%       % lim alpha->0 (cos(alpha)*sin(alpha)/alpha) = 1
%       w = k*alpha + gamma*cos(alpha)*sin(alpha)+...
%           gamma*h*theta;      
%     else
      w = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha;
%     end
    % Remove 45-49
    % Extract e,alpha,theta from last values of ODE solution
    %e = y0Polar(1);
    %alpha = y0Polar(2);
    %theta = y0Polar(3);
    y2Init=[x;y;phi];
    % Solve cartesian ODE for small timestep
    [t, y2] = ode45(@(t,y2)odeFunction32(t, y2, u, w), [0 dt], y2Init);
    %[t, y2] = ode45(@odeFunction3,[0 dt],y2Init);
    yrec = [yrec, y2'];
    trec=[trec;t+(i-1)*dt*ones(1,length(t))'];
    %y0Cartesian = y2(end,:)';
    x=y2(end,1);
    y=y2(end,2);
    phi=y2(end,3);
    urec=[urec u*ones(1,length(y2))];
    wrec=[wrec w*ones(1,length(y2))];
    clear t;
    clear y2;
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