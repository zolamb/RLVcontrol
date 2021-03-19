% Clear previous results
clear;
clc;

% Initial conditions
y0Polar = [sqrt(2) -pi -pi/4];
y0Cartesian = [-1 1 3*pi/4];

% Control vars
% u = 0;
% w = 0;

% Gains
gamma = 3;
h = 1;
k = 6;

u = sqrt(2)*gamma*cos(-pi);
w = k*-pi + gamma*cos(-pi)*sin(-pi)*(-pi + h*(-pi/4));

% Timestep
dt = 0.1;

yrec = [];
for i=1:100
    % Solve polar ODE for small timestep
    [t, y] = ode45(@(t,y)odeFunction2(y, gamma, h, k), [0 dt], y0Polar);
    y0Polar = y(end,:)';
    
    % Extract e,alpha,theta from last values of ODE solution
    e = y0Polar(1);
    alpha = y0Polar(2);
    theta = y0Polar(3);
    
    % Find new u,w using the above values
    u = gamma*e*cos(alpha);
    w = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha;
    
    % Solve cartesian ODE for small timestep
    [t, y2] = ode45(@(t,y2)odeFunction3(y2, u, w), [0 dt], y0Cartesian);
    yrec = [yrec, y2'];
    y0Cartesian = y2(end,:)';
end

% Plot cartesian results
plot(yrec(1,:), yrec(2,:))

