clear;
clc;
% y0 = [sqrt(2) -pi/4 -pi];
y0 = [-1 1 3*pi/4];
dt = 0.01;

u = 0;
w = 0;

yrec = [];

for i=1:10000
    [t, y] = ode45(@(t,y)odeFunction3(y, u, w), [0 dt], y0);
    e = sqrt(y(end,1)^2 + y(end,2)^2);
    theta = atan(y(end,2)/y(end,1));
    alpha = theta - y(end,3);
    u = 3*e*cos(alpha);
    w = 6*alpha + 3*cos(alpha)*sin(alpha)*(alpha+1*theta)/alpha;
    yrec = [yrec, y'];
    y0 = y(end,:)';
end

% polarplot(y(:,3), y(:,1))
plot(yrec(:,1), yrec(:,2))

% Simulate using eq.1
% calculate new u,w with the new x,y from simulating eq.1
