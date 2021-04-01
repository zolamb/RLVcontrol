% Clear previous results
clear;
clc;

%% Initialize Vehicle Constants
% RLV Physical Constants
width = 3.7;            % width of rocket (m)
L = 47.7;               % length of rocket (m)
bL = 15.0;              % distance from center of rocket to center of mass (m)
m = 250000.0;           % mass of rocket (kg)
g = 9.81;               % acceleration due to gravity (m/s^2)
Fw = m*g;               % weight of rocket (N)
I = 0.5*m*(width/2)^2;  % inertia for a cylinder (1/2*m*r^2) (kg*m^2)

%%
yrec = [];
trec = [];
dt = 0.01;

U = [0, 0, 2*m*g, 0];
y2Init = [0, pi/2, 0, 0, 0, 0, 0, 0];

for i=1:5000
    if(i==500)
        U = [0, 1000, 2*m*g, 0];
    end
    [t, y2] = ode45(@(t,y2)odeFunction4(y2, width, L, bL, m, Fw, I, U), [0 dt], y2Init);
    yrec = [yrec, y2'];
    y2Init = y2(end,:)';
end

figure(1)
plot(yrec(5,:),yrec(6,:))


