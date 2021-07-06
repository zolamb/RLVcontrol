% File: NaturalResponse.m
% Description:
%       This script serves the purpose of verifying our 2D model
%       of the RLV. Expected results are in comments before each plot.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear Data
clear;
clc;
close all;

%% Initialize Vehicle Constants
% RLV Physical Constants
w = 3.7;                % width of rocket (m)
L = 47.7;               % length of rocket (m)
bL = 15.0;              % distance from center of rocket to center of mass (m)
m = 250000.0;           % mass of rocket (kg)
g = 9.81;               % acceleration due to gravity (m/s^2)
Fw = m*g;               % weight of rocket (N)
I = 0.25*m*(w/2)^2 + (1/12)*m*L^2;      % inertia for a cylinder (1/2*m*r^2) (kg*m^2)

%% Natural Response - Case 1
% Expected Result - Flies straight up
u = [0 0 2*m*g 0]; % [F1, F2, Ft, psi]
y0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, y0);

% Plot position
figure(1);
plot(y(:,1), y(:,2));
hold on;
plot(y(end,1), y(end,2), "bo");
title("Position");
xlabel("x (m)");
ylabel("y (m)");
grid on;

%% Natural Response - Case 2
% Expected Result - Hovers completely still
u = [0 0 m*g 0]; % [F1, F2, Ft, psi]
y0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, y0);

% Plot position
figure(2);
plot(y(:,1), y(:,2), "bo");
title("Position");
xlabel("x (m)");
ylabel("y (m)");
grid on;

%% Natural Response - Case 3
% Expected Result - Flies up and to the right
u = [100 1000 1.5*m*g 0]; % [F1, F2, Ft, psi]
y0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, y0);

% Plot position
figure(3);
grid on;
plot(y(:,1), y(:,2));
hold on;
plot(y(end,1), y(end,2), "bo");

for i=1:length(y)
    if(mod(i,100) == 0 && i < length(y)-50)
        quiver(y(i,1), y(i,2), -sin(y(i,3))*6000, cos(y(i,3))*6000);
    end
end

title("Position");
xlabel("x(m)");
ylabel("y(m)");
axis([-10000 10000 0 2.5e4]);
figure(4);
plot(t(:,1),y(:,3)*180/pi);
title("Heading angle");
xlabel("Time(s)");
ylabel("Theta (deg)");
grid on;

%% Natural Response - Case 4
% Expected Result - Flies up and to the left
u = [1000 100 1.5*m*g 0]; % [F1, F2, Ft, psi]
y0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, y0);

% Plot position
figure(5);
plot(y(:,1), y(:,2));
hold on;
plot(y(end,1), y(end,2), "bo");

for i=1:length(y)
    if(mod(i,100) == 0 && i < length(y)-50)
        quiver(y(i,1), y(i,2), -sin(y(i,3))*6000, cos(y(i,3))*6000);
    end
end

title("Position");
xlabel("x(m)");
ylabel("y(m)");
axis([-10000 10000 0 2.5e4]);
grid on;

figure(6);
plot(t(:,1),y(:,3)*180/pi);
title("Heading angle");
xlabel("Time(s)");
ylabel("Theta (deg)");
grid on;