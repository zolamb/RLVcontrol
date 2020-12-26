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
w = 3.7;             % width of rocket (m)
L = 47.7;             % length of rocket (m)
bL = 15.0;            % distance from center of rocket to center of mass (m)
m = 250000.0;            % mass of rocket (kg)
g = 9.81;           % acceleration due to gravity (m/s^2)
Fw = m*g;           % weight of rocket (N)
I = 0.5*m*(w/2)^2;  % inertia for a cylinder (1/2*m*r^2) (kg*m^2)

%% Natural Response - Case 1
% Expected Result - Flies straight up
u = [0 0 2*m*g 0]; % [F1, F2, Ft, psi]
x0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, x0);

% Plot position
figure(1);
plot(y(:,1), y(:,2));
title("Position");
xlabel("x (m)");
ylabel("y (m)");
grid on;

%% Natural Response - Case 2
% Expected Result - Hovers completely still
u = [0 0 m*g 0]; % [F1, F2, Ft, psi]
x0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, x0);

% Plot position
figure(2);
plot(y(:,1), y(:,2), "bo");
title("Position");
xlabel("x (m)");
ylabel("y (m)");
grid on;

%% Natural Response - Case 3
% Expected Result - Flies up and to the right
u = [95 100 2*m*g 0]; % [F1, F2, Ft, psi]
x0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, x0);

% Plot position
figure(3);
plot(y(:,1), y(:,2));
title("Position");
xlabel("x (m)");
ylabel("y (m)");
grid on;

%% Natural Response - Case 4
% Expected Result - Flies up and to the left
u = [100 95 2*m*g 0]; % [F1, F2, Ft, psi]
x0 = [0, 0, 0, 0, 0, 0];
tspan = 0:0.2:100;
[t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, u), tspan, x0);

% Plot position
figure(4);
plot(y(:,1), y(:,2));
title("Position");
xlabel("x (m)");
ylabel("y (m)");
grid on;