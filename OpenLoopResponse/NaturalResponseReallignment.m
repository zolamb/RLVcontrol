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
width = 3.7;            % width of rocket (m)
L = 47.7;               % length of rocket (m)
aL = 3.7;               % length from top of rocket to gas thrusters (m)
bL = 15.0;              % distance from center of rocket to center of mass (m)
d = L/2 + bL - aL;      % distance from center of mass to gas thrusters (m)
m = 250000.0;           % mass of rocket (kg)
g = 9.81;               % acceleration due to gravity (m/s^2)
Fw = m*g;               % weight of rocket (N)
I = 0.25*m*(width/2)^2 + (1/12)*m*L^2;  % inertia for a cylinder (1/2*m*r^2) (kg*m^2)

%% Natural Response - Case 1
% Expected Result - rotates to face opposite direction
u = [0 500]; % [F3, F4]
y0 = [0, 0, 50, 0, -pi/2, 0];
tspan = 0:0.2:150;
[t, y] = ode45(@(t,y)odeFunctionReallignment(y, width, L, bL, m, d, I, u), tspan, y0);

% Plot position
figure(1);
grid on;
plot(y(:,1), y(:,2));
hold on;
plot(y(end,1), y(end,2), "bo");
quiver(y(1,1), y(1,2), -sin(y(1,5))*6000, cos(y(1,5))*6000);
for i=1:length(y)
    if(mod(i,100) == 0 && i < length(y)-50)
        quiver(y(i,1), y(i,2), -sin(y(i,5))*6000, cos(y(i,5))*6000);
    end
end

title("Position");
xlabel("x(m)");
ylabel("y(m)");
axis([-10000 10000 -2.5e4 2.5e4]);
figure(4);
plot(t(:,1),y(:,5)*180/pi);
title("Heading angle");
xlabel("Time(s)");
ylabel("Theta (deg)");
grid on;