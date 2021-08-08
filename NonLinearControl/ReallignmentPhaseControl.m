% Clear previous results
clear;
clc;

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

% CREATE AN ODEFUNCTION WITHOUT GRAVITY. WRITE OUT NEW SYS OF EQUATIONS