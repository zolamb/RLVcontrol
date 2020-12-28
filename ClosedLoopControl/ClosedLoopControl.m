% File: ClosedLoopControl.m
% Description:
%       Design an LQR controller to control our RLV model. Looks at both
%       the continuous time response as well as the discrete time response.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clear Data
clear;
clc;
close all;

%% Initialize Vehicle Constants
% Original Constants
% w = 10;             % width of rocket
% L = 40;             % length of rocket
% bL = 10;            % distance from center of rocket to center of mass
% m = 100;            % mass of rocket
% g = 9.81;           % acceleration due to gravity
% Fw = m*g;        % weight of rocket
% I = 0.5*m*(w/2)^2;  % inertia for a cylinder (1/2*m*r^2)

% RLV Physical Constants
w = 3.7;             % width of rocket (m)
L = 47.7;             % length of rocket (m)
bL = 15.0;            % distance from center of rocket to center of mass (m)
m = 250000.0;            % mass of rocket (kg)
g = 9.81;           % acceleration due to gravity (m/s^2)
Fw = m*g;           % weight of rocket (N)
I = 0.5*m*(w/2)^2;  % inertia for a cylinder (1/2*m*r^2) (kg*m^2)

%% Create System of ODE's
% State variables: x1 x2 x3 x4 x5 x6
%       x1 -> x position
%       x2 -> y position
%       x3 -> body angle
%       x4 -> x velocity
%       x5 -> y velocity
%       x6 -> body angular velocity
syms x1 x2 x3 x4 x5 x6

% Control variables: u1 u2 Ft psi
%       u1 -> F1 + F2
%       u2 -> F1 - F2
%       Ft -> Thrust
%       psi -> Thrust angle
syms u1 u2 Ft psi

% State Space
F = [x4;
     x5;
     x6;
     -(u1/m)*sin(x3) - (Ft/m)*sin(x3 + psi);
     (u1/m)*cos(x3) + (Ft/m)*cos(x3 + psi) - Fw/m;
     (w/(2*I))*u2 - (Ft/I)*(L/2 - bL)*sin(psi) 
     ];

%% Create State Space Model
% Creating list of state and control variables
stateVars = [x1 x2 x3 x4 x5 x6];
controlVars = [u1 u2 Ft psi];

% Take partial derivatives of F with respect to states
A = jacobian(F, stateVars);

% Take partial derivatives of F with respect to control inputs
B = jacobian(F, controlVars);

% Linearize about the trim state 'p'
p = [0 0 0 0 0 0 0 0 m*g 0];
v = [x1 x2 x3 x4 x5 x6 u1 u2 Ft psi];
A = double(subs(A,v,p));
B = double(subs(B,v,p));
C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
D = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

%% LQR Controller Design
% Determine controllability
rank(ctrb(A,B)) % This should equal the number of states - 6

% LQR Control
Q = [100 0 0 0 0 0;          % X pos
     0 100 0 0 0 0;          % Y pos
     0 0 1e7 0  0 0;         % Theta
     0 0 0 100 0  0;          % Vx
     0 0 0 0 100 0;           % Vy
     0 0 0 0 0 1e7];          % d(Theta)/dt
R = [100 0 0 0;              % u1 = f1+f2
     0 1 0 0;               % u2 = f1-f2
     0 0 1 0;               % Ft
     0 0 0 1000000];            % Psi 
K = lqr(A, B, Q, R);

%% Continuous Time Control
sys = ss((A-B*K), B, C, D);
y0 = [100 100 0 0 0 0];
figure(99)
initial(sys, y0)

%% Discrete Time Control
Ustar = [0 0 m*g 0]'; % control at the point of linearization  
ystar = [500, 1000, 0, 0, 0, 0]'; %state space point of linearization
y0 =  [520, 800, 0, 0, 0, 0]'; %initial state  
dt=0.01;
xRec=[];
tRec=[];
urec=[];
for i=1:5000
     u = -K*(y0-ystar);% feedback control (small signals) 
     % defined as u(1)=f1+f2, u(2)=f1-f2, u(3)=ft, u(4)=psi
     ur = [(u(1)+u(2))/2 (u(1)-u(2))/2 u(3) u(4)]'; % Converting to F1,F2 form
     U=Ustar+ur; % U rocket control (large signals)
     % control saturations (Physical Limitations ?)
     if U(3,1)>3*m*g
         U(3,1)=3*m*g;
     elseif U(3,1)<=0
        U(3,1)=0; 
     else
        U(3,1)=U(3,1);
     end
     if U(4,1)>pi/30 % 6 degrees
        U(4,1)=pi/30;
     elseif U(4,1)<-pi/30
        U(4,1)=-pi/30; 
     else
        U(4,1)=U(4,1);
     end
     % control saturations end
     [t, y] = ode45(@(t,y)odeFunction(y, w, L, bL, m, Fw, I, U), [0 dt], y0);
     xRec=[xRec,y'];
     tRec=[tRec, (i-1)*dt+t'];
     tmp=[ones(1,length(t))*U(1,1);
          ones(1,length(t))*U(2,1);
          ones(1,length(t))*U(3,1);
          ones(1,length(t))*U(4,1)];
     urec=[urec tmp];
     y0 = y(end,:)';
end

%% Plot Discrete Response
% Figure 1 - x position
figure(1)
plot(tRec,xRec(1,:));
title('X position')
ylabel('x (m)')
xlabel('t (s)')

% Figure 2 - y position
figure(2)
plot(tRec,xRec(2,:));
title('Y position')
ylabel('y (m)')
xlabel('t (s)')

% Figure 3 - body angle theta
figure(3)
plot(tRec,xRec(3,:)*180/pi); % *180/pi
title('Body angle theta')
ylabel('theta (deg)')
xlabel('t (s)')

% Figure 4 - Control inputs F1, F2, Ft
figure(4)
plot(tRec,urec(1,:),'r-'), hold on
plot(tRec,urec(2,:),'g-'), hold on
plot(tRec,urec(3,:),'b-'), hold on
title('Force vector control inputs')
legend('F1', 'F2', 'Ft')

% Figure 5 - Control input psi
figure(5)
plot(tRec,urec(4,:)*180/pi,'b-'), hold on
title('Thrust angle control input')
ylabel('Psi (deg)')
xlabel('t (s)')

% Figure 6 - Position
figure(6)
plot(xRec(1,:),xRec(2,:))
title('Position')
ylabel('y (m)')
xlabel('x (m)')

