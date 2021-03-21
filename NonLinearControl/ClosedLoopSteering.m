% Clear previous results
clear;
clc;

%% Initialize Vehicle Constants
% RLV Physical Constants
w = 3.7;            % width of rocket (m)
L = 47.7;           % length of rocket (m)
bL = 15.0;          % distance from center of rocket to center of mass (m)
m = 250000.0;       % mass of rocket (kg)
g = 9.81;           % acceleration due to gravity (m/s^2)
Fw = m*g;           % weight of rocket (N)
I = 0.5*m*(w/2)^2;  % inertia for a cylinder (1/2*m*r^2) (kg*m^2)

%% Create System of ODE's for u,w,theta
% State variables: x1 x2 x3
%       x1 -> vel
%       x2 -> angular vel
%       x3 -> body angle
syms x1 x2 x3

% Control variables: u1 u2 Ft psi
%       u1 -> F1 + F2
%       u2 -> F1 - F2
%       Ft -> Thrust
%       psi -> Thrust angle
syms u1 u2 Ft psi

% State Space
ddotX = -(u1)/m*sin(x3) - (Ft/m)*sin(x3 + psi);
ddotY = (u1)/m*cos(x3) + (Ft/m)*cos(x3 + psi) - Fw/m;
ddotTheta = (w/(2*I))*(u2) - (Ft/I)*(L/2 - bL)*sin(psi);
F = [sqrt(ddotX^2 + ddotY^2);
     ddotTheta;
     x2 
     ];

%% Create State Space Model
% Creating list of state and control variables
stateVars = [x1 x2 x3];
controlVars = [u1 u2 Ft psi];

% Take partial derivatives of F with respect to states
A = jacobian(F, stateVars);

% Take partial derivatives of F with respect to control inputs
B = jacobian(F, controlVars);

% Linearize about the trim state 'p'
p = [0 0 0 0 0 0 0];
v = [x1 x2 x3 u1 u2 Ft psi];
A = double(subs(A,v,p));
B = double(subs(B,v,p));
C = [1 0 0;
     0 1 0;
     0 0 1];
D = [0 0 0 0;
     0 0 0 0;
     0 0 0 0];

%% LQR Controller Design
% Determine controllability
rank(ctrb(A,B)) % This should equal the number of states - 3

% LQR Control
Q = [100 0 0;         % vel
     0 1e7 0;         % angular vel
     0 0 1e7];              % angle pos.
R = [1 0 0 0;               % u1 = f1+f2
     0 1 0 0;               % u2 = f1-f2
     0 0 1 0;               % Ft
     0 0 0 1e9];            % Psi 
K = lqr(A, B, Q, R);




%% Control Block Design
% Target parking pose:
xP=5; yP=-5; thetaP=0; % we will use it in formulas for e and alpha

%Initial condition:
x=-1; y=1; phi=3*pi/4;

% Gains
gamma = 3;
h = 1;
k = 6;


% Initial condition for 


% Timestep
dt = 0.01;

yrec = [];
urec = [];
wrec = [];
trec=[];
for i=1:500
    %%%% Compute e, alpha, and theta %%%%
    e=sqrt((xP-x)^2+(yP-y)^2); %distance between x,y and xP=0,yP=0
    theta=atan2(yP-y,xP-x)-thetaP; % ThetaP is acting as an offset because the paper specifies equations where theta->0
    alpha=theta-(phi-thetaP);
    alpha=atan2(sin(alpha),cos(alpha));
    
    
    %%%%% Update Controls %%%%
    uRef = gamma*e*cos(alpha);
    if alpha <= 1e-50
      % lim alpha->0 (cos(alpha)*sin(alpha)/alpha) = 1
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)+...
          gamma*h*theta;
    else
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha;
    end
    
    
    
    
    
    %%%% Compute u = -K(y0-ystar)  --> (u1,u2,ft,psi)
    %%%% convert to (f1,f2,ft,psi)
    %%%% apply saturations
    %%%% solve ODE for x1...x6 (this is the true position using actuators)(MIGHT NEED TO make theta-90)
    %%%% compute actual u,w from the solution (u=sqrt(dotx^2 + doty^2),w=dotTheta)
    %%%% Repeat
    
    
    
    
    % --------------------NOT DOING THIS ODE SOLUTION----------------------
%     %%%% Solve Cartesian ODE %%%%
%     % Update initial conditions
%     y2Init=[x;y;phi];
%     
%     % Solve cartesian ODE for small timestep
%     [t, y2] = ode45(@(t,y2)odeFunction2(y2, uRef, wRef), [0 dt], y2Init);
    % --------------------NOT DOING THIS ODE SOLUTION----------------------
    % Record results
%     yrec = [yrec, y2'];
%     trec=[trec;t+(i-1)*dt*ones(1,length(t))'];
%     x=y2(end,1);
%     y=y2(end,2);
%     phi=y2(end,3);
%     urec=[urec u*ones(1,length(y2))];
%     wrec=[wrec w*ones(1,length(y2))];
    % --------------------NOT DOING THIS RECORDING ------------------------
    
    
    
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