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



%% Create System of ODE's for u,w,theta
% State variables: x1 x2 x3 x4 x5 x6 x7
%       x1 -> magnitude of vel
%       x2 -> angular vel
%       x3 -> x vel
%       x4 -> y vel
%       x5 -> theta
%       x6 -> x
%       x7 -> y
syms x1 x2 x3 x4 x5 x6 x7

% Control variables: u1 u2 Ft psi
%       u1 -> F1 + F2
%       u2 -> F1 - F2
%       Ft -> Thrust
%       psi -> Thrust angle
syms u1 u2 Ft psi

% State Space
ddotX = (u1)/m*cos(x5) + (Ft/m)*cos(x5 + psi);
ddotY = (u1)/m*sin(x5) + (Ft/m)*sin(x5 + psi) - Fw/m;
ddotTheta = (width/(2*I))*(u2) - (Ft/I)*(L/2 - bL)*sin(psi);
F = [(ddotX*x3 + ddotY*x4) / sqrt(x3^2 + x4^2)
     ddotTheta;
     ddotX;
     ddotY;
     x2
     x3
     x4];

%% Create State Space Model
%Initial condition:
u=0.1; w=0; xdot=0; ydot=0.1; phi=pi/2; x=0; y=0;

% Creating list of state and control variables
stateVars = [x1 x2 x3 x4 x5 x6 x7];
controlVars = [u1 u2 Ft psi];

% Take partial derivatives of F with respect to states
Ax = jacobian(F, stateVars);

% Take partial derivatives of F with respect to control inputs
Bx = jacobian(F, controlVars);

% Linearize about the trim state 'p'
p = [0.1 0 0 0.1 pi/2 0 0 0 0 m*g 0];
v = [x1 x2 x3 x4 x5 x6 x7 u1 u2 Ft psi];
A = double(subs(Ax,v,p));
B = double(subs(Bx,v,p));
C = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 1 0 0 0 0;
     0 0 0 1 0 0 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];
D = [0 0 0 0;
     0 0 0 0;
     0 0 0 0];

%% LQR Controller Design
% Determine controllability
rank(ctrb(A,B)) % This should equal the number of states - 3

% LQR Control
Q = [1 0 0 0 0 0 0;              % vel e10, e11
     0 1 0 0 0 0 0;              % angular vel
     0 0 1 0 0 0 0;
     0 0 0 1 0 0 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];                % heading angle = 0 because it doesn't matter
R = [1 0 0 0;               % u1 = f1+f2
     0 1 0 0;               % u2 = f1-f2
     0 0 1 0;            % Ft
     0 0 0 1];            % Psi 
K = lqr(A, B, Q, R);

%% Control Block Design
% Target parking pose:
xP=-500; yP=1500; thetaP=pi/2; % we will use it in formulas for e and alpha

% Gains
gamma = 3;
h = 1;
k = 6;

% Initial conditions array form
Ustar = [0 0 0 0]';
ystar = [0, 0, 0]';
y1Init =  [u, w, xdot, ydot, phi, x, y]'; %initial state  

% Timestep
dt = 0.01;

% Data recordings
yrec1 = [];
trec1 = [];
actuatorsRec = [];
urec = [];
wrec = [];

% uRef,wRef error tolerance ~ 0.1m/s,0.1deg/s
uReferrorTolerance = 5;
wReferrorTolerance = 0.1;

% Control Loop
for i=1:1 % ------ LEAVING THIS AT 1 TO JUST TEST IF MY u AND w TRACK TO REFERENCES ---------------------
    %%%% Compute e, alpha, and theta %%%%
    e=sqrt((xP-x)^2+(yP-y)^2); %distance between x,y and xP=0,yP=0
    theta=atan2(yP-y,xP-x)-thetaP; % ThetaP is acting as an offset because the paper specifies equations where theta->0
    alpha=theta-(phi-thetaP);
    alpha=atan2(sin(alpha),cos(alpha));
    
    %%%% Update Controls %%%%
    uRef = gamma*e*cos(alpha)
    if alpha <= 1e-50
      % lim alpha->0 (cos(alpha)*sin(alpha)/alpha) = 1
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)+...
          gamma*h*theta
    else
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha
    end
    
    %%%% INNER LOOP %%%%
%     while(abs(uRef - u) > uReferrorTolerance || abs(wRef - w) > wReferrorTolerance)
    for j=1:20000 % ------ USING THIS LOOP TO JUST TEST IF MY u AND w TRACK TO REFERENCES ---------------------
        disp(w);
        %%%% Compute control input u = -K(y0-ystar)  --> (u1,u2,ft,psi)
        ystar = [uRef, wRef, 0]';
        y0 = [u, w, 0]';
        
        
        % uK defined as u(1)=f1+f2, u(2)=f1-f2, u(3)=ft, u(4)=psi
        uK = -K*(y0-ystar);
        % Converting to F1,F2 form
        uK = [(uK(1)+uK(2))/2 (uK(1)-uK(2))/2 uK(3) uK(4)]';
        uK=uK*-1;
        U=Ustar+uK; % U rocket control (linear control)

        % Apply saturations
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

        % Solve ODE for new position
        [t, y1] = ode45(@(t,y1)odeFunction4(y1, width, L, bL, m, Fw, I, U), [0 dt], y1Init);
        
        % Store last x,y,phi position and solve for current u,w
        u=y1(end,1);
        w=y1(end,2);
        phi=y1(end,5);
        x = y1(end,6);
        y = y1(end,7);

        % Record results
        yrec1=[yrec1,[x y phi u w]'];
        trec1=[trec1, (i-1)*dt+t'];
        tmp=[ones(1,length(t))*U(1,1);
             ones(1,length(t))*U(2,1);
             ones(1,length(t))*U(3,1);
             ones(1,length(t))*U(4,1)];
        actuatorsRec=[actuatorsRec tmp];

        % Create new initial conditions array
        y1Init = y1(end,:)';
    end
end

% Plot x,y results
figure(1)
plot(yrec1(1,:), yrec1(2,:))
axis equal

% Plot thrust force
figure(2)
plot(trec1(1,:), actuatorsRec(3,:))