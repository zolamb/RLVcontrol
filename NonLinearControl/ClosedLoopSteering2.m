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
%       x1 -> vel mag (v)
%       x2 -> sideslip (beta)
%       x3 -> heading angle (phi)
%       x4 -> turning rate (phidot)
syms x1 x2 x3 x4

% Control variables: u1 u2 Ft psi
%       u1 -> F1 + F2
%       u2 -> F1 - F2
%       Ft -> Thrust
%       psi -> Thrust angle
syms u1 u2 Ft psi

% State Space
F = [(u1/m)*cos(x2) + (Ft/m)*cos(x2 - psi) - (Fw/m)*sin(x2 + x3);
     (u1/(x1*m))*cos(2*x3 + x2) + (Ft/(x1*m))*cos(2*x3 + x2 + psi) + (Fw/(x1*m)) - x4;
     x4;
     (width/(2*I))*u2 - (Ft/I)*(L/2 - bL)*sin(psi)];
 

%% Create State Space Model
%Initial condition:
x=0; y=0; xdot=0; ydot=1; phi=pi/2; phidot=0; u=1; w=0; beta=0;

% Creating list of state and control variables
stateVars = [x1 x2 x3 x4];
controlVars = [u1 u2 Ft psi];

% Take partial derivatives of F with respect to states
Ax = jacobian(F, stateVars);

% Take partial derivatives of F with respect to control inputs
Bx = jacobian(F, controlVars);

% Linearize about the trim state 'p'
p = [1 0 0 0 0 0 m*g 0]; 
v = [x1 x2 x3 x4 u1 u2 Ft psi];
A = double(subs(Ax,v,p));
B = double(subs(Bx,v,p));

% Determine controllability
rank(ctrb(A,B)) % This should equal the number of states

%% LQR Controller Design
% LQR Control
Q = [1e9 0 0 0;               % v
     0 1e3 0 0;               % beta
     0 0 0 0;                % phi
     0 0 0 1e13];              % phidot
R = [1e2 0 0 0;               % u1 = f1+f2
     0 1e2 0 0;               % u2 = f1-f2
     0 0 1e2 0;               % Ft
     0 0 0 1e18];              % Psi 
K = lqr(A, B, Q, R);

%% Control Block Design
% Target parking pose:
xP=0; yP=3000; phiP=pi/2; % we will use it in formulas for e and alpha

% Gains
gamma = 3;
h = 1;
k = 6;

% Initial conditions array form
Ustar = [0 0 m*g 0]';
ystar = [0, 0, 0, 0]';
y1Init =  [x, y, xdot, ydot, phi, phidot]';  

% Timestep
dt = 0.01;

% Data recordings
yrec1 = [];
trec1 = [];
trec2 = [];
actuatorsRec = [];
uRefrec = [];
wRefrec = [];

% Control Loop
e=sqrt((xP-x)^2+(yP-y)^2);
for i=1:700
% while(e>100)
    % Compute e, alpha, and theta
    e=sqrt((xP-x)^2+(yP-y)^2); % Distance between x,y and xP=0,yP=0
    theta=atan2(yP-y,xP-x)-phiP; % ThetaP is acting as an offset because the paper specifies equations where theta->0
    alpha=theta-(phi-phiP);
    alpha=atan2(sin(alpha),cos(alpha));
    
    % Update Controls 
    uRef = gamma*e*cos(alpha);
    if alpha <= 1e-50
      % lim alpha->0 (cos(alpha)*sin(alpha)/alpha) = 1
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)+...
          gamma*h*theta;
    else
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha;
    end
        
%     for j=1:1000
        disp(w)

        % Compute control input u = -K(y0-ystar)  --> (u1,u2,ft,psi)
        ystar = [uRef, 0, 0, wRef]';
        y0 = [u, beta, phi, w]';

        % uK defined as u(1)=f1+f2, u(2)=f1-f2, u(3)=ft, u(4)=psi
        uK = -K*(y0-ystar);

        % Converting to F1,F2 form
        uK = [(uK(1)+uK(2))/2 (uK(1)-uK(2))/2 uK(3) uK(4)]';
        U=Ustar+uK;

        % Apply saturations
        if U(1,1)>5*m*g
            U(1,1)=5*m*g;
        elseif U(1,1)<-5*m*g
           U(1,1)=-5*m*g; 
        else
           U(1,1)=U(1,1);
        end

        if U(2,1)>5*m*g
            U(2,1)=5*m*g;
        elseif U(2,1)<-5*m*g
           U(2,1)=-5*m*g; 
        else
           U(2,1)=U(2,1);
        end

        if U(3,1)>20*m*g
            U(3,1)=20*m*g;
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
        [t, y1] = ode45(@(t,y1)odeFunction5(y1, width, L, bL, m, Fw, I, U), [0 dt], y1Init);

        % Store last state and solve for current u,w,beta
        x = y1(end,1);
        y = y1(end,2);
        xdot = y1(end,3);
        ydot = y1(end,4);
        phi = y1(end,5);
        phidot = y1(end,6);
        u = sqrt(xdot^2 + ydot^2);
        w = phidot;
        beta = atan2(ydot,xdot) - phi;

        % Record results
        yrec1=[yrec1,y1'];
        trec1=[trec1, (i-1)*dt+t'];
        trec2=[trec2 (i-1)*dt+t(end)];
        tmp=[ones(1,length(t))*U(1,1);
             ones(1,length(t))*U(2,1);
             ones(1,length(t))*U(3,1);
             ones(1,length(t))*U(4,1)];
        actuatorsRec=[actuatorsRec tmp];
        uRefrec = [uRefrec uRef];
        wRefrec = [wRefrec wRef];

        % Create new initial conditions array
        y1Init = y1(end,:)';
%     end
end

% Compute the u,w,beta values and add to yRec
uList = sqrt(yrec1(3,:).^2 + yrec1(4,:).^2);
wList = yrec1(6,:);
betaList = atan2(yrec1(4,:),yrec1(3,:)) - yrec1(5,:);
yrec1 = [yrec1; uList; wList; betaList];

% Plot x,y results
figure(1)
plot(yrec1(1,:), yrec1(2,:))
title('Position')
axis equal

figure(2)
plot(trec1(1,:), actuatorsRec(1,:))
title('f1')

figure(3)
plot(trec1(1,:), actuatorsRec(2,:))
title('f2')

figure(4)
plot(trec1(1,:), actuatorsRec(3,:))
title('Ft')

figure(5)
plot(trec1(1,:), actuatorsRec(4,:))
title('psi')



% 
% figure(2)
% plot(trec1(1,:), yrec1(9,:))
% title('beta')
% 
% figure(3)
% plot(trec1(1,:), yrec1(5,:)*180/pi)
% title('phi')
% 
% figure(4)
% plot(trec1(1,:), yrec1(7,:))
% title('velocity')
% hold on;
% plot(trec2(1,:), uRefrec(1,:))
% hold off;
% 
% figure(5)
% plot(trec1(1,:), yrec1(8,:))
% title('omega')
% hold on;
% plot(trec2(1,:), wRefrec(1,:))
% hold off;
% 
% 
