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
I = 0.25*m*(width/2)^2 + (1/12)*m*L^2;  % inertia for a cylinder (1/2*m*r^2) (kg*m^2)

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
syms F1 F2 Ft psi

% State Space
F = [((F1 + F2)/m)*cos(x2) + (Ft/m)*cos(x2 - psi)  - (Fw/m)*sin(x2 + x3);
     -((F1 + F2)/(x1*m))*sin(x2) + (Ft/(x1*m))*sin(psi - x2) - (Fw/(x1*m))*cos(x2 + x3) - x4;
     x4;
     (width/(2*I))*(F1 - F2) - (Ft/I)*(L/2 - bL)*sin(psi)];

%% Create State Space Model
%Initial condition:
x=0; y=0; xdot=0; ydot=50; phi=pi/2; phidot=-6.53*pi/180; u=50; w=-6.53*pi/180; beta=0;

% Creating list of state and control variables
stateVars = [x1 x2 x3 x4];
controlVars = [F1 F2 Ft psi];

% Take partial derivatives of F with respect to states
Ax = jacobian(F, stateVars);

% Take partial derivatives of F with respect to control inputs
Bx = jacobian(F, controlVars);

% Linearize about the trim state 'p'
p = [u beta phi phidot 0 0 m*g 0]; 
v = [x1 x2 x3 x4 F1 F2 Ft psi];
A = double(subs(Ax,v,p));
B = double(subs(Bx,v,p));

% Determine controllability
rank(ctrb(A,B)) % This should equal the number of states

%% LQR Controller Design
% % LQR Control
% Q = [1e12 0 0 0;               % v
%      0 1e18 0 0;              x % beta
%      0 0 1 0;                % phi
%      0 0 0 1e12];              % phidot
% R = [1e-1 0 0 0;               % u1 = f1+f2
%      0 1e-1 0 0;               % u2 = f1-f2
%      0 0 1 0;               % Ft
%      0 0 0 1e18];              % Psi 
% K = lqr(A, B, Q, R);
% LQR Control
Q = [1e14 0 0 0;               % v
     0 1e18 0 0;               % beta
     0 0 1 0;                % phi
     0 0 0 1e18];              % phidot
R = [1 0 0 0;               % u1 = f1+f2
     0 1 0 0;               % u2 = f1-f2
     0 0 1 0;               % Ft
     0 0 0 1e14];              % Psi 
K = lqr(A, B, Q, R);

%% Control Block Design
% Target parking pose:
xP=3500; yP=1000; phiP=0; % we will use it in formulas for e and alpha

% Gains
% gamma = 3;
% h = 1;
% k = 6;

% Less aggressive gains
% gamma = 0.25;
% h = 1;
% k = 0.5;

% Less Less aggressive gains
gamma = 0.05;
h = 0.25;
k = 0.1;

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
e = 100;
% for i=1:4000
i = 1;
flag = 1;
while(e>5)
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
    
%     wRef*180/pi

    % Compute control input u = -K(y0-ystar)  --> (u1,u2,ft,psi)
    ystar = [uRef, 0, phi, wRef]';
    y0 = [u, beta, phi, w]';

    % uK defined as u(1)=f1+f2, u(2)=f1-f2, u(3)=ft, u(4)=psi
    uK = -K*(y0-ystar);

    % Converting to F1,F2 form
    % uK = [(uK(1)+uK(2))/2 (uK(1)-uK(2))/2 uK(3) uK(4)]';
    U=Ustar+uK;

    % Apply saturations
    if U(1,1)>50*m*g
        U(1,1)=50*m*g;
    elseif U(1,1)<-50*m*g
       U(1,1)=-50*m*g; 
    else
       U(1,1)=U(1,1);
    end

    if U(2,1)>50*m*g
        U(2,1)=50*m*g;
    elseif U(2,1)<-50*m*g
       U(2,1)=-50*m*g; 
    else
       U(2,1)=U(2,1);
    end

    if U(3,1)>50*m*g
        U(3,1)=50*m*g;
    elseif U(3,1)<=0
       U(3,1)=0; 
    else
       U(3,1)=U(3,1);
    end

    if U(4,1)>pi/4 % 6 degrees
       U(4,1)=pi/4;
    elseif U(4,1)<-pi/4
       U(4,1)=-pi/4; 
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
    y1Init = y1(end,:)'
    i = i + 1;
    if(i>2000)
        break
    end
end

% Compute the u,w,beta values and add to yRec
uList = sqrt(yrec1(3,:).^2 + yrec1(4,:).^2);
wList = yrec1(6,:);
betaList = atan2(yrec1(4,:),yrec1(3,:)) - yrec1(5,:);
yrec1 = [yrec1; uList; wList; betaList];

% Plot x,y results
load("refs0");

figure(1)
plot(yrec1(1,:), yrec1(2,:))
hold on;
plot(yrec(1,:), yrec(2,:), "m--")
plot(yrec1(1,end), yrec1(2,end), "bo")

for i=1:length(yrec1)
    if(mod(i,10000) == 0 && i < length(yrec1)-10000)
        quiver(yrec1(1,i), yrec1(2,i), cos(yrec1(5,i))*100, sin(yrec1(5,i))*100);
%         quiver(yrec1(1,i), yrec1(2,i), cos(yrec1(5,i) + yrec1(9,i))*25, sin(yrec1(5,i) + yrec1(9,i))*25);
    end
%     pause(0.1);
end

legend("Rocket Position","Kinematic Solution Reference", "Target Location", "location", 'northwest')
hold off;
title('Position')
xlabel("x(m)")
ylabel("y(m)")
grid on
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
plot(trec1(1,:), actuatorsRec(4,:)*180/pi)
title('psi')

figure(6)
plot(trec1(1,:), yrec1(5,:)*180/pi)
title('phi')



figure(7)
subplot(3,1,1)
plot(trec1(1,:), yrec1(7,:))
title('Velocity (v)', 'FontSize', 10)
hold on;
plot(trec2(1,:), uRefrec(1,:), "m--")
hold off;
xlabel("Time(s)")
ylabel("Velocity(m/s)")
grid on

% figure(8)
subplot(3,1,2)
plot(trec1(1,:), yrec1(8,:)*180/pi)
title('Angular Velocity (omega)', 'FontSize', 10)
hold on;
plot(trec2(1,:), wRefrec(1,:)*180/pi, "m--")
hold off;
xlabel("Time(s)")
ylabel("Angular Velocity(deg/s)")
grid on

% figure(9)
subplot(3,1,3)
plot(trec1(1,:), yrec1(9,:)*180/pi)
title('Sideslip (beta)', 'FontSize', 10)
hold on;
plot(trec1(1,:), zeros(1, length(trec1)), "m--");
hold off;
xlabel("Time(s)")
ylabel("Sideslip(deg)")
grid on









figure(8)
subplot(3,1,1)
plot(trec1(1,:), yrec1(1,:))
title('X Position')
hold on;
plot(trec2(1,:), ones(length(trec2(1,:)),1)*xP, "m--")
hold off;
xlabel("Time(s)")
ylabel("X")
grid on

% figure(8)
subplot(3,1,2)
plot(trec1(1,:), yrec1(2,:))
title('Y Position')
hold on;
plot(trec2(1,:), ones(length(trec2(1,:)),1)*yP, "m--")
hold off;
xlabel("Time(s)")
ylabel("Y")
grid on

% figure(9)
subplot(3,1,3)
plot(trec1(1,:), yrec1(5,:)*180/pi)
title('Heading Angle')
hold on;
plot(trec2(1,:), ones(length(trec2(1,:)),1)*phiP*180/pi, "m--")
hold off;
xlabel("Time(s)")
ylabel("Phi")
grid on