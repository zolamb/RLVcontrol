% Clear previous results
clear;
clc;

 global vRef
 global wRef;

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
syms F1 F2 Ft psi

% State Space
F = [((F1 + F2)/m)*cos(x2) + (Ft/m)*cos(x2 - psi) - (Fw/m)*sin(x2 + x3);
     -((F1 + F2)/(x1*m))*sin(x2) + (Ft/(x1*m))*sin(psi - x2) - (Fw/(x1*m))*cos(x2 + x3) - x4;
     x4;
     (width/(2*I))*(F1 - F2) - (Ft/I)*(L/2 - bL)*sin(psi)];

%% Create State Space Model
%Initial condition:
x=2500; y=3500; 
xdot=0; ydot=1; 
phi=-pi/2; phidot=0; 
u=1; w=0; beta=0;

%% Control Block Design
% Target parking pose:
xP=5000; yP=0; phiP=-90*pi/180; % we will use it in formulas for e and alpha

% Gains
% gamma = 3;
% h = 1;
% k = 6;

% Less aggressive gains
% gamma = 0.25;
% h = 1;
% k = 0.5;

% Less Less aggressive gains
%gamma = 0.02*10;
%h = 0.25;
%k = 0.1*10;

 gamma = 3;
 h = 1;
 k = 6;


% Initial conditions array form
Ustar = [0 0 m*g 0]';
ystar = [0, 0, 0, 0]';
y1Init =  [x, y, xdot, ydot, phi, phidot]';  

% Timestep
dt = 0.05;

% Data recordings
yrec1 = [];
trec1 = [];
trec2 = [];
actuatorsRec = [];
uRefrec = [];
wRefrec = [];

tRec=[];
xCRef=[];
yCRef=[];
phiRef=[];
i=1;
e=sqrt((xP-x)^2+(yP-y)^2);
%Reference trajectory
while(e>0.01)
  % Compute e, alpha, and theta
  e=sqrt((xP-x)^2+(yP-y)^2); % Distance between x,y and xP=0,yP=0
  theta=atan2(yP-y,xP-x)-phiP; % ThetaP is acting as an offset because the paper specifies equations where theta->0
  alpha=theta-(phi-phiP);
  alpha=atan2(sin(alpha),cos(alpha));
  % Update velocity reference 
  vRef = gamma*e*cos(alpha);
  % Update turning rate reference 
  if alpha <= 1e-50
    % lim alpha->0 (cos(alpha)*sin(alpha)/alpha) = 1
    wRef = k*alpha + gamma*cos(alpha)*sin(alpha)+gamma*h*cos(alpha)*theta;
  else
    wRef = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha;
  end
  S0=[x;y;phi];
  vRefRec(i)=vRef;
  wRefRec(i)=wRef;
  eRec(i)=e;
  [tSol sol]=ode45(@odeUnicycle,[(i-1)*dt i*dt],S0);
  tRec=[tRec;tSol];
  xCRef=[xCRef;sol(:,1)];
  yCRef=[yCRef;sol(:,2)];
  phiRef=[phiRef;sol(:,3)];
  x=xCRef(end);
  y=yCRef(end);
  phi=phiRef(end);
  clear tSol;
  clear sol;
  i=i+1;
end
figure(1);
plot(xCRef,yCRef);
figure(2)
plot(tRec,phiRef*180/pi);
figure(3)
tSamp=[0:length(vRefRec)-1]*dt;
subplot(3,1,1),stairs(tSamp,vRefRec);
subplot(3,1,2),plot(tSamp,eRec);
subplot(3,1,3),stairs(tSamp,wRefRec);  





