% Clear previous results
clear;
close all;
clc;


 global vRef
 global wRef;
 global F1;
 global F2;
 global FT;
 global Fw;
 global psi;
 

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
x=0; y=0; 
xdot=0; ydot=7500; phi=pi/2; phidot=0; 
u=1; w=0; beta=0;

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
% LQR Control
Q = [1e15 0 0 0;               % v
     0 1e8 0 0;               % beta
     0 0 1 0;                % phi
     0 0 0 1e15];              % phidot
R = [1e2 0 0 0;               % u1 = f1+f2
     0 1e2 0 0;               % u2 = f1-f2
     0 0 1e2 0;               % Ft
     0 0 0 1e17];              % Psi 
K = lqr(A, B, Q, R);

%% Control Block Design
% Target parking pose:
xP=2500; yP=3500; phiP=90*pi/180; % we will use it in formulas for e and alpha

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
xRefRec(i)=x;
yRefRec(i)=y;
vRefRec(i)=ydot;
wRefRec(i)=0;
i=i+1; 
%Reference trajectory
tsamp=[];
while(e>0.01)
  % Compute e, alpha, and theta
  tsamp=[tsamp,(i-1)*dt];
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
  xRefRec(i)=x;
  yRefRec(i)=y;
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
  phiRefRec(i)=phi;
  clear tSol;
  clear sol;
  i=i+1;
end
figure(1);
plot(xCRef,yCRef),hold on;
figure(3)
tSamp=[0:length(vRefRec)-1]*dt;
subplot(4,1,1),stairs(tSamp,vRefRec),ylabel('vRef');
subplot(4,1,2),plot(tSamp,eRec),ylabel('e');
subplot(4,1,3),stairs(tSamp,wRefRec),ylabel('wRef'); 
subplot(4,1,4),stairs(tSamp,phiRefRec*180/pi),ylabel('phiRef');  
accx=diff(diff(xRefRec))/dt^2;
accy=diff(diff(yRefRec))/dt^2;
ddphi=diff(diff(phiRefRec))/dt^2;
for i=1:length(accx),
  FTref(i)=sqrt(m^2*accx(i)^2+(m*accy(i)+Fw)^2);
  a(i)=sqrt(accx(i)^2+accy(i)^2);
  psiRef(i)=asin(-ddphi(i)*I/(FTref(i)*(L/2 - bL)));
end    
figure(2)
subplot(4,1,1),plot(yRefRec),ylabel('yref');
subplot(4,1,2),stairs(psiRef*180/pi),ylabel('psiRef');
subplot(4,1,3),stairs(ddphi),ylabel('ddphi');
subplot(4,1,4),plot(tRec,phiRef*180/pi),ylabel('phiRef');

figure(5);
subplot(4,1,1),stairs(accx),ylabel('a_x');
subplot(4,1,2),stairs(accy),ylabel('a_y');
subplot(4,1,3),stairs(a),ylabel('a');
subplot(4,1,4),stairs(FTref),ylabel('FT');


