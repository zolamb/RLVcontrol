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



%% Create System of ODE's
% State variables: x1 x2 x3 x4 x5 x6
%       x1 -> body angle
%       x2 -> x velocity
%       x3 -> y velocity
%       x4 -> body angular velocity
syms x1 x2 x3 x4

% Control variables: u1 u2 Ft psi
%       u1 -> F1 + F2
%       u2 -> F1 - F2
%       Ft -> Thrust
%       psi -> Thrust angle
syms u1 u2 Ft psi

% State Space
F = [ x4;
     (u1/m)*cos(x1) + (Ft/m)*cos(x1 + psi);
     (u1/m)*sin(x1) + (Ft/m)*sin(x1 + psi) - Fw/m;
     (width/(2*I))*u2 - (Ft/I)*(L/2 - bL)*sin(psi) 
     ];


%% Create State Space Model
% Creating list of state and control variables
stateVars = [x1 x2 x3 x4];
controlVars = [u1 u2 Ft psi];

% Take partial derivatives of F with respect to states
A = jacobian(F, stateVars);

% Take partial derivatives of F with respect to control inputs
B = jacobian(F, controlVars);

% Linearize about the trim state 'p'
p = [pi/2 0 0 0 0 0 m*g 0];
v = [x1 x2 x3 x4 u1 u2 Ft psi];
A = double(subs(A,v,p));
B = double(subs(B,v,p));
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
D = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

%% LQR Controller Design
% Determine controllability
rank(ctrb(A,B)) % This should equal the number of states - 6

% LQR Control
Q = [0 0 0 0;           % Theta
     0 1e11 0 0;         % Vx
     0 0 1e8 0;         % Vy
     0 0 0 1e15];       % d(Theta)/dt
R = [1e-1 0 0 0;               % u1 = f1+f2
     0 1e-1 0 0;               % u2 = f1-f2
     0 0 1e-2 0;               % Ft
     0 0 0 1e7];            % Psi 
K = lqr(A, B, Q, R);

%% Control Block Design
% Target parking pose:
xP=0; yP=100; thetaP=pi/2; % we will use it in formulas for e and alpha

%Initial condition:
x=0; y=0; phi=pi/2;

% Gains
gamma = 3;
h = 1;
k = 6;

% Initial Conditions
u = 0;
w = 0;
x_dot = 0;
y_dot = 0;
phi_dot = 0;

% Initial conditions array form
Ustar = [0 0 m*g 0]';
ystar = [0, 0, 0, 0]'; % will be filled with x_dot, y_dot, theta_dot refs
y1Init =  [x, y, phi, 0, 0, 0]'; %initial state  

% Timestep
dt = 0.01;

% Data recordings
yrec1 = [];
trec1 = [];
actuatorsRec = [];
urec = [];
wrec = [];

% uRef,wRef error tolerance ~ 0.1m/s,0.1deg/s
% uReferrorTolerance = 0.1;
% wReferrorTolerance = 0.1*pi/180;

% Control Loop
for i=1:1
    %%%% Compute e, alpha, and theta %%%%
    e=sqrt((xP-x)^2+(yP-y)^2); %distance between x,y and xP=0,yP=0
    theta=atan2(yP-y,xP-x)-thetaP; % ThetaP is acting as an offset because the paper specifies equations where theta->0
    alpha=theta-(phi-thetaP);
    alpha=atan2(sin(alpha),cos(alpha));
    
    %%%%% Update Controls %%%%
    uRef = gamma*e*cos(alpha)
    if alpha <= 1e-50
      % lim alpha->0 (cos(alpha)*sin(alpha)/alpha) = 1
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)+...
          gamma*h*theta
    else
      wRef = k*alpha + gamma*cos(alpha)*sin(alpha)*(alpha+h*theta)/alpha
    end
    
    %%%%% Solve for reference velocities %%%%%
    y_dot_ref = sqrt(uRef^2 / (1 + tan(wRef)^2)) %%%%%%%%%%%%%%%%
    x_dot_ref = y_dot_ref * tan(wRef) %%%%%%%%%%%%%%%%%%%%%%%%%
    phi_dot_ref = wRef

    % Loop the actuator commands to get to uRef,wRef
%     while(abs(uRef - u) > uReferrorTolerance || abs(wRef - w) > wReferrorTolerance)
%     while(x_dot ~= x_dot_ref && y_dot ~= y_dot_ref && phi_dot ~= phi_dot_ref)
    for j=1:1000
        disp(y_dot);
        %%%% Compute control input u = -K(y0-ystar)  --> (u1,u2,ft,psi)
        ystar = [0, x_dot_ref, y_dot_ref, phi_dot_ref]';
        y0 = [phi, x_dot, y_dot, phi_dot]';
        
        
        % uK defined as u(1)=f1+f2, u(2)=f1-f2, u(3)=ft, u(4)=psi
        uK = -K*(y0-ystar);
        % Converting to F1,F2 form
        uK = [(uK(1)+uK(2))/2 (uK(1)-uK(2))/2 uK(3) uK(4)]';
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
        [t, y1] = ode45(@(t,y1)odeFunction3(y1, width, L, bL, m, Fw, I, U), [0 dt], y1Init);

        % Store last x,y,phi position and solve for current u,w
        x=y1(end,1);
        y=y1(end,2);
        phi=y1(end,3);
        x_dot = y1(end,4);
        y_dot = y1(end,5);
        phi_dot = y1(end,6);

        % Record results
        yrec1=[yrec1,y1'];
        trec1=[trec1, (i-1)*dt+t'];
        tmp=[ones(1,length(t))*U(1,1);
             ones(1,length(t))*U(2,1);
             ones(1,length(t))*U(3,1);
             ones(1,length(t))*U(4,1)];
        actuatorsRec=[actuatorsRec tmp];
%         urec=[urec u*ones(1,length(y1))];
%         wrec=[wrec w*ones(1,length(y1))];
        
        % Create new initial conditions array
        y1Init = y1(end,:)'; % May need y1(end,3)-90deg. %%%%%%%%%%%%%%% Probably not here tho
    end
end

% Plot cartesian results
figure(1)
plot(yrec1(1,:), yrec1(2,:))
axis equal
% figure(2)
% plot(trec,yrec2(3,:)*180/pi)
% figure(3)
% plot(trec,urec)
% figure(4)
% plot(trec,wrec)