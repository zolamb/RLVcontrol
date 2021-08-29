% Clear previous results
clear;
clc;

% Target parking pose:
xP=3500; yP=3000; thetaP=0; % we will use it in formulas for e and alpha

%Initial condition:
x=0; y=0; phi=pi/2;

% Control vars
u = 0;      % Speed
w = 0;      % Angular speed

% Less aggressive gains
% gamma = 0.25;
% h = 1;
% k = 0.5;

% gamma = 0.01;
gamma = 0.01;
h = 0.01;
k = 0.08;

% More aggressive gains
% gamma = 3;
% h = 1;
% k = 6;

% Timestep
dt = 0.01;

% IC's
y2Init=[x;y;phi];
    
yrec = [];
urec = [];
wrec = [];
trec=[];

e = 1000;
for i=1:6000
    if(e < 100)
        disp("Here")
        break
    end
    % Compute e, alpha, and theta
    % phi - robot heading, theta - heading of the parking pose
    e=sqrt((xP-x)^2+(yP-y)^2); %distance between x,y and xP=0,yP=0
    theta=atan2(yP-y,xP-x)-thetaP; % ThetaP is acting as an offset because the paper specifies equations where theta->0
    alpha=theta-(phi-thetaP);
    alpha=atan2(sin(alpha),cos(alpha)); % Wrap angle to +- pi

    % Update controls
    u = 100;
    if alpha <= 1e-50 || e <= 1e-50
      w = k*alpha + gamma*u*sin(alpha)*(alpha+h*theta)/(alpha*e);
    else
      w = k*alpha + gamma*u*sin(alpha)*(alpha+h*theta)/(alpha*e);
    end
    
    % Solve cartesian ODE for small timestep
    [t, y2] = ode45(@(t,y2)odeFunction2(t, y2, u, w), [0 dt], y2Init);
    
    % Record results
    yrec = [yrec, y2'];
    trec=[trec;t+(i-1)*dt*ones(1,length(t))'];
    x=y2(end,1);
    y=y2(end,2);
    phi=y2(end,3);
    urec=[urec u*ones(1,length(y2))];
    wrec=[wrec w*ones(1,length(y2))];
    
    % Update IC's
    y2Init=[x;y;phi];
end

% Plot cartesian results
figure(1)
plot(yrec(1,:), yrec(2,:))
title("Position");
xlabel("x");
ylabel("y");
hold on;
plot(yrec(1,end), yrec(2,end), "bo")
axis equal
figure(2)
plot(trec,yrec(3,:)*180/pi)
figure(3)
plot(trec,urec)
title('velocity')
figure(4)
plot(trec,wrec)
title('omega')