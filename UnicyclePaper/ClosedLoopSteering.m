y0 = [sqrt(2) -pi/4 -pi];
[t, y] = ode45(@(t,y)odeFunction2(y), [0 250], y0);
% polarplot(y(:,3), y(:,1))
plot(y(:,1).*cos(y(:,3)), y(:,1).*sin(y(:,3)))

% Simulate using eq.1
% calculate new u,w with the new x,y from simulating eq.1
