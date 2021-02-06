y0 = [5 60*pi/180];
[t, y] = ode45(@(t,y)odeFunction(y), [0 100], y0);
polarplot(y(:,2), y(:,1))