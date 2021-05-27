function dx = odeUnicycle(t, x) % first parameter must be to be t
    % Cartesian system simulations
    global vRef;
    global wRef;
    
    % Newtonian set of ODEs
    xC=x(1);        %<- x positions
    yC=x(2);        %<- y positions
    phi=x(3);
    dx(1,1) = vRef*cos(phi);
    dx(2,1) = vRef*sin(phi);
    dx(3,1) = wRef;
end