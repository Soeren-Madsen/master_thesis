function stat = states(x)
    a=0;
    dt=0.1;
    
    %One sine wave
    stat = [x(1)+x(2)*dt a*dt x(3)+x(5)*x(4)*dt x(4)-x(5)*x(3)*dt x(5)];
    
end