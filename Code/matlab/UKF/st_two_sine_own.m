function stat = states(x)
    a=0;
    dt=0.02;
    stat = [x(1)+x(2)*dt a*dt x(3)+x(5)*x(4)*dt x(4)-x(5)*x(3)*dt x(5) x(6)+x(8)*x(7)*dt x(7)-x(8)*x(6)*dt x(8)];
    
end