function stat = states(x)
    a=0;
    dt=0.1;
    omega = 2;
    %No freq
    %stat = [x(1)+x(2)*dt a*dt x(3)+omega*x(4)*dt x(4)-omega*x(3)*dt] ;
    
    %One sine wave
    %stat = [x(1)+x(2)*dt a*dt x(3)+x(5)*x(4)*dt x(4)-x(5)*x(3)*dt x(5)];
    
    %One sine wave their states Corrected x(4) to x(3) in -sin
    stat = [x(1)+0.5*x(2)*dt x(2) x(3)*cos(x(5)*dt)+x(4)*sin(x(5)*dt) -sin(x(5)*dt)*x(3)+cos(x(5)*dt)*x(4) x(5)];
    
    %two sine waves
    %stat = [x(1)+x(2)*dt a*dt x(3)+x(5)*x(4)*dt x(4)-x(5)*x(3)*dt x(5) x(6)+x(8)*x(7)*dt x(7)-x(8)*x(6)*dt x(8)];
    
    %Two sine waves amp est
    %stat = [x(1)+x(2)*dt a*dt x(3) x(4) x(5) x(6)];
    
    %two sine wave, their states Corrected x(4) to x(3) in -sin
    %stat = [x(1)+0.5*x(2)*dt x(2) x(3)*cos(x(5)*dt)+x(4)*sin(x(5)*dt) -sin(x(5)*dt)*x(3)+cos(x(5)*dt)*x(4) x(5) x(6)*cos(x(8)*dt)+x(7)*sin(x(8)*dt) -sin(x(8)*dt)*x(6)+cos(x(8)*dt)*x(7) x(8)];
    
    %three sine waves
    %stat = [x(1)+x(2)*dt a*dt x(3)+x(5)*x(4)*dt x(4)-x(5)*x(3)*dt x(5) x(6)+x(8)*x(7)*dt x(7)-x(8)*x(6)*dt x(8) x(9)+x(11)*x(10)*dt x(10)-x(11)*x(9)*dt x(11)]; 
    
end