function stat = states(x)
    a=0;
    dt=0.02;

    %two sine wave implemented as in paper, corrected x(4) to x(3) in first
    %-sin part
    %stat = [x(1)+0.5*x(2)*dt x(2) x(3)*cos(x(5)*dt)+x(4)*sin(x(5)*dt) -sin(x(5)*dt)*x(3)+cos(x(5)*dt)*x(4) x(5) x(6)*cos(x(8)*dt)+x(7)*sin(x(8)*dt) -sin(x(8)*dt)*x(6)+cos(x(8)*dt)*x(7) x(8)];
    
    %not corrected
    stat = [x(1)+0.5*x(2)*dt x(2) x(3)*cos(x(5)*dt)+x(4)*sin(x(5)*dt) -sin(x(5)*dt)*x(4)+cos(x(5)*dt)*x(4) x(5) x(6)*cos(x(8)*dt)+x(7)*sin(x(8)*dt) -sin(x(8)*dt)*x(6)+cos(x(8)*dt)*x(7) x(8)];
    
end