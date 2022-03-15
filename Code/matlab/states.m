function stat = states(x)
    a=0;
    dt=0.1;
    stat = double.empty(11,0);
    stat(1) = x(1)+0.5*x(2)*dt+0.25*a*dt^2;
    stat(2) = x(2)+0.5*a*dt; %Fjernet et ^2 fra dt her.
    stat(3) = x(3)*cos(x(5)*dt)+x(4)*sin(x(5)*dt);
    stat(4) = -sin(x(5)*dt)*x(4)+cos(x(5)*dt)*x(4); %første x(4) tror jeg skal være x(3)
    stat(5) = x(5);
    stat(6) = x(6)*cos(x(8)*dt)+x(7)*sin(x(8)*dt);
    stat(7) = -sin(x(8)*dt)*x(6)+cos(x(8)*dt)*x(7);
    stat(8) = x(8);
    stat(9) = x(9)*cos(x(11)*dt)+x(10)*sin(x(11)*dt);
    stat(10) = -sin(x(11)*dt)*x(9)+cos(x(11)*dt)*x(10);
    stat(11) = x(11);

    
end