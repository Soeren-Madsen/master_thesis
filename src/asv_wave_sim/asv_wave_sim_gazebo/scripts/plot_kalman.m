clear; clc;
x = importdata("log_x.txt");
v = importdata("log_v.txt");
x(:,1) = [];
%v(:,1) = [];
x(1,:) = [];
%v(1,:) = [];

figure(1)
subplot(2,2,1)
plot(x(:,1)-5)
hold on;
plot(x(:,3))
title('altitude drone')
subplot(2,2,2)
plot(x(:,2))
title('velocity drone')
subplot(2,2,3)
plot(x(:,3))
title('altitude ship')
subplot(2,2,4)
plot(x(:,4))
title('velocity ship')

figure(2)
plot(v(:,1))
hold on;
plot(v(:,2)-1.4)
plot(v(:,3)-1.4)
