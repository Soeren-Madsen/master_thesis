clear; clc; format compact;

v_simple = importdata("log_vel_simple.txt")
v_kalman = importdata("log_vel_kalman.txt")
mu_sim = mean(v_simple(:,3))
mu_kal = mean(v_kalman(:,3))

delta = 1
mu1 = mu_sim; %Det er student t test. Lektion 8 exercise 10 i statistik
mu2 = mu_kal;
n1 = 30;
n2 = 30;
s1 = std(v_simple(:,3))
s2 = std(v_kalman(:,3))
T=(mu1-(mu2-1.09))/(sqrt(s1^2/n1+s2^2/n2));
v=((s1^2/n1+s2^2/n2)^2)/((s1^2/n1)^2/(n1-1)+(s2^2/n2)^2/(n2-1))
P=tcdf(T,floor(v))

figure(1)
boxplot([v_simple(:,3),v_kalman(:,3)])
ylim([-3.5,0])