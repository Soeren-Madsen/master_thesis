clear; clc; format compact;
%VIRKER IKKE MED ACC, skifter kalman nogle gange når den rammer båden. Burg
%fix_velocity og dronens velocity lagt sammen ved touchdown.

max_vec_simple = []
max_vec_kalman = []
for i = 1:30
    imu_sim = importdata("log_acc_simple/log_acc_simple" + string(i) + ".csv");
    acc_sim = imu_sim.data(:,28);
    max_vec_simple = [max_vec_simple,max(acc_sim)];
%     imu_kal = importdata("log_acc_kalman/log_acc_kalman" + string(i) + ".csv");
%     acc_kal = imu_kal.data(:,28);
%     max_vec_kalman = [max_vec_kalman,max(acc_kal)];
end
test = importdata("log_acc_kalman/log_acc_kalman4.csv");
acc = test.data(:,28);
plot(acc)


mu1 = 395; %Det er student t test. Lektion 8 exercise 10 i statistik
mu2 = 435;
n1 = 10;
n2 = 16;
s1 = 15;
s2 = 30;
T=(mu1-mu2)/(sqrt(s1^2/n1+s2^2/n2));
v=((s1^2/n1+s2^2/n2)^2)/((s1^2/n1)^2/(n1-1)+(s2^2/n2)^2/(n2-1))
P_20=tcdf(T,23) %Indsæt hel tal af v i stedet for 23
