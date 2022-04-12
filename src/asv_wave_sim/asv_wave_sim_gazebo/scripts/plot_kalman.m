clear; clc;
%v_pos_dist = importdata("log_v_pos_dist.txt");
%v_vel_dist = importdata("log_v_vel_dist.txt");
%v_vel_dist_lp = importdata("log_v_vel_dist_lp.txt");
v_pos_dist = importdata("log_v_pos_dist_3_wave.txt");
v_vel_dist = importdata("log_v_vel_dist_3_wave.txt");
v_vel_dist_lp = importdata("log_v_vel_dist_lp_3_wave.txt");
v_vel_aruco = importdata("log_v_vel_aruco_3_wave.txt");
v_vel = importdata("log_v.txt");
%x(:,1) = [];
%v(:,1) = [];
%x(1,:) = [];
%v(1,:) = [];

% figure(1)
% subplot(2,2,1)
% plot(x(:,1)-5)
% hold on;
% plot(x(:,3))
% title('altitude drone')
% subplot(2,2,2)
% plot(x(:,2))
% title('velocity drone')
% subplot(2,2,3)
% plot(x(:,3))
% title('altitude ship')
% subplot(2,2,4)
% plot(x(:,4))
% title('velocity ship')

% figure(2)
% plot(v(:,1))
% hold on;
% plot(v(:,2)-1.4)

% figure(3)
% plot(v_vel_dist(:,1))
% hold on;
% plot(v_vel_dist(:,2)-1.4)

% figure(4)
% plot(v_pos_dist(:,3)-5)
% hold on;
% plot(v_vel_dist(:,3)-5)
% plot(v_vel_dist_lp(:,3)-5)
% xlim([0 800])
% legend('dist pos cont', 'dist vel cont', 'dist vel cont lp')
% 
% figure(5)
% plot(v_vel_aruco(:,3)-5)
% hold on;
% plot(v_vel_aruco(:,4)-5)
% plot(v_vel_aruco(:,4)-v_vel_aruco(:,3))
% xlim([0 800])
% legend('dist aruco', 'dist ground truth', 'dist diff')
% 
% figure(6)
% plot(v_vel_aruco(:,1))
% hold on;
% plot(v_vel_aruco(:,2)-1.4)
% legend('drone pos', 'ship pos')

figure(7)
plot(v_vel(:,1)+5.95)
hold on
plot(v_vel(:,2))

%%
avg_dist_pos = mean(abs(v_pos_dist(1:800,3)-5))
avg_dist_vel = mean(abs(v_vel_dist(1:800,3)-5))
avg_dist_vel_lp = mean(abs(v_vel_dist_lp(1:800,3)-5))
std_dist_pos = std(abs(v_pos_dist(1:800,3)-5))
std_dist_vel = std(abs(v_vel_dist(1:800,3)-5))
std_dist_vel_lp = std(abs(v_vel_dist_lp(1:800,3)-5))

