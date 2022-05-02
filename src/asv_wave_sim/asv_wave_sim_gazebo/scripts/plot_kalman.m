clear; clc;
%v_pos_dist = importdata("log_v_pos_dist.txt");
%v_vel_dist = importdata("log_v_vel_dist.txt");
%v_vel_dist_lp = importdata("log_v_vel_dist_lp.txt");
%v_pos_dist = importdata("log_v_pos_dist_3_wave.txt");
%v_vel_dist = importdata("log_v_vel_dist_3_wave.txt");
%v_vel_dist_lp = importdata("log_v_vel_dist_lp_3_wave.txt");
%v_vel_aruco = importdata("log_v_vel_aruco_3_wave.txt");
%v_land = importdata("log_v_good_land.txt");
v_pitch = importdata("log_pitch.txt");
x = importdata("log_x.txt");
%x(:,1) = [];
%v(:,1) = [];
%x(1:10,:) = [];
%v(1,:) = [];

% figure(1)
% subplot(2,3,1)
% plot(x(:,1)-5)
% hold on;
% plot(x(:,3))
% title('altitude drone')
% subplot(2,3,2)
% plot(x(:,2))
% title('velocity drone')
% subplot(2,3,3)
% plot(x(:,3))
% title('altitude ship')
% subplot(2,3,4)
% plot(x(:,4))
% title('velocity ship')
% subplot(2,3,5)
% plot(x(:,5))
% title('acceleration ship')

figure(2)
%plot(v_vel(:,5), v_vel(:,1))

%subplot(2,1,1)
hold on;
plot(v_pitch(:,5), v_pitch(:,2))
%plot(v_vel(:,5), v_vel(:,3)-pi)
plot(v_pitch(:,5), -v_pitch(:,4)- v_pitch(:,7)-0.013)
%subplot(2,1,2)
%plot(v_pitch(:,5), v_pitch(:,8)/20)


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

% figure(7)
% plot(v_vel(:,4), v_vel(:,1)-1.12-(4-1.12))
% hold on
% plot(v_vel(:,4), v_vel(:,2))

% figure(8)
% plot(v_land(:,1))
% hold on;
% plot(v_land(:,2)-6)
% legend('drone pos', 'ship pos')

%%
avg_dist_pos = mean(abs(v_pos_dist(1:800,3)-5))
avg_dist_vel = mean(abs(v_vel_dist(1:800,3)-5))
avg_dist_vel_lp = mean(abs(v_vel_dist_lp(1:800,3)-5))
std_dist_pos = std(abs(v_pos_dist(1:800,3)-5))
std_dist_vel = std(abs(v_vel_dist(1:800,3)-5))
std_dist_vel_lp = std(abs(v_vel_dist_lp(1:800,3)-5))


