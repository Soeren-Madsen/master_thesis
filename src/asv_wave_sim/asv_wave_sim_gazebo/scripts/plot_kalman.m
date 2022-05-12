clear; clc;
%v_pos_dist = importdata("log_v_pos_dist.txt");
%v_vel_dist = importdata("log_v_vel_dist.txt");
%v_vel_dist_lp = importdata("log_v_vel_dist_lp.txt");
%v_pos_dist = importdata("log_v_pos_dist_3_wave.txt");
%v_vel_dist = importdata("log_v_vel_dist_3_wave.txt");
%v_vel_dist_lp = importdata("log_v_vel_dist_lp_3_wave.txt");
%v_vel_aruco = importdata("log_v_vel_aruco_3_wave.txt");
%v_land = importdata("log_v_good_land.txt");
% v_pitch = importdata("log_pitch.txt");
% v_dist = importdata("log_dist.txt");
% v_roll = importdata("log_roll.txt");
% dist_test = importdata("dist_test_no_movement.txt");
xy_cor = importdata("log_v.txt")
x = importdata("log_x.txt");
%x(:,1) = [];
%v(:,1) = [];
%x(1:10,:) = [];
%v(1,:) = [];
%dist_test(1:50,:)=[]

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
% figure(1)
% hold on;
% plot(v_pitch(:,5), v_pitch(:,1)) %Roll gt
% plot(v_pitch(:,5), v_pitch(:,3)-pi) %Roll est
% 
% figure(2)
% hold on;
% 
% 
% plot(v_pitch(:,5), v_pitch(:,2)) %Pitch gt
% plot(v_pitch(:,5), -v_pitch(:,4)- v_pitch(:,7)-0.013) %Pitch est
% %plot(v_pitch(:,5), v_pitch(:,8)/20) %Ship alt
% ylabel({'Angle [Radians]'});
% xlabel({'Time [s]'});
% title({'Pitch estimation with ground truth'});

%FFT
% Y = fft(v_pitch(:,4));
% L = 752;
% Fs = 12
% P2 = abs(Y/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% figure (11)
% f = Fs*(0:(L/2))/L;
% plot(f,P1) 
% title('Single-Sided Amplitude Spectrum of X(t)')
% xlabel('f (Hz)')
% ylabel('|P1(f)|')


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

% figure(9)
% plot(v_dist(:,4), v_dist(:,2))
% hold on;
% plot(v_dist(:,4), v_dist(:,3))
% ylabel({'Distance [m]'});
% xlabel({'Time [s]'});
% title({'Measurement of distance between ship and drone using aruco marker'});
% 
% avg_error = mean(abs(v_dist(:,2)-v_dist(:,3)))
% std_error = std(v_dist(:,2)-v_dist(:,3))

% figure(10)
% hold on;
% plot(v_roll(:,4),v_roll(:,1)-0.3)
% plot(v_roll(:,4), v_roll(:,2)-pi) %Roll est
% 
% max_error = max(abs(v_roll(:,2)-pi+0.3))

% figure(11)
% hold on;
% plot(dist_test(:,3), dist_test(:,1)+0.17) %Distance aruco marker
% plot(dist_test(:,3), dist_test(:,2)) %Drones own position

figure (12)
hold on;
plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,1)) %X_cor from aruco
plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,2)) %y_cor from aruco
plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,4)) %x_pos drone
plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,5)) %y_pos drone
%%
avg_dist_pos = mean(abs(v_pos_dist(1:800,3)-5))
avg_dist_vel = mean(abs(v_vel_dist(1:800,3)-5))
avg_dist_vel_lp = mean(abs(v_vel_dist_lp(1:800,3)-5))
std_dist_pos = std(abs(v_pos_dist(1:800,3)-5))
std_dist_vel = std(abs(v_vel_dist(1:800,3)-5))
std_dist_vel_lp = std(abs(v_vel_dist_lp(1:800,3)-5))


