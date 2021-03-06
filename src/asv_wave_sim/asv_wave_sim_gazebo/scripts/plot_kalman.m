clear; clc; format compact;
%v_pos_dist = importdata("log_v_pos_dist.txt");
%v_vel_dist = importdata("log_v_vel_dist.txt");
%v_vel_dist_lp = importdata("log_v_vel_dist_lp.txt");
% v_pos_dist = importdata("log_v_pos_dist_3_wave.txt");
%v_vel_dist = importdata("log_v_vel_dist_3_wave.txt");
%v_vel_dist_lp = importdata("log_v_vel_dist_lp_3_wave.txt");
% v_vel_aruco = importdata("log_v_vel_aruco_3_wave.txt");
% v_land = importdata("log_v_very_good_landing_5_predict.txt");
v_pitch = importdata("log_pitch.txt");
% v_dist = importdata("log_dist.txt");
% v_roll = importdata("log_roll.txt");
% dist_test = importdata("dist_test_no_movement.txt");
% xy_cor = importdata("xy_cor_pic.txt");
% dist_motion = importdata("dist_motion_delay.txt");
% dist_motion = importdata("dist_motion_no_delay.txt");
% ori_motion = importdata("ori_motion.txt");
% combi = importdata("combi.txt");
% kalman_no_movement = importdata("kalman_no_movement.txt");
% kalman_no_movement_gt = importdata("kalman_no_movement_gt.txt");
% kalman_no_movement_predict = importdata("kalman_no_movement_predict.txt")
% kalman_no_movement_predict_gt = importdata("kalman_no_movement_predict_gt.txt")
% kalman_movement = importdata("kalman_movement.txt");
% kalman_movement_gt = importdata("kalman_movement_gt.txt");
% kalman_movement_6_predict = importdata("kalman_movement_6_predict.txt");
% kalman_movement_6_predict_gt = importdata("kalman_movement_6_predict_gt.txt");
% kalman_movement_6_predict = importdata("kalman_movement_7_predict.txt");
% kalman_movement_6_predict_gt = importdata("kalman_movement_7_predict_gt.txt");
% kalman_movement_6_predict = importdata("kalman_movement_8_predict.txt");
% kalman_movement_6_predict_gt = importdata("kalman_movement_8_predict_gt.txt");
% kalman_movement_6_predict = importdata("new_kalman_states.txt");
% kalman_movement_6_predict_gt = importdata("new_kalman_gt.txt");
% kalman_sim_vel_gt = importdata("kalman_sim_gt.txt")
% kalman_sim_vel = importdata("kalman_sim.txt")
% sim_follow = importdata("sim_follow.txt");
%sim_predict = importdata("sim_predict.txt")
% sim_predict = importdata("sim_predict_8.txt") %8 predict
% sim_landing = importdata("sim_landing.txt")
% % 
% kalman_no_movement(1:30,:) = [];
% kalman_no_movement(430:912,:) = [];
% kalman_no_movement_gt(1:30,:) = [];
% kalman_no_movement_gt(430:908,:) = [];
% kalman_no_movement_predict(1:30,:) = []
% kalman_no_movement_predict(700:1448,:) = []
% kalman_no_movement_predict_gt(1:30,:) = []
% kalman_no_movement_predict_gt(700:1380,:) = []
% kalman_movement(1:30,:) = [];
% kalman_movement(1000:1591,:) = [];
% kalman_movement_gt(1:30,:) = [];
% kalman_movement_gt(1000:1596,:) = [];
% kalman_movement_6_predict(1:30,:) = [];
% kalman_movement_6_predict(1000:1532,:) = [];
% kalman_movement_6_predict_gt(1:30,:) = [];
% kalman_movement_6_predict_gt(1000:1592,:) = [];

% kalman_movement_6_predict(1:30,:) = [];
% kalman_movement_6_predict(1000:1401,:) = [];
% kalman_movement_6_predict_gt(1:30,:) = [];
% kalman_movement_6_predict_gt(1000:1377,:) = [];

% kalman_movement_6_predict(1:30,:) = [];
% kalman_movement_6_predict(500:921,:) = [];
% kalman_movement_6_predict_gt(1:30,:) = [];
% kalman_movement_6_predict_gt(500:833,:) = [];

% kalman_movement_6_predict(1:30,:) = [];
% kalman_movement_6_predict(700:1136,:) = [];
% kalman_movement_6_predict_gt(1:30,:) = [];
% kalman_movement_6_predict_gt(700:1161,:) = [];

%v(1,:) = [];
% dist_test(1:50,:)=[]
% ori_motion(800:1300,:)=[]
% combi(1000:1408,:)=[]
% kalman_sim_gt(844:862,:) = []
% kalman_sim_vel(647:703,:) = []
%sim_predict(600:1195,:) = []
%sim_predict(1:10,:) = []


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
figure(2)
hold on;


plot(v_pitch(:,5), v_pitch(:,2)) %Pitch gt
plot(v_pitch(:,5), -v_pitch(:,4)- v_pitch(:,7)-0.013) %Pitch est
%plot(v_pitch(:,5), v_pitch(:,8)/20) %Ship alt
ylabel({'Angle [Radians]'});
xlabel({'Time [s]'});
title({'Pitch estimation with ground truth'});

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
% 
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
% plot(v_land(:,2))
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
% plot(dist_test(:,3), dist_test(:,1)+0.21) %Distance aruco marker
% plot(dist_test(:,3), dist_test(:,2)) %Drones own position
% 
% figure (12)
% hold on;
% plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,1)) %X_cor from aruco
% plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,2)) %y_cor from aruco
% plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,4)) %x_pos drone
% plot(xy_cor(:,3)-xy_cor(1,3), xy_cor(:,5)) %y_pos drone

% figure(13)
% hold on;
% plot(dist_motion(:,3)-dist_motion(1,3), dist_motion(:,1)+0.21) %Distance aruco marker
% plot(dist_motion(:,3)-dist_motion(1,3), dist_motion(:,2)-dist_motion(:,4)) %Ground truth

% for i=1:length(ori_motion)
%     if ori_motion(i,1) < 0
%         ori_motion(i,1)=ori_motion(i,1)+2*pi;
%     end
% end
% 
% figure(14)
% hold on;
% plot(ori_motion(:,3)-ori_motion(1,3), ori_motion(:,2)) %pitch est
% plot(ori_motion(:,3)-ori_motion(1,3), ori_motion(:,1)-3.1) %Roll est
% plot(ori_motion(:,3)-ori_motion(1,3), ori_motion(:,5)) %Roll gt
% plot(ori_motion(:,3)-ori_motion(1,3), ori_motion(:,4)) %pitch gt
% 
% for i=1:length(combi)
%     if combi(i,1) < 0
%         combi(i,1)=combi(i,1)+2*pi;
%     end
% end
% 
% figure(14)
% hold on;
% % plot(combi(:,3)-combi(1,3), combi(:,2)) %pitch est
% plot(combi(:,3)-combi(1,3), combi(:,1)-3.1) %Roll est
% plot(combi(:,3)-combi(1,3), combi(:,5)) %Roll gt
% % plot(combi(:,3)-combi(1,3), combi(:,4)) %pitch gt

% figure(15)
% hold on;
% plot(combi(:,3)-combi(1,3), combi(:,6))
% plot(combi(:,3)-combi(1,3), combi(:,7)-combi(:,8)-0.21)
% 
% error = abs((combi(:,7)-combi(:,8)-0.21)-combi(:,6))
% mean(error)
% max(error)
% std(error)

% figure(16)
% %subplot(2,3,1)
% %plot(kalman_no_movement(:,3)-kalman_no_movemet(1,3), kalman_no_movement(:,1))
% hold on;
% % plot(kalman_no_movement(:,3)-kalman_no_movement(1,3), kalman_no_movement(:,1))
% % title('altitude drone')
% % subplot(2,3,2)
% % plot(kalman_no_movement(:,3)-kalman_no_movement(1,3), kalman_no_movement(:,2))
% % title('velocity drone')
% % subplot(2,3,3)
% % hold on;
% plot(kalman_no_movement(:,3)-kalman_no_movement(1,3), kalman_no_movement(:,4)-0.21)
% plot(kalman_no_movement_gt(:,3)-kalman_no_movement_gt(1,3), kalman_no_movement_gt(:,4))
% title('altitude ship')
% % subplot(2,3,4)
% % plot(kalman_no_movement(:,3)-kalman_no_movement(1,3), kalman_no_movement(:,5))
% % title('velocity ship')
% % subplot(2,3,5)
% % plot(kalman_no_movement(:,3)-kalman_no_movement(1,3), kalman_no_movement(:,6))
% % title('acceleration ship')


% figure(17)
% subplot(2,3,1)
% hold on;
% plot(kalman_no_movement_predict(:,3)-kalman_no_movement_predict(1,3), kalman_no_movement_predict(:,1))
% title('altitude drone')
% subplot(2,3,2)
% plot(kalman_no_movement_predict(:,3)-kalman_no_movement_predict(1,3), kalman_no_movement_predict(:,2))
% title('velocity drone')
% subplot(2,3,3)
% hold on;
% plot(kalman_no_movement_predict(:,3)-kalman_no_movement_predict(1,3), kalman_no_movement_predict(:,4)-0.21)
% plot(kalman_no_movement_predict_gt(:,3)-kalman_no_movement_predict_gt(1,3), kalman_no_movement_predict_gt(:,4))
% title('altitude ship')
% subplot(2,3,4)
% plot(kalman_no_movement_predict(:,3)-kalman_no_movement_predict(1,3), kalman_no_movement_predict(:,5))
% title('velocity ship')
% subplot(2,3,5)
% plot(kalman_no_movement_predict(:,3)-kalman_no_movement_predict(1,3), kalman_no_movement_predict(:,6))
% title('acceleration ship')

% figure(18)
% % subplot(2,1,1)
% hold on;
% plot(kalman_movement(:,3)-kalman_movement(1,3), kalman_movement_gt(:,2)-1.21)
% plot(kalman_movement_gt(:,3)-kalman_movement_gt(1,3), kalman_movement_gt(:,4))
% title('Altitude drone and motion table with correction')
% ylabel({'Height [m]'});
% xlabel({'Time [s]'});
% % subplot(2,1,2)
% % plot(kalman_movement(:,3)-kalman_movement(1,3), kalman_movement(:,2))
% % title('velocity drone')
% subplot(2,1,2)
% hold on;
%plot(kalman_movement(:,3)-kalman_movement(1,3), kalman_movement(:,4))
%plot(kalman_movement_gt(:,3)-kalman_movement_gt(1,3), kalman_movement_gt(:,4)+0.21)
% title('Altitude ship and kalman estimate')
%ylabel({'Height [m]'});
%xlabel({'Time [s]'});
% subplot(2,1,4)
% plot(kalman_movement(:,3)-kalman_movement(1,3), kalman_movement(:,5))
% title('velocity ship')
% subplot(2,1,5)
% plot(kalman_movement(:,3)-kalman_movement(1,3), kalman_movement(:,6))
% title('acceleration ship')

figure(19)
% % subplot(2,1,1)
hold on;
plot(kalman_movement_6_predict(:,3)-kalman_movement_6_predict(1,3), kalman_movement_6_predict_gt(:,2))
plot(kalman_movement_6_predict_gt(:,3)-kalman_movement_6_predict_gt(1,3), kalman_movement_6_predict_gt(:,4)+1+0.21)
title('Altitude drone and motion table with correction')
ylabel({'Height [m]'});
xlabel({'Time [s]'});
% %subplot(2,3,2)
% %plot(kalman_movement_6_predict(:,3)-kalman_movement_6_predict(1,3), kalman_movement_6_predict(:,2))
% %title('velocity drone')
% subplot(2,1,2)
% hold on;
% plot(kalman_movement_6_predict(:,3)-kalman_movement_6_predict(1,3), kalman_movement_6_predict(:,4))
% plot(kalman_movement_6_predict_gt(:,3)-kalman_movement_6_predict_gt(1,3), kalman_movement_6_predict_gt(:,4) + 0.21)
% title('Altitude ship and kalman estimate')
% ylabel({'Height [m]'});
% xlabel({'Time [s]'});
% 
% subplot(2,3,4)
% plot(kalman_movement_6_predict(:,3)-kalman_movement_6_predict(1,3), kalman_movement_6_predict(:,5))
% title('velocity ship')
% subplot(2,3,5)
% plot(kalman_movement_6_predict(:,3)-kalman_movement_6_predict(1,3), kalman_movement_6_predict(:,6))
% title('acceleration ship')

% figure(20)
% %subplot(3,1,1)
% hold on;
% plot(kalman_sim(:,3)-kalman_sim(1,3), kalman_sim_gt(:,2))
% plot(kalman_sim_gt(:,3)-kalman_sim_gt(1,3), kalman_sim_gt(:,4)-0.8)
% title('Altitude drone and altitude ship')
% ylabel({'Height [m]'});
% xlabel({'Time [s]'});
% subplot(3,1,3)
% plot(kalman_sim(:,3)-kalman_sim(1,3), kalman_sim(:,2))
% title('velocity drone')
% xlabel({'Time [s]'});
% ylabel({'Velocity [m/s]'});
% subplot(3,1,2)
% hold on;
% plot(kalman_sim(:,3)-kalman_sim(1,3), kalman_sim(:,4)+6.7)
% plot(kalman_sim_gt(:,3)-kalman_sim_gt(1,3), kalman_sim_gt(:,4) )
% title('Altitude ship and kalman estimate')
% ylabel({'Height [m]'});
% xlabel({'Time [s]'});

% subplot(2,3,4)
% plot(kalman_movement_6_predict(:,3)-kalman_movement_6_predict(1,3), kalman_movement_6_predict(:,5))
% title('velocity ship')
% subplot(2,3,5)
% plot(kalman_movement_6_predict(:,3)-kalman_movement_6_predict(1,3), kalman_movement_6_predict(:,6))
% title('acceleration ship')

% error = abs(kalman_sim_gt(:,2)-(kalman_sim_gt(:,4)-0.8));
% mu = mean(error)
% max_pos = max(error)
% std_pos = std(error)
% error_vel = abs(kalman_sim_vel_gt(:,2)-(kalman_sim_vel_gt(:,4)-0.8));
% mu_vel = mean(error_vel)
% max_vel = max(error_vel)
% std_vel = std(error_vel)

% figure(21)
% hold on;
% plot(sim_follow(:,3)-sim_follow(1,3), sim_follow(:,1)-5)
% plot(sim_follow(:,3)-sim_follow(1,3), sim_follow(:,2))
% title('Altitude drone and altitude ship')
% ylabel({'Height [m]'});
% xlabel({'Time [s]'});

% figure(22)
% hold on;
% %plot(sim_predict(:,3)-sim_predict(1,3), sim_predict(:,1)-5)
% plot(sim_predict(:,3)-sim_predict(1,3), sim_predict(:,4)+7.2)
% plot(sim_predict(:,3)-sim_predict(1,3), sim_predict(:,2))
% title('Drone following ship using predictive Kalman')
% ylabel({'Height [m]'});
% xlabel({'Time [s]'});

% figure(23)
% hold on;
% plot(sim_landing(:,3)-sim_landing(1,3), sim_landing(:,1)-1.1)
% %plot(sim_predict(:,3)-sim_predict(1,3), sim_predict(:,4)+7.2)
% plot(sim_landing(:,3)-sim_landing(1,3), sim_landing(:,2))
% title('Drone following ship using predictive Kalman')
% ylabel({'Height [m]'});
% xlabel({'Time [s]'});

