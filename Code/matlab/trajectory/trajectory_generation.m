clear; clc;
format compact;

%INDSÆT SÅ DRONEN STOPPER NÅR DEN RAMMER BÅDEN
landed = 0

%General parameters:
t0=0;
freq = 1;
A = 0.5;
freq2 = 2;
A2 = 0.7;
freq3 = 0.7;
A3 = 1.5;

%Drone parameters:
z_t0 = 6;
z_dot_t0 = 0;

%Platform parameters:
zd_hat = zd(t0,freq, A) + zd(t0,freq2, A2) + zd(t0,freq3, A3);
zd_dot_hat = zd_dot(t0,freq, A) + zd_dot(t0,freq2, A2) + zd_dot(t0,freq3, A3);

%Trajectory time calculation:
zrmax_ddot = 2;
zrmax_dot = 2;
za = z_t0 - zd_hat;
za_dot = z_dot_t0 - (zd_dot_hat);
za_tau = 1;
syms tau;

%calculation loop:
t = t0;
dt = 0.05;
z_ref = z_t0;
zd_ref = z_dot_t0
zd_vec = zd_hat;
za_ddot_plot = 0;
za_dot_plot = 0;
za_plot = 0;
t_plot = t;
while t < 5
    if landed == 0
        zd_d_tau = zd_dot(tau,freq,A) + zd_dot(tau,freq2,A2) + zd_dot(tau,freq3,A3);
        zd_d_t = zd_dot(t,freq,A) + zd_dot(t,freq2,A2) + zd_dot(t,freq3,A3);
        eqn = za_dot + zrmax_ddot*(tau-t)-zd_d_tau+zd_d_t == 0;
        tf = vpasolve(eqn);
        zd_tf = zd(tf,freq,A) + zd(tf,freq2,A2) + zd(tf,freq3,A3);
        zd_t = zd(t,freq,A) + zd(t,freq2,A2) + zd(t,freq3,A3);
        zd_d_tf = zd_dot(tf,freq,A) + zd_dot(tf,freq2,A2) + zd_dot(tf,freq3,A3);
        
        za_tau = za + za_dot*(tf-t)+1/2*zrmax_ddot*(tf-t)^2-zd_tf+zd_t+zd_d_t*(tf-t);
        za_dot_tau = za_dot+ zrmax_ddot*(tf-t)-zd_d_tf+zd_d_t;
        
        zd_dd = zd_ddot(t,freq,A) + zd_ddot(t,freq2,A2) + zd_ddot(t,freq3,A3);
        
        if za_tau <= 0 %Break
            za_ddot =  zrmax_ddot - zd_dd;
        elseif za_tau > 0
            if za_dot <= -zrmax_dot - zd_d_t
                za_ddot = -zd_dd;
            else
                za_ddot =  -zrmax_ddot - zd_dd;
            end
        end
        t=t+dt
        za_dot = za_dot+ za_ddot*dt;
        za=za + za_dot*dt+1/2*za_ddot*dt^2;
        if za<= 0
            landed = 1
        end
        zd_vec = [zd_vec, zd_t];
        zr = za+zd_t;
        z_ref = [z_ref, za+zd_t];
        zd_ref = [zd_ref, za_dot+zd_d_t];
        za_ddot_plot = [za_ddot_plot, za_ddot];
        za_dot_plot = [za_dot_plot, za_dot];
        za_plot = [za_plot, za];
        t_plot = [t_plot, t];
    else
        t=t+dt
        zd_vec = [zd_vec, zd(t,freq,A) + zd(t,freq2,A2) + zd(t,freq3,A3)];
        z_ref = [z_ref, zd(t,freq,A) + zd(t,freq2,A2) + zd(t,freq3,A3)];
        t_plot = [t_plot, t];
        zd_ref = [zd_ref, zd_dot(t,freq,A) + zd_dot(t,freq2,A2) + zd_dot(t,freq3,A3)]
    end
end
%figure(1)
%plot(za_ddot_plot)
%figure(2)
%plot(za_dot_plot)
%figure(3)
%plot(za_plot)
figure(3)
hold on;
plot(t_plot, zd_vec);
plot(t_plot, z_ref);
%plot(t_plot,zd_ref)



