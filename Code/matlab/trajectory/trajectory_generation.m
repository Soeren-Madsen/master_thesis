clear; clc;
format compact;

%General parameters:
t0=0;
freq = 1;
A = 0.5;

%Drone parameters:
z_t0 = 10;
z_dot_t0 = 0;

%Platform parameters:
zd_hat = zd(t0,freq, A);
zd_dot_hat = zd_dot(t0,freq, A);

%Trajectory time calculation:
zrmax_ddot = 6;
zrmax_dot = 2;
za = z_t0 - zd_hat;
za_dot = z_dot_t0 - zd_dot(t0,freq, A);
za_tau = 1;
syms tau;

%calculation loop:
t = t0;
dt = 0.05;
z_ref = z_t0;
zd_vec = zd_hat;
za_ddot_plot = 0;
za_dot_plot = 0;
za_plot = 0;
t_plot = t;
while t < 10
    eqn = za_dot + zrmax_ddot*(tau-t)-zd_dot(tau,freq,A)+zd_dot(t,freq,A) == 0;
    tf = vpasolve(eqn);
    za_tau = za + za_dot*(tf-t)+1/2*zrmax_ddot*(tf-t)^2-zd(tf,freq,A)+zd(t,freq,A)+zd_dot(t,freq,A)*(tf-t);
    za_dot_tau = za_dot+ zrmax_ddot*(tf-t)-zd_dot(tf,freq,A)+zd_dot(t,freq,A);
    
    if za_tau <= 0 %Break
        za_ddot =  zrmax_ddot - zd_ddot(t,freq,A);
    elseif za_tau > 0
        if za_dot <= -zrmax_dot - zd_dot(t,freq,A)
            za_ddot = -zd_ddot(t,freq,A);
        else
            za_ddot =  -zrmax_ddot - zd_ddot(t,freq,A);
        end
    end
    t=t+dt;
    za_dot = za_dot+ za_ddot*dt;
    za=za + za_dot*dt+1/2*za_ddot*dt^2;
    
    zd_vec = [zd_vec, zd(t,freq,A)];
    zr = za-zd(t,freq,A);
    z_ref = [z_ref, za+zd(t,freq,A)];
    za_ddot_plot = [za_ddot_plot, za_ddot];
    za_dot_plot = [za_dot_plot, za_dot];
    za_plot = [za_plot, za];
    t_plot = [t_plot, t];
end
%figure(1)
%plot(za_ddot_plot)
%figure(2)
%plot(za_dot_plot)
figure(3)
%plot(za_plot)
%figure(3)
hold on;
plot(t_plot, zd_vec);
plot(t_plot, z_ref);



