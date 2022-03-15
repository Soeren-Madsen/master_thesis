format bank;
format compact;
clear; clc;

A1=10.;
A2=20.;
A3=30.;
freq1=2.;
freq2=1.;
freq3=4.;

dt=0.1;
t=0;
init_state=[100., 0., 0., A1, freq1, 0., A2, freq2, 0., A3, freq3];
ukf = unscentedKalmanFilter(@states,@measure,init_state);
measure(init_state)
%%
out = zeros(1,11);
for i=1:100
    t=dt*i;
    meas=100-(A1*sin(freq1*t)+A2*sin(freq2*t)+A3*sin(freq3*t));
    predict(ukf);
    correct(ukf,[meas,100.0]);
    state = ukf.State
    out = [out; state];
    gt = [100, 0, A1*sin(freq1*t), A1*cos(freq1*t), freq1, A2*sin(freq2*t), A2*cos(freq2*t), freq2]%, A3*sin(freq3*t), A3*cos(freq3*t), freq3] 
    mes = ukf.State(1)-ukf.State(3)-ukf.State(6)-ukf.State(9)
    gt_mes = meas
    mes-gt_mes
end
%%
plot(out(:,5))
hold on;
plot(out(:,8))


