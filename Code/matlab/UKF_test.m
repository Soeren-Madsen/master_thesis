format bank;
format compact;

A1=10.;
A2=20.;
A3=30.;
freq1=2.;
freq2=1.;
freq3=4.;

dt=0.1;
t=0;
init_state=[100., 0., 0., A1-0.1, freq1-0.1, 0., A2-0.1, freq2-0.1, 0., A3-0.1, freq3-0.1];
ukf = unscentedKalmanFilter(@states,@measure,init_state);
 measure(init_state)
%%
for i=1:10
    t=dt*i;
    meas=100-(A1*sin(freq1*t)+A2*sin(freq2*t)+A3*sin(freq3*t));
    predict(ukf);
    correct(ukf,[meas,100.0]);
    state = ukf.State
    mes = ukf.State(1)-ukf.State(3)-ukf.State(6)-ukf.State(9)
    gt_mes = 100-(A1*sin(freq1*t)+A2*sin(freq2*t)+A3*sin(freq3*t))
    mes-gt_mes
    gt = [100, 0, A1*sin(freq1*t), A1*cos(freq1*t), freq1, A2*sin(freq2*t), A2*cos(freq2*t), freq2, A3*sin(freq3*t), A3*cos(freq3*t), freq3] 
end


