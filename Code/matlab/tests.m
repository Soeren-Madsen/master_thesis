x3_0_init = sin(0)
x4_0_init = cos(0)
x5_0_init = 2

x3_0 = sin(2*0.1)
x3_0_hat = x3_0_init*cos(x5_0_init*0.1)+ x4_0_init*sin(x5_0_init*0.1)
x4_0 = cos(2*0.1)
x4_0_hat = -sin(x5_0_init*0.1)*x3_0_init+cos(x5_0_init*0.1)*x4_0_init
x3_1 = sin(2*0.2)
x3_1_hat = x3_0_hat*cos(x5_0_init*0.1)+ x4_0_hat*sin(x5_0_init*0.1)
x4_1 = cos(2*0.2)
x4_1_hat = -sin(x5_0_init*0.1)*x3_0_hat+cos(x5_0_init*0.1)*x4_0_hat