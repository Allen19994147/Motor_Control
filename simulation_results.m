%%%Plot simulation results of theta_control.slx
clc

figure(1)
hold on
plot(out.input)
plot(out.out2)
plot(out.out)

legend("input","compensated","plant")
hold off
