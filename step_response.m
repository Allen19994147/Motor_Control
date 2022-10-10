%%% Step response of controll system II project
%%% go with theta_control.slx file
clc
figure(1)
hold on
plot(out.out)
plot(out.out2)
legend('uncompensated','compensated')
title('Step response')
xlabel('time')
ylabel('magnitude')
axis([0 10 0 1.2])
hold off
figure(2)
plot(out.control_u)
legend('voltage clamper')
title('control effort')
xlabel('time')
ylabel('magnitude')
axis([0 10 -10 15])

%energy_comsumption = sum(out.control_u)
