clc
clear all
%______plant___________
A = [-14.29 0;1 0];
B = [1;0];
C = [0 6.945];
[num den] = ss2tf(A,B,C,[0]);
plant = tf(num,den);


%__________K_____________________
%p = 5.5;
p = 3.4;
Q = p*C'*C;
%Q = diag([1 1 5]);
R = [0.1];
K = lqr(A,B,Q,R);
pc = eig(A-B*K)';

%__________L_________________________
pe = 4*max(pc);
pe = [pe,pe+0.0001];
L = place(A',C',pe)';
rank([C;C*A]);

%_______compensated_________________
ac = poly(A-B*K);
ae = poly(A-L*C);
compensator_num = [-5 -3];
M_bar = -1*place((A-B*K-L*C)',K',compensator_num)';
%__________simulation result_________
num = conv(poly(A-B*K-L*C+M_bar*K),num);
den = conv(ac,ae);
N_bar = den(end)/num(end);
r2y = tf(num,den)*N_bar;
M = N_bar.*M_bar;
figure(1)
step(r2y)
stepinfo(r2y)
figure(2)
bode(r2y)
margin(r2y)
[Gm,Pm,Wcg,Wcp] = margin(r2y);
BW_compensated = bandwidth(r2y);
%__________r2u_______not sure to be accurate__
[num den] = ss2tf(A-B*K-L*C,M,-K,[N_bar]);
r2u = tf(num,den)
figure(3)
step(r2u)
stepinfo(r2u)


%%%________C2D___________________________
% Ts = 0.01;
% figure (4)
% hold on
% compensated_digital = c2d(r2y,Ts,"tustin")
% step(compensated_digital)
%compensated_digital = c2d(r2y,Ts,"zoh");
%step(compensated_digital)
%compensated_digital = c2d(r2y,Ts,"matched");
%step(compensated_digital)
%legend('continuous','tustin','zoh','matched')
% hold off

