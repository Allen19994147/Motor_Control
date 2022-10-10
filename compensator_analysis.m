%%%analysis of compensator of control system II final project
clear all
clc
%%%%define plant
num = [6.945];
den = [1 14.29 0];
plant = tf(num,den);
%define closed loop TF
nu = [361.7 2894 5426];
de = [1 40.04 565.4 3200 5426];
cltf = tf(nu,de);
%Compute controller via CLTF and plant
%Assume the model as united feedback
controller = cltf/plant/(1-cltf);
%compute error function
err = 1/(1+controller*plant);
oltf = controller * plant;
figure(1)
step(cltf)
stepinfo(cltf)

figure(2)
step(err)
stepinfo(err)

figure(3)
step(controller)
stepinfo(controller)

