%%%check robustness for my controller
clc

%my controller;
num = [52 1161 6736 11165];
den = [1 40.04 203.75 306];
%roots(den)
controller = tf(num,den);
%nyquist(controller);
global aa bb
for i = 1:30
    aa = 6.945 + 6.945*(rand()-0.5)*2*0.4
    bb = 14.29 + 14.29*(rand()-0.5)*2*0.4
    num = [aa];
    den = [1 bb 0];
    plant = tf(num,den);
    oltf = controller*plant;
    u = controller/(1+oltf);
    cltf = oltf/(1+oltf);
%     step(cltf);
%     stepinfo(cltf)
    % figure(2)
    step(u);
    stepinfo(u)
    pause(1)
    clf
    
    
end
    


