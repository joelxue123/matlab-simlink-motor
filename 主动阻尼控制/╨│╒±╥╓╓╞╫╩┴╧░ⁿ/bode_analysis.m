%==================================================================================================
% 
%
%
%
%==================================================================================================
% a = 4095/4096;
% b = 4085/4096;
% Ts = 1/4096000;
% k1_k2 =  7.4*10^6*(a-b)/(1-a);
% num = k1_k2*Ts*Ts*[1 -a]
% den = conv([1 -1],conv([1 -1],[1 -b])); %
% sys = tf(num,den,Ts)
% g = feedback(sys,1)
% w = logspace(-1,5,100000);
% bode(g,w);

% [h,w]= freqz(sos,80000);
% 
% close all; 
% figure(1); 
% semilogx(w/pi,20*log10(abs(h)));



hold on
Jm = 0.015;
JL = 2.25;
Ks = 65;
kf = 0.1;
num_p = [JL,kf,Ks];
den_p1 = [Jm+JL,0];
den_p2 = [Jm*JL/(Jm+JL),kf,Ks];
num = num_p; 
den = conv(den_p1,den_p2); 

sys = tf(num_p,den)
sys_inv = tf(den,num_p)
sys_cmp = tf(den_p2,num_p)
% sys_c = tf(num_c,den_c,Ts);
% sys_o = tf(num_o,den_o,Ts);
% sys = tf(k*num,den,Ts);
% g = feedback(sys,1);
% w = logspace(2,4.85,50000); % up to 11KHz ; 4.85-5.6
% % 
bode(sys_inv);
bode(sys)
% step(sys)
% % grid minor
% margin(sys)
% grid minor
% legend('x','y')
