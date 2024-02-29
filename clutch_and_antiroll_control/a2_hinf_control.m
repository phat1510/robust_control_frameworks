%% Clutch and anti-roll control
% ----------------------------------------
% Created on: 15 Jan 2024
% By: Phat Do
% ----------------------------------------

%% Reset workspace
clc
clear
close all

% System parameters
k = 0.8;
Jm = 4*1e-4;
Jl = 4*1e-4;
Bm = 0.015;
Bl = 0;

noise_fre = 800;

% Plant transfer fucntion
s = tf('s');
pl = Jl*s^2+Bl*s+k;
pm = Jm*s^2+Bm*s+k;
G = k/(pl*pm-k^2);

%% PI frequency analysis
% PID controller
Kp = 0.3;
Kd = 0.0033;
K_pd = Kd*s+Kp;

TF_cl = G*K_pd/(1+G*K_pd);
S_PD = 1/(1+G*K_pd);
T_PD = 1 - S_PD;
KS_PD = K_pd*S_PD;
SG_PD = S_PD*G;


%% Define templates
% For tracking error (S)
Ms=2;wb=7;epsi=0.0001;
We=tf([1/Ms wb],[1 wb*epsi]);

% For control input (KS)
Mu=3;wbc=75;epsi1=0.01;
Wu=tf([1 wbc/Mu],[epsi1 wbc]);

% For input disturbance
Wd=ss(0.15);

% Noise filter
Wn=(3*s+150)/(s+2000);

% Test 01
% [A,B,C,D]=linmod('We_only');
% P1=ss(A,B,C,D)

% Test 01
% systemnames = 'G We Wd';
% inputvar = '[r(1); d; n; u(1)]';
% outputvar = '[We; r-G-n]';
% input_to_G = '[u+Wd]';
% input_to_We = '[r-G-n]';
% % input_to_Wu = '[u]';
% input_to_Wd= '[d]';
% sysoutname = 'P1';
% cleanupsysic = 'yes';
% sysic;
% P1;



% Test 02
% systemnames = 'G We Wu Wd';
% inputvar = '[r(1); d; n; u(1)]';
% outputvar = '[We; Wu; r-G-n]';
% input_to_G = '[u+Wd]';
% input_to_We = '[r-G-n]';
% input_to_Wu = '[u]';
% input_to_Wd= '[d]';
% sysoutname = 'P1';
% cleanupsysic = 'yes';
% sysic;
% P1;

% Test 03
systemnames = 'G We Wu Wd Wn';
inputvar = '[r(1); d; n; u(1)]';
outputvar = '[We; Wu; r-G-Wn]';
input_to_G = '[u+Wd]';
input_to_We = '[r-G-Wn]';
input_to_Wu = '[u]';
input_to_Wd= '[d]';
input_to_Wn = '[n]';
sysoutname = 'P1';
cleanupsysic = 'yes';
sysic;
P1;


nmeas=1; ncon=1;
[K_inf,CL,gam,info] = hinfsyn(P1,nmeas,ncon,'DISPLAY','ON');

%% Hinf performance 
S=1/(1+G*K_inf);
T=1-S;
SG=S*G;
KS=K_inf*S;

S_HinfNorm = hinfnorm(S)
T_HinfNorm = hinfnorm(T)

% Data extraction
w=logspace(-2,3,500);
[sv1,wout1] = sigma(1/We,w);
[sv2,wout2] = sigma(1/Wu,w);
[sv3,wout3] = sigma(S_PD,w);
[sv4,wout4] = sigma(T_PD,w);
[sv5,wout5] = sigma(SG_PD,w);
[sv6,wout6] = sigma(KS_PD,w);
[sv7,wout7] = sigma(S,w);
[sv8,wout8] = sigma(T,w);
[sv9,wout9] = sigma(SG,w);
[sv10,wout10] = sigma(KS,w);

figure
% Output sensitivity
subplot(221), semilogx(wout1,20*log10(sv1),'red',wout3,20*log10(sv3),'blue',wout7,20*log10(sv7),'magenta')
title('S','interpreter','latex'), 
% xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('1/We','PD control','$H_\infty$ control','interpreter','latex');
legend('Location','southeast');
ylim([-120 10]);
grid on

% Complementary sensitivity
subplot(222), semilogx(wout4,20*log10(sv4),'blue',wout8,20*log10(sv8),'magenta')
title('T','interpreter','latex'), 
% xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('PD control','$H_\infty$ control','interpreter','latex');
legend('Location','southwest');
ylim([-100 10]);
grid on

% Closed-loop sensitivity
subplot(223), semilogx(wout5,20*log10(sv5),'blue',wout9,20*log10(sv9),'magenta')
title('SG','interpreter','latex'), 
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('PD control','$H_\infty$ control','interpreter','latex');
legend('Location','southwest');
ylim([-100 20]);
grid on

% Controller sensitivity
subplot(224), semilogx(wout2,20*log10(sv2),'red',wout6,20*log10(sv6),'blue',wout10,20*log10(sv10),'magenta')
title('KS','interpreter','latex'), 
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('1/Wu','PD control','$H_\infty$ control','interpreter','latex');
legend('Location','southwest');
ylim([-80 30]);
grid on

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'Hinf_case1.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.45\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'Hinf_case2.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.45\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'Hinf_case3.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.45\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

%% Simulation

% sim("a2_clutch_control_sim.slx");
% 
% simtime = tout;
% output_hinf     = clutch_sim.Data(:,1);
% ref             = clutch_sim.Data(:,2);
% output_pd       = clutch_sim.Data(:,3);
% kinf            = clutch_sim.Data(:,4);
% kpd             = clutch_sim.Data(:,5);
% 
% figure
% subplot(121)
% plot(simtime, ref, 'black', simtime, output_pd, 'blue', simtime, output_hinf,'magenta'); hold on
% % plot([0 60],[1.05 1.05],'red');
% % plot([0 60],[(1-0.05) (1-0.05)],'red');
% xlabel('Time (s)','interpreter','latex')
% ylabel('Controlled output','interpreter','latex')
% xlim([0 5])
% % legend('Ref','PD control','$H_\infty$ control','5\% range','','interpreter','latex');
% legend('Ref','PD control','$H_\infty$ control','interpreter','latex');
% legend('Location','southeast');
% grid on
% hold off
% 
% 
% subplot(122)
% plot(simtime, kpd, 'blue', simtime, kinf,'magenta'); hold on
% xlabel('Time (s)','interpreter','latex')
% ylabel('Control input','interpreter','latex')
% legend('PD control','$H_\infty$ control','interpreter','latex');
% xlim([0 5])
% grid on


% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'clutch_sim_no_noise.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.3\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'clutch_sim_noise_800_no_filter.tex';
% cleanfigure('targetResolution', 10);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.3\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'clutch_sim_noise_800_filter.tex';
% cleanfigure('targetResolution', 50);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.3\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% % -----------------------------


