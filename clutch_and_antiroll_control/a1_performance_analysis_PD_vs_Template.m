%% Clutch and anti-roll control
% ----------------------------------------
% Created on: 15 Jan 2024
% By: Phat Do
% ----------------------------------------

%% Reset workspace
clc
clear
close

% System parameters
k = 0.8;
Jm = 4*1e-4;
Jl = 4*1e-4;
Bm = 0.015;
Bl = 0;

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

% % Sensitivity function
% L=G_tf*K_pid;
% 
% % sensitivity function
% S=inv(1+L);
% pole(S);
% T=feedback(L,1);
% pole(T);
% SG=series(G_tf,S);
% pole(SG);
% KG = G_tf*K_pid;
% KS = K_pid * S;

%% Define templates
% for tracking error (S)
Ms=2;wb=1;epsi=0.001;
We=tf([1/Ms wb],[1 wb*epsi]);

% for control input (KS)
Mu=3;wbc=75;epsi1=0.01;
Wu=tf([1 wbc/Mu],[epsi1 wbc]);
% Wu=ss(1/3);

%% stability analysis
% poleS=pole(S);
% poleT=pole(T);
% poleSG=pole(SG);
% poleKS=pole(KS);
% if (sum(poleS>0) & sum(poleT>0) & sum(poleSG>0) & sum(poleKS>0))
%     display('the closed-loop system is not stable')
% else
%     display('the closed-loop system is internally stable')
%  end


%%%%
w=logspace(-2,3,500);
[sv1,wout1] = sigma(1/We,w);
[sv2,wout2] = sigma(S_PD,w);
[sv3,wout3] = sigma(1/Wu,w);
[sv4,wout4] = sigma(KS_PD,w);

figure
% Tracking error sensitivity
subplot(121)
semilogx(wout1,20*log10(sv1),wout2,20*log10(sv2))
title('Tracking error sensitivity (S)','interpreter','latex')
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('$1/W_e$','PD control','interpreter','latex');
legend('Location','southeast');
grid on

% Controller sensitivity
subplot(122)
semilogx(wout3,20*log10(sv3),wout4,20*log10(sv4))
title('Controller sensitivity (KS)','interpreter','latex')
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('$1/W_u$','PD control','interpreter','latex');
legend('Location','southeast');
grid on

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'template_PD.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.3\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

