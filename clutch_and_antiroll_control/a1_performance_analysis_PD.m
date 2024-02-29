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
Ms=1.5;wb=1;epsi=0.001;
We=tf([1/Ms wb],[1 wb*epsi]);

% for control input (KS)
Mu=3;wbc=75;epsi1=0.01;
Wu=tf([1 wbc/Mu],[epsi1 wbc]);

%% stability analysis
poleS=pole(S_PD);
poleT=pole(T_PD);
poleSG=pole(SG_PD);
poleKS=pole(KS_PD);
if (sum(poleS>0) & sum(poleT>0) & sum(poleSG>0) & sum(poleKS>0))
    display('the closed-loop system is not stable')
else
    display('the closed-loop system is internally stable')
 end


%%%%
w=logspace(-2,3,500);
[sv1,wout1] = sigma(S_PD,w);
[sv2,wout2] = sigma(T_PD,w);
[sv3,wout3] = sigma(SG_PD,w);
[sv4,wout4] = sigma(KS_PD,w);

figure
% Tracking error sensitivity
subplot(221)
semilogx(wout1,20*log10(sv1))
title('Tracking error sensitivity (S)','interpreter','latex')
% xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
grid on

% Complementary sensitivity
subplot(222)
semilogx(wout2,20*log10(sv2))
title('Complementary sensitivity (T)','interpreter','latex')
% xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
ylim([-80 20])
grid on

% Plant sensitivity
subplot(223)
semilogx(wout3,20*log10(sv3))
title('Plant sensitivity (SG)','interpreter','latex')
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
ylim([-80 20])
grid on

% Controller sensitivity
subplot(224)
semilogx(wout4,20*log10(sv4))
title('Controller sensitivity (KS)','interpreter','latex')
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
grid on

% -----------------------------
fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
fig1_name = 'PD_only.tex';
cleanfigure('targetResolution', 50);
matlab2tikz(append(fig_loc, fig1_name), ... filename
    'width', '\textwidth', ... image width
    'height', '0.45\textwidth', ... image height
     'showInfo', false);  % ... turn off information
% -----------------------------

