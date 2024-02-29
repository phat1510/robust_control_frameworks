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
% P1

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
% P1

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
P1


nmeas=1; ncon=1;
[K_inf,CL,gam,info] = hinfsyn(P1,nmeas,ncon,'DISPLAY','ON');

%% Hinf performance 
S=1/(1+G*K_inf);
T=1-S;
SG=S*G;
KS=K_inf*S;

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
title('S','interpreter','latex'), 
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


%% Uncertainty modeling

range = 0.1;
k_u = ureal('k',k,'Range',[k*(1-range),k*(1+range)]);
Jm_u = ureal('Jm',Jm,'Range',[Jm*(1-range),Jm*(1+range)]);
Jl_u = ureal('Jl',Jl,'Range',[Jl*(1-range),Jl*(1+range)]);
Bm_u = ureal('Bm',Bm,'Range',[Bm*(1-range),Bm*(1+range)]);
Bl_u = ureal('Bl',Bl,'Range',[-0.0001,0.0001]);
% Plant transfer fucntion
pl_u = Jl_u*s^2+Bl_u*s+k_u;
pm_u = Jm_u*s^2+Bm_u*s+k_u;
G_u = k_u/(pl_u*pm_u-k_u^2);

% figure
% [sv1, wout1] = sigma(G_u.NominalValue, usample(G_u,25));

figure
subplot(121)

order_weight=1;
[P,Info] = ucover(usample(G_u,25),G_u.NominalValue,order_weight,'InputMult')

[B,SAMPLES] = usample(G_u,25);

w=logspace(-3,3,500);

for i=1:length(B)
%     sigma((G_u.NominalValue -  B(:,:,i,1))/G_u.NominalValue, ...
%     'b--',Info.W1,'r',{0.001,1000}),
    [mag_out, freq_out] = sigma((G_u.NominalValue -  B(:,:,i,1))/G_u.NominalValue, w);
    semilogx(freq_out, 20*log10(mag_out),'blue','LineStyle','--');
    xlim([1e-3 1e3])
    hold on
end
[mag_out, freq_out] = sigma(Info.W1, w);
semilogx(freq_out, 20*log10(mag_out),'red');
title('$1^{st}$ order','interpreter','latex'),
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
grid on;
hold off

subplot(122)
% sigma(G_u.NominalValue,'r-+', usample(G_u,25),'b');grid on;

order_weight=3;

[P,Info] = ucover(usample(G_u,25),G_u.NominalValue,order_weight,'InputMult');

[B,SAMPLES] = usample(G_u,25);

w=logspace(-3,3,500);
for i=1:length(B)
%     sigma((G_u.NominalValue -  B(:,:,i,1))/G_u.NominalValue, ...
%     'b--',Info.W1,'r',{0.001,1000}),
    [mag_out, freq_out] = sigma((G_u.NominalValue -  B(:,:,i,1))/G_u.NominalValue, w);
    semilogx(freq_out, 20*log10(mag_out),'blue','LineStyle','--');
    xlim([1e-3 1e3])
    hold on
end


[mag_out, freq_out] = sigma(Info.W1, w);
semilogx(freq_out, 20*log10(mag_out),'red');
title('$3^{rd}$ order','interpreter','latex'),
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
grid on;
hold off

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'upper_bound_selection.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '\textwidth', ... image width
%     'height', '0.25\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------


figure
[sv1,wout1] = sigma(1/Info.W1,w);
[sv2,wout2] = sigma(T_PD,w);
[sv3,wout3] = sigma(T,w);
semilogx(wout1,20*log10(sv1),'red',wout2,20*log10(sv2),'blue',wout3,20*log10(sv3),'magenta')
% title('T','interpreter','latex'), 
% xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('Robustness template','PD control','$H_\infty$ control','interpreter','latex');
legend('Location','southwest');
ylim([-80 30]);
grid on

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'robust_template.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '0.5\textwidth', ... image width
%     'height', '0.3\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

figure

for i=1:length(B)
    [mag_out, freq_out] = sigma(B(:,:,i,1),w);
    semilogx(freq_out, 20*log10(mag_out),'b');
    xlim([1e-3 1e3])
    hold on
end
[mag_out, freq_out] = sigma(G_u.NominalValue,w);
p = semilogx(freq_out, 20*log10(mag_out),'--','Color','red');
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
grid on
hold off

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'modeling_error.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '0.5\textwidth', ... image width
%     'height', '0.3\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------
