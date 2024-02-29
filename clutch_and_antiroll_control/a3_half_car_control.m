clear; close all; clc;
disp('---------------------------------------------------');
disp('--- Half Vehicle LTI Hinf                          ');
disp('--- BE                                             ');
disp('---------------------------------------------------');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Specification of system parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Megane params
meganeParametersHalf
actuator = 1000;
qhalf = halfLinearModel(ms,mus_fl,mus_fr,k_fl,k_fr,c_fl,c_fr,kt_fl,kt_fr,kb,t_f,Ix,actuator);

%Wn1  = .001;
%Wn2  = Wn1;

Mu=1;wbc=5;epsi1=0.01;
Wu=tf([1 wbc/Mu],[epsi1 wbc]);

Wzr1 = 0.1;     % to be chosen
Wzr2 = Wzr1;

kzs =  3;% to be chosen
wzs =   10;% to be chosen
Wzs = kzs*tf(1,[1/(2*pi*wzs) 1]);
% Wzs = Wu;

krol=  2.5;% to be chosen
wrol=   15;% to be chosen
Wrol = krol*tf(1,[1/(2*pi*wrol) 1]);

Mu=1;wbc=5;epsi1=0.01;
Wu=tf([1 wbc/Mu],[epsi1 wbc]);
% Wrol = Wu;

Mu=0.1;wbc=100;epsi1=0.01;
Wu=tf([1 wbc/Mu],[epsi1 wbc]);

Wu1  = 0.1;% to be chosen
Wu2  = Wu1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% LTI / Hinf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
systemnames    = 'qhalf Wzr1 Wzr2 Wu1 Wu2 Wzs Wrol';
inputvar       = '[zr1; zr2; Fdz; Mdx; u1; u2]';
outputvar      = '[Wzs; Wrol; Wu1; Wu2; qhalf(12); qhalf(13)]';
input_to_qhalf = '[Wzr1; Wzr2; Fdz; Mdx; u1; u2]';
input_to_Wzr1  = '[zr1]';
input_to_Wzr2  = '[zr2]';
input_to_Wu1   = '[u1]';
input_to_Wu2   = '[u2]';
input_to_Wzs   = '[qhalf(3)]';
input_to_Wrol  = '[qhalf(5)]';
cleanupsysic   = 'yes';
Pnom           = sysic;

%%% LTI Hinf controller
nmeas = 2;
ncon  = 2;

[hinfK,hinfCL,hinfGinf,info] = hinfsyn(Pnom,nmeas,ncon,'DISPLAY','ON')
CLhinf = lft(qhalf,hinfK);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Frequency response Linear
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w=logspace(-1,2,1000);

[sv1,wout1] = sigma(qhalf(3,1), w);
[sv2,wout2] = sigma(qhalf(3,2), w);
[sv3,wout3] = sigma(hinfGinf/Wzs/Wzr1, w);
[sv4,wout4] = sigma(CLhinf(3,1), w);
[sv5,wout5] = sigma(CLhinf(3,2), w);

[sv6,wout6] = sigma(qhalf(5,1), w);
[sv7,wout7] = sigma(qhalf(5,2), w);
[sv8,wout8] = sigma(hinfGinf/Wrol/Wzr1, w);
[sv9,wout9] = sigma(CLhinf(5,1), w);
[sv10,wout10] = sigma(CLhinf(5,2), w);

figure
subplot(211), semilogx(wout3,20*log10(sv3),'k-.', ...
                        wout1,20*log10(sv1), 'red',...
                        wout2,20*log10(sv2),'red', ...
                        wout4,20*log10(sv4),'blue', ...
                        wout5,20*log10(sv5),'blue'), grid on,
legend('Template','Open-loop left, right','','Closed-loop left, right','','interpreter','latex');
z=title(['Frequency response to z_s/z_r with \gamma = ' num2str(hinfGinf)])
% xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('Location','southwest');

subplot(212), semilogx(wout8,20*log10(sv8), 'k-.',...
                        wout6,20*log10(sv6),  'red',...
                        wout7,20*log10(sv7), 'red',...
                        wout9,20*log10(sv9), 'blue',...
                        wout10,20*log10(sv10),'blue'), grid on,
legend('Template','Open-loop left, right','','Closed-loop left, right','','interpreter','latex');
z=title(['Frequency response to \theta/z_r with \gamma = ' num2str(hinfGinf)])
xlabel('Frequency (rad/s)','interpreter','latex')
ylabel('Magnitude (dB)','interpreter','latex')
legend('Location','southwest');

% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'half_car_templates.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '0.8\textwidth', ... image width
%     'height', '0.45\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------

%% Simulation
sim("a3_anti_roll_control_sim.slx")

figure

simtime = tout;
zs      = half_car.Data(:,1);
zr_fr   = half_car.Data(:,2);
zr_fl   = half_car.Data(:,3);
zdef_fl = half_car.Data(:,4);
zdef_fr = half_car.Data(:,5);
subplot(211)
plot(simtime, zs, 'blue', simtime, zr_fr,'magenta',simtime, zr_fl,'red'); hold on
% xlabel('Time (s)','interpreter','latex')
ylabel('Bounce (m)','interpreter','latex')
legend('$z_s$','$z_{r,fr}$','$z_{r,fl}$','interpreter','latex');
xlim([0 10])
ylim([-0.005 0.015])
grid on
subplot(212)
plot(simtime, zdef_fr, 'magenta', simtime, zdef_fl,'red'); hold on
xlabel('Time (s)','interpreter','latex')
ylabel('Deflection (m)','interpreter','latex')
legend('$zdef_{fr}$','$zdef_{fl}$','interpreter','latex');
xlim([0 10])
grid on
% -----------------------------
% fig_loc = '/home/phatdo/Dropbox/tex_doc/clutch_and_antiroll_control/fig/';
% fig1_name = 'half_car_sim.tex';
% cleanfigure('targetResolution', 100);
% matlab2tikz(append(fig_loc, fig1_name), ... filename
%     'width', '0.8\textwidth', ... image width
%     'height', '0.45\textwidth', ... image height
%      'showInfo', false);  % ... turn off information
% -----------------------------
