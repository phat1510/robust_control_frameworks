%% Reset the system
clc;
clear;
close;

%% TO DO
% *Compute the vector [1 rho1 rho2]
% *Complete the simulation
% *Test with a simple input
% *Generate a full track for testing with vx changing over time
% *Conclude


% System definition
Cf = 57117;
Cr = 81396;
Iz = 1975;
m = 1621;
g = 9.8;
lf = 1.15;
lr = 1.38;
r = 0.2;
h = 1.5;
Ts = 0.01;

%% Track
L = [40 ... go str
    50 ... clo
    50 ... cir
%     50 ... clo
%     40 ... go str
%     50 ... clo
%     50 ... cir
%     50 ... clo
%     40 ... go str
%     50 ... clo
%     50 ... cir
%     50 ... clo
%     40 ... go str
%     50 ... clo
%     50 ... cir
%     50 ... clo
%     40 ... go str
    ];
accel = 0.006;
% L = [300 ... go str
%     50 ... clo
%     120 ... cir
%     70 ... go str
%     60 ... clo
%     600 ... cir
%     100 ... go str
%     60 ... clo
%     720 ... cir
%     20 ... go str
%     ];

vx_track = 15;
wp = wpt(L,vx_track);

% sim("track_generator.slx");
% 
% sim_time = tout;
% x_ref = track.Data(:,1);
% y_ref = track.Data(:,2);
% 
% figure;
% plot(x_ref, y_ref);
% grid on;

%% Vx profile
% Velocity profile params
% t1 = 10;
% t2 = 50;
% t3 = 80;
% t4 = 100;
% accel = 2; %m/s^2 accel

vx = 14; %m/s

%% LMI LTI case
A = [-(Cf+Cr)/(m*vx)    -vx + (-lf*Cf+lr*Cr)/(m*vx) 0;
    (-lf*Cf+lr*Cr)/(Iz*vx) (lf^2*Cf+lr^2*Cr)/(Iz*vx) 0;
    0 1 0];
B = [Cf/m lf*Cf/Iz 0]';
C = diag([1 1 1]);
D = 0;

V_LIN = ss(A,B,C,D);
Q_gain_lin = diag([0.1 10 10]); 
R_gain_lin = 1;

V_DIS = c2d(V_LIN, Ts);
A_dis = V_DIS.A;
B_dis = V_DIS.B;
C_dis = V_DIS.C;
D_dis = V_DIS.D;
K_dlqr = dlqr(A_dis, B_dis, Q_gain_lin, R_gain_lin);

%% LPV case + LMI
% Define time varying parameters
Y = sdpvar(1,3,'full');
Q = Q_gain_lin;
R = R_gain_lin;
P = sdpvar(3,3);
W = sdpvar(4,4);

C_ = [Q^(1/2); zeros(1,3)];
D_ = [zeros(3,1); R^(1/2)];

F1 = [P>=0];
F2 = [[-P+Q (A_dis*P-B_dis*Y);(A_dis*P-B_dis*Y)' -P]<=0];
F3 = [[W (C_*P+D_*Y);(C_*P+D_*Y)' P]>=0];
F = [F1, F2, F3];

ops = sdpsettings;
ops.verbose =0;
optimize(F, trace(W),ops);
K_lmi = value(Y)*inv(value(P))

K_dlqr
K_gain = K_lmi
% K_gain = K_dlqr;

%% Full LMIs
% Define vertices
vx_min = 5;
vx_max = 20;
vxin_max = 1/vx_min;
vxin_min = 1/vx_max; 

% Scheduling parameters
rho1 = sdpvar(1); % vx
rho2 = sdpvar(1); % 1/vx

% Parameterize system matrix
A_full = [-(Cf+Cr)*rho2/m    -rho1 + (-lf*Cf+lr*Cr)*rho2/m 0;
    (-lf*Cf+lr*Cr)*rho2/Iz (lf^2*Cf+lr^2*Cr)*rho2/Iz 0;
    0 1 0];
A_full = A_full*Ts + eye(3);
B_full = B * Ts;

% Parameterize Y
Ya = sdpvar(1,3,'full');
Yb = sdpvar(1,3,'full');
Yc = sdpvar(1,3,'full');
Y_final = sdpvar(1,3,'full');
Y_final = Ya + rho1*Yb + rho2*Yc;

F1 = [P>=0];
F2 = [[-P+Q (A_full*P-B_full*Y_final);(A_full*P-B_full*Y_final)' -P]<=0];
F3 = [[W (C_*P+D_*Y_final);(C_*P+D_*Y_final)' P]>=0];

F_full = [F1, F2, F3, vx_min <= rho1 <= vx_max, uncertain(rho1), ...
    vxin_min <= rho2 <= vxin_max, uncertain(rho2)];

ops = sdpsettings;
ops.verbose =0;
optimize(F_full, trace(W),ops);

Ka = value(Ya)*inv(value(P));
Kb = value(Yb)*inv(value(P));
Kc = value(Yc)*inv(value(P));

% Compare
K_lpv = Ka + vx * Kb + 1/vx * Kc

%% Visualization
sim("vehicle_dynamic_dis_3states.slx");

simtime = tout;
x_ref   = monitor.Data(:,1);
x_real  = monitor.Data(:,2);
y_ref   = monitor.Data(:,3);
y_real  = monitor.Data(:,4);
f_contact = monitor.Data(:,5);
vx_ref = monitor.Data(:,6);
vy = monitor.Data(:,7);
steer = monitor.Data(:,8);

x_real_lti = LTI.Data(:,1);
y_real_lti = LTI.Data(:,2);

%--------------------------------------------------
% SIMPLE TRAJECTORY
%--------------------------------------------------
FontSize = 10;
figure(1)

% Plot trajectories
subplot(1,2,1);
plot(x_ref, y_ref, x_real_lti, y_real_lti, x_real, y_real, LineWidth=1);
legend('Reference', 'LTI', 'LPV','FontSize',FontSize, ...
    'interpreter','latex');
xlabel('x (meter)','FontSize',FontSize, 'interpreter','latex');
ylabel('y (meter)','FontSize',FontSize, 'interpreter','latex');
title('a. Position tracking performance','FontSize',FontSize, 'interpreter','latex');
% xlim([-400 800])
% ylim([-1100 100])
xlim([0 140])
ylim([70-140 70])
axis square
grid on

% Plot tracking errors
subplot(1,2,2);
lane_error_vec = [x_real-x_ref y_real-y_ref];
lane_error = zeros(length(lane_error_vec),1);
lti_lane_error_vec = [x_real_lti-x_ref y_real_lti-y_ref];
lti_lane_error = zeros(length(lti_lane_error_vec),1);

for i=1:length(tout)
    lane_error(i) = norm(lane_error_vec(i));
    lti_lane_error(i) = norm(lti_lane_error_vec(i));
end

plot(tout, lti_lane_error,  LineWidth=1);
hold on
plot(tout, lane_error, LineWidth=1); 
legend('LTI', 'LPV', ...
    'FontSize',FontSize, ...
    'interpreter','latex', ...
    'Location', 'northwest');

xlabel('Time (s)','FontSize',FontSize, 'interpreter','latex');
ylabel('Position error (meter)','FontSize',FontSize, 'interpreter','latex');
title('b. Lane tracking error $|e_{track}|$','FontSize',FontSize, 'interpreter','latex');
axis square
grid on

cleanfigure('targetResolution', 300);
matlab2tikz('figs/lpv_vs_lti.tex', ... filename
    'width', '14cm', ... image width
    'showInfo', false, ... turn off information
    'strictFontSize',true);

%--------------------------------------------------
% MORE ANALYSIS
%--------------------------------------------------


% figure(2)
% subplot(2,2,1);
% plot(tout, f_contact, LineWidth=1);
% ylabel('Lateral force')
% grid on;
% subplot(2,2,2);
% plot(tout, vy, LineWidth=1);
% ylabel('Lateral velocity')
% grid on;
% subplot(2,2,3);
% plot(tout, vx_ref, LineWidth=1);
% ylabel('Longitudinal velocity')
% grid on;
% subplot(2,2,4);
% plot(tout, steer, LineWidth=1);
% ylabel('Steering angle')
% grid on;

velocity_profile = true;
if velocity_profile
    figure(2)
    plot(tout, vx_ref, LineWidth=1.5);
    ylabel('$v_x$ (m/s)','FontSize',FontSize, 'interpreter','latex')
    xlabel('Time (s)','FontSize',FontSize, 'interpreter','latex')
    xlim([0 9.3])
    ylim([10 15])
    grid on;

    cleanfigure('targetResolution', 300);
    matlab2tikz('figs/vel_prof.tex', ... filename
    'width', '6cm', ... image width
    'showInfo', false, ... turn off information
    'strictFontSize',true);
end


%--------------------------------------------------
% LATERAL FORCE VALIDATION
%--------------------------------------------------
force_validation = false
if (force_validation)
figure(3)
% Plot trajectories
subplot(1,2,1);
plot(x_ref, y_ref, LineWidth=1.5);
xlabel('x (meter)','FontSize',FontSize, 'interpreter','latex');
ylabel('y (meter)','FontSize',FontSize, 'interpreter','latex');
title('a. Reference trajectory','FontSize',FontSize, 'interpreter','latex');
xlim([0 140])
ylim([50-140 50])
axis square
grid on

% Plot tracking errors
subplot(1,2,2);
plot(tout, f_contact, LineWidth=1.5);
ylabel('Lateral force (N)','FontSize',FontSize, 'interpreter','latex')
xlabel('Time (s)','FontSize',FontSize, 'interpreter','latex');
title('b. Lateral force','FontSize',FontSize, 'interpreter','latex');
xlim([0 9.5])
axis square
grid on

cleanfigure('targetResolution', 300);
matlab2tikz('figs/traj_valid.tex', ... filename
    'width', '14cm', ... image width
    'showInfo', false, ... turn off information
    'strictFontSize',true);

end


%--------------------------------------------------
% LATERAL FORCE VALIDATION
%--------------------------------------------------
function wp_time = wpt(segment_length, vx)
time = 0;
    for i = 2:(length(segment_length)+1)
        time = time + segment_length(i-1)/vx;
        wp_time(i) = time;
    end

end

function K_dlqr = discreteLQR(vx)
% System definition
Cf = 57117;
Cr = 81396;
Iz = 1975;
m = 1621;
g = 9.8;
lf = 1.15;
lr = 1.38;
r = 0.2;
h = 1.5;
Ts = 0.01;

A = [-(Cf+Cr)/(m*vx)    -vx + (-lf*Cf+lr*Cr)/(m*vx) 0;
    (-lf*Cf+lr*Cr)/(Iz*vx) (lf^2*Cf+lr^2*Cr)/(Iz*vx) 0;
    0 1 0];
B = [Cf/m lf*Cf/Iz 0]';
C = diag([1 1 1]);
D = 0;

V_LIN = ss(A,B,C,D);
Q_gain_lin = diag([1 10 10]);
R_gain_lin = 1;

V_DIS = c2d(V_LIN, Ts);
A_dis = V_DIS.A;
B_dis = V_DIS.B;
C_dis = V_DIS.C;
D_dis = V_DIS.D;
K = dlqr(A_dis, B_dis, Q_gain_lin, R_gain_lin);
K_dlqr = K;
end
