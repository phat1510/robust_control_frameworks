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
% L = [80 ... go str
%     40 ... clo
%     50 ... cir
%     50 ... go str
%     40 ... clo
%     100 ... cir
%     50 ... go str
%     40 ... clo
%     30 ... cir
%     20 ... go str
%     ];
L = [80 ... go str
    40 ... clo
    250 ... cir
    1 ... go str
    1 ... clo
    1 ... cir
    1 ... go str
    1 ... clo
    1 ... cir
    1 ... go str
    ];
vx_track = 5;
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
t1 = 10;
t2 = 50;
t3 = 80;
t4 = 100;
accel = 2; %m/s^2 accel


%% LTI case
vx =15;
A = [-(Cf+Cr)/(m*vx)    -vx + (-lf*Cf+lr*Cr)/(m*vx) 0;
    (-lf*Cf+lr*Cr)/(Iz*vx) (lf^2*Cf+lr^2*Cr)/(Iz*vx) 0;
    0 1 0];
B = [Cf/m lf*Cf/Iz 0]';
C = diag([1 1 1]);
D = 0;

V_LIN = ss(A,B,C,D);
Q_gain_lin = diag([1 1 1000]);
R_gain_lin = 0.1;
% K_lqr = -lqr(V_LIN, Q_gain_lin, R_gain_lin)

V_DIS = c2d(V_LIN, Ts);
A_dis = V_DIS.A;
B_dis = V_DIS.B;
C_dis = V_DIS.C;
D_dis = V_DIS.D;
K_dlqr = dlqr(A_dis, B_dis, Q_gain_lin, R_gain_lin)

%% LPV case
vx_min = 10;
vx_max = 60;
vxin_max = 1/vx_min;
vxin_min = 1/vx_max;

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
% optimize(F, trace(W));

% K_lmi = value(Y)*inv(value(P))

% K_gain = K_lmi;
K_gain = K_dlqr;

%% Full LMIs
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
optimize(F_full, trace(W));

Ka = value(Ya)*inv(value(P))
Kb = value(Yb)*inv(value(P))
Kc = value(Yc)*inv(value(P))

%% Visualization
sim("vehicle_dynamic_dis_3states.slx");

simtime = tout;
x_ref   = monitor.Data(:,1);
x_real  = monitor.Data(:,2);
y_ref   = monitor.Data(:,3);
y_real  = monitor.Data(:,4);

figure(1)
plot(x_ref, y_ref, x_real, y_real, LineWidth=1);
legend("ref", "real");
grid on

function wp_time = wpt(segment_length, vx)
time = 0;
    for i = 2:(length(segment_length)+1)
        time = time + segment_length(i-1)/vx;
        wp_time(i) = time;
    end

end