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
L = [80 ... go str
    50 ... clo
    50 ... cir
    50 ... go str
    60 ... clo
    130 ... cir
    20 ... go str
    70 ... clo
    40 ... cir
    20 ... go str
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


%% LTI case + DLQR
% Test vx from 10 to 35 m/s (36 to 126 km/h)
vx = 10; %m/s
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
K_dlqr = dlqr(A_dis, B_dis, Q_gain_lin, R_gain_lin)

%% LPV case
vx_min = 5;
vx_max = 20;
vxin_max = 1/vx_min;
vxin_min = 1/vx_max;

range = [vx_min vx_max;
        1/vx_max 1/vx_min];
p = [vx 1/vx]';

[alpha, vertx] = my_polydec(p,range);

A1 = [-(Cf+Cr)*vxin_max/m    -vx_max + (-lf*Cf+lr*Cr)*vxin_max/m 0;
    (-lf*Cf+lr*Cr)*vxin_max/Iz (lf^2*Cf+lr^2*Cr)*vxin_max/Iz 0;
    0 1 0];
A2 = [-(Cf+Cr)*vxin_min/m    -vx_min + (-lf*Cf+lr*Cr)*vxin_min/m 0;
    (-lf*Cf+lr*Cr)*vxin_min/Iz (lf^2*Cf+lr^2*Cr)*vxin_min/Iz 0;
    0 1 0];
A3 = [-(Cf+Cr)*vxin_min/m    -vx_max + (-lf*Cf+lr*Cr)*vxin_min/m 0;
    (-lf*Cf+lr*Cr)*vxin_min/Iz (lf^2*Cf+lr^2*Cr)*vxin_min/Iz 0;
    0 1 0];
A4 = [-(Cf+Cr)*vxin_max/m    -vx_min + (-lf*Cf+lr*Cr)*vxin_max/m 0;
    (-lf*Cf+lr*Cr)*vxin_max/Iz (lf^2*Cf+lr^2*Cr)*vxin_max/Iz 0;
    0 1 0];

% Continuous linear system at vertices
V_LIN_01 = ss(A1,B,C,D);
V_LIN_02 = ss(A2,B,C,D);
V_LIN_03 = ss(A3,B,C,D);
V_LIN_04 = ss(A4,B,C,D);

% Discrete linear system at vertices
V_DIS_01 = c2d(V_LIN_01, Ts);
V_DIS_02 = c2d(V_LIN_02, Ts);
V_DIS_03 = c2d(V_LIN_03, Ts);
V_DIS_04 = c2d(V_LIN_04, Ts);

% Extract system matrices
A_dis1 = V_DIS_01.A;
B_dis1 = V_DIS_01.B;
A_dis2 = V_DIS_02.A;
B_dis2 = V_DIS_02.B;
A_dis3 = V_DIS_03.A;
B_dis3 = V_DIS_03.B;
A_dis4 = V_DIS_04.A;
B_dis4 = V_DIS_04.B;

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

% K_gain = K_lmi;
K_gain = K_dlqr;

%% Full LMIs
Y1 = sdpvar(1,3,'full');
Y2 = sdpvar(1,3,'full');
Y3 = sdpvar(1,3,'full');
Y4 = sdpvar(1,3,'full');

F1 = [P>=0];

F21 = [[-P+Q (A_dis1*P-B_dis1*Y1);(A_dis1*P-B_dis1*Y1)' -P]<=0];
F22 = [[-P+Q (A_dis2*P-B_dis2*Y2);(A_dis2*P-B_dis2*Y2)' -P]<=0];
F23 = [[-P+Q (A_dis3*P-B_dis3*Y3);(A_dis3*P-B_dis3*Y3)' -P]<=0];
F24 = [[-P+Q (A_dis4*P-B_dis4*Y4);(A_dis4*P-B_dis4*Y4)' -P]<=0];

F31 = [[W (C_*P+D_*Y1);(C_*P+D_*Y1)' P]>=0];
F32 = [[W (C_*P+D_*Y2);(C_*P+D_*Y2)' P]>=0];
F33 = [[W (C_*P+D_*Y3);(C_*P+D_*Y3)' P]>=0];
F34 = [[W (C_*P+D_*Y4);(C_*P+D_*Y4)' P]>=0];

F_full = [F1,F21,F22,F23,F24,F31,F32,F33,F34];
optimize(F_full, trace(W),ops);

K_lmi1 = value(Y1)*inv(value(P));
K_lmi2 = value(Y2)*inv(value(P));
K_lmi3 = value(Y3)*inv(value(P));
K_lmi4 = value(Y4)*inv(value(P));


K_lmi_lpv = alpha(1)*K_lmi2 + alpha(2)*K_lmi3 + alpha(3)*K_lmi4 + alpha(4)*K_lmi1
%% Visualization
% sim("vehicle_dynamic_dis_3states.slx");
% 
% simtime = tout;
% x_ref   = monitor.Data(:,1);
% x_real  = monitor.Data(:,2);
% y_ref   = monitor.Data(:,3);
% y_real  = monitor.Data(:,4);
% 
% figure(1)
% plot(x_ref, y_ref, x_real, y_real);
% legend("ref", "real");
% grid on

function wp_time = wpt(segment_length, vx)
time = 0;
    for i = 2:(length(segment_length)+1)
        time = time + segment_length(i-1)/vx;
        wp_time(i) = time;
    end

end