%% Reset the system
clc;
clear;
close;

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
Ts = 0.009;

%% LTI case

vx = 20;
A = [-(Cf+Cr)/(m*vx)    -vx + (-lf*Cf+lr*Cr)/(m*vx);
    (-lf*Cf+lr*Cr)/(Iz*vx) (lf^2*Cf+lr^2*Cr)/(Iz*vx)];
B = [Cf/m lf*Cf/Iz]';
C = diag([1 1]);
D = 0;

V_LIN = ss(A,B,C,D);
Q_gain_lin = diag([1 500]);
R_gain_lin = 0.1;
% K_lqr = -lqr(V_LIN, Q_gain_lin, R_gain_lin)

V_DIS = c2d(V_LIN, Ts);
A_dis = V_DIS.A;
B_dis = V_DIS.B;
C_dis = V_DIS.C;
D_dis = V_DIS.D;
K_dlqr = dlqr(A_dis, B_dis, Q_gain_lin, R_gain_lin)

%% LPV case

%--------------------
% To do
% define vx and 1/vx as sdp var
% update LMIs

% Define time varying parameters
Y = sdpvar(1,2,'full');
Q = Q_gain_lin;
R = R_gain_lin;
P = sdpvar(2,2);
W = sdpvar(3,3);

C_ = [Q^(1/2); zeros(1,2)];
D_ = [zeros(2,1); R^(1/2)];

F1 = [P>=0];
F2 = [[-P+Q (A_dis*P-B_dis*Y);(A_dis*P-B_dis*Y)' -P]<=0];
F3 = [[W (C_*P+D_*Y);(C_*P+D_*Y)' P]>=0];
F = [F1, F2, F3];
optimize(F, trace(W));

K_lmi = value(Y)*inv(value(P))

K_gain = K_lmi;
%% Another LMIs
% Y = sdpvar(2,2);
% L = sdpvar(1,2,'full');
% Q = diag([1 500]);
% R = 0.1;
% F = [Y >= 0];
% F = [F, [-A*Y-B*L+(-A*Y-B*L)' Y L';Y inv(Q) zeros(2,1); L zeros(1,2) inv(R)] >= 0];
% optimize(F,-trace(Y))
% K_gain_lin = value(L)*inv(value(Y))



%% Visualization

% sim("vehicle_dynamic.slx");
sim("vehicle_dynamic_dis.slx");

simtime = tout;
x_ref   = monitor.Data(:,1);
x_real  = monitor.Data(:,2);
y_ref   = monitor.Data(:,3);
y_real  = monitor.Data(:,4);

figure(1)
plot(x_ref, y_ref, x_real, y_real);
legend("ref", "real");
grid on