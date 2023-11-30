% Author: Minh Phat Do
% Description: Consensus-based CACC

%% Reset workspace
clear;
clc;
close;

%% System definition

% Define the plattoon configuration
G = digraph([1 2 3], [2 3 4]);
plot(G)
Adj = adjacency(G)*eye(4);
S = [1;1;1;1];
L0 = Adj*S;
D0 = diag(L0);
L = D0-Adj;
P = diag([0 0 0 1]);
L_hat = L + P;
lamda_L_hat = eig(L_hat)
lamda_1 = lamda_L_hat(1);
% lamda_3 = lamda_L(3);
% lamda_4 = lamda_L(4);

%% LMIs setup

% Define SDP variables
X = sdpvar(3,3);
Y = sdpvar(1,3,'full');

tau = 0.1;
A = [0 1 0; 0 0 1; 0 0 -1/tau];
B = [0;0;1/tau];
C = [1 0 0];
D = [0;0;0];
% Co = ctrb(A,B)
% rank(Co)

% Define contraints and solve LMIs
F = [X*A' + A*X + lamda_1*B*Y + conj(lamda_1)*Y'*B' <= 0];
% F = [X>=0, F, trace(X)==1]
F = [X>=0, F]
optimize(F);


%% Results
K = value(Y)*inv(value(X))

% Test = eig(A+B*K)



