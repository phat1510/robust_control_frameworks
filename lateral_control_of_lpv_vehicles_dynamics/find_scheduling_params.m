clc
clear

%%%%-----------DID NOT WORK-------------%%%%%%%%%%%%%%%

vx_min = 5;
vx_max = 20;
vxin_max = 0.2;
vxin_min = 0.05;

rho = sdpvar(2,1,'full');
rho = [rho;1];

theta = [vx_min vx_max vxin_max vxin_min; 1 1 1 1];

alpha = sdpvar(4,1,'full');

F = [alpha >=0];
optimize(F, theta*alpha==rho);


