clc
clear
close all

%% System definition

vX = 0;
vY = 0;
vx = 0;
v = 0;
psi = 0;
rho = 0;
yaw_rate = 0;
Ts = 0.01;

psi_plot = 0;

X = 0;
Y = 0;

v_ref = 5; %km/s

vX = v * cos(psi);
vY = v * sin(psi);

v = v_ref;
rho = 0;
yaw_rate = v_ref * rho;

t_s1 = 40 / v_ref;
t_s2 = 50 / v_ref;
t_s3 = 50 / v_ref;


% k = s/A;
% s: arc length

%% Phase 1
step = t_s1 / Ts;
i = 1;
for t = 0:step
    psi = psi + reference_generator(v, 0)*Ts;
    vX = v * cos(psi);
    vY = v * sin(psi);
    X = X + vX * Ts;
    Y = Y + vY * Ts;
    X_plot(i) = X;
    Y_plot(i) = Y;
    psi_plot(i) = psi;
    i = i +1;
end

%% Phase 2
step = t_s2 / Ts;
i=0;
arc_length = 0;
for t = 0:step
    i = i + 1;
    arc_length = arc_length + v_ref * Ts;
    rho = -arc_length/2500;
    psi = psi + reference_generator(v, rho)*Ts;
    vX = v * cos(psi);
    vY = v * sin(psi);
    X = X + vX * Ts;
    Y = Y + vY * Ts;
    X_plot2(i) = X;
    Y_plot2(i) = Y;
    psi_plot2(i) = psi;
end

%% Phase 3
step = t_s3 / Ts;
rho = -1/50;
yaw_rate = v_ref * rho;
i=0;
for t = 0:step
    i = i + 1;
    psi = psi + reference_generator(v, rho)*Ts;
    vX = v * cos(psi);
    vY = v * sin(psi);
    X = X + vX * Ts;
    Y = Y + vY * Ts;
    X_plot3(i) = X;
    Y_plot3(i) = Y;
    psi_plot3(i) = psi;
end

figure(1)
plot(X_plot,Y_plot, X_plot2, Y_plot2,':', X_plot3, Y_plot3, '--', 'LineWidth',2);

axis([0 140 -70 70])
ylabel('$Y$','interpreter','latex','FontSize',12);
xlabel('$X$','interpreter','latex','FontSize',12);
legend("phase 1", "phase 2", "phase 3", 'interpreter','latex','FontSize',12)
grid on

