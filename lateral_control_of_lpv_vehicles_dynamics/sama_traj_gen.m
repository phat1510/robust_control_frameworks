function [phi, phi_dot, time] = reference_generator(vx, rho, ts)
% %time = [0 : ts : (length(rho) - 1)*ts];
% % Yaw rate
% phi_dot = rho*vx;
% % Yaw angle
% phi = cumtrapz(ts, phi_dot);

d=0;

for d = 0:140;
    if 0<d<=40
        rho=0   
    elseif 40<d<=90
        arclength=d - 40
        rho=-arclength/3500
    else
        rho=-1/50
    end
d=vx*ts
end
% Yaw rate
phi_dot = rho*vx;
% Yaw angle
phi = cumtrapz(ts, phi_dot);