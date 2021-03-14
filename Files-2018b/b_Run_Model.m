clc;
clear all;
close all;
format compact;

%% Run Simulation
out = sim('a_Quadrotor_Plant_Sim.slx');

%% Plot Parameters
p = out.yout{1}.Values.Data;
q = out.yout{2}.Values.Data;
r = out.yout{3}.Values.Data;

phi = out.yout{4}.Values.Data;
the = out.yout{5}.Values.Data;
psi = out.yout{6}.Values.Data;

u = out.yout{7}.Values.Data;
v = out.yout{8}.Values.Data;
w = out.yout{9}.Values.Data;

X = out.yout{10}.Values.Data;
Y = out.yout{11}.Values.Data;
Z = out.yout{12}.Values.Data;

time = out.tout;

thrust = out.Thrust;
Mphi   = out.Mphi;
Mtheta = out.Mtheta;
Mpsi   = out.Mpsi;

LW = 2;

figure(1)
subplot(3, 2, [1 2])
plot(time, thrust.*ones(size(time, 1), 1), 'linewidth', LW)
hold on
plot(time, Mphi.*ones(size(time, 1), 1), 'linewidth', LW)
plot(time, Mtheta.*ones(size(time, 1), 1), 'linewidth', LW)
plot(time, Mpsi.*ones(size(time, 1), 1), 'linewidth', LW)
hold off
grid on
title('Inputs')
xlabel('time - Sec')
legend('Thrust', 'M_{\phi}', 'M_{\theta}', 'M_{\psi}')
ylim([-1 6])

subplot(3, 2, 3)
plot(time, u, 'linewidth', LW)
hold on
plot(time, v, 'linewidth', LW)
plot(time, w, 'linewidth', LW)
hold off
grid on
legend('u', 'v', 'w')
title('Linear Velocity')
xlabel('time - Sec')
ylim([-1 1])

subplot(3, 2, 4)
plot(time, p, 'linewidth', LW)
hold on
plot(time, q, 'linewidth', LW)
plot(time, r, 'linewidth', LW)
hold off
grid on
legend('p', 'q', 'r')
title('Body Rates')
xlabel('time - Sec')
ylim([-1 1])

subplot(3, 2, 5)
plot(time, rad2deg(phi), 'linewidth', LW)
hold on
plot(time, rad2deg(the), 'linewidth', LW)
plot(time, rad2deg(psi), 'linewidth', LW)
hold off
grid on
legend('{\phi}', '{\theta}', '{\psi}')
title('Euler angles')
xlabel('time - Sec')
ylim([-1 1])

subplot(3, 2, 6)
plot(time, X, 'linewidth', LW)
hold on
plot(time, Y, 'linewidth', LW)
plot(time, Z, 'linewidth', LW)
hold off
grid on
legend('X', 'Y', 'Z')
title('Position')
xlabel('time - Sec')
ylim([-100 700])
