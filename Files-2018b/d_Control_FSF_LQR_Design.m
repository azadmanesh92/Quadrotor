clc;
clear all;
close all;
format compact;

%% Linearization Plant
sys = linmod('a_Quadrotor_Plant_Sim');

A  = sys.a;
BB = sys.b;
C  = sys.c;
D  = sys.d;

%% Analysis Complete Dynamics [Eigenvalues, Controllability and Observability]
Eig = eig(A);

Phi_C = ctrb(A, BB);
Size_Phi_C = size(Phi_C);
Rank_Phi_C = rank(Phi_C);

Phi_O = obsv(A, C);
Size_Phi_O = size(Phi_O);
Rank_Phi_O = rank(Phi_O);

%% Extract Linear Rotational Dynamics
% States Order
% X = [ [p q r] [phi theta psi] [u v w] [X Y Z] ] ^ T
% U = [ Thrust M_phi M_theta M_psi ] ^ T
%------------------------------------------
% New States Order [Rotational Dynamics]
% X = [ p q r phi theta psi ] ^ T
% U = [ M_phi M_theta M_psi ] ^ T

States_no   = [1 2 3 4 5 6];
Controls_no = [2 3 4];

for i = 1:length(States_no)
   for j = 1:length(States_no) 
      A_Reform(i, j) = A(States_no(i), States_no(j));
      C_Reform(i, j) = C(States_no(i), States_no(j));
   end
   for k = 1:length(Controls_no) 
      B_Reform(i, k) = BB(States_no(i), Controls_no(k));
      D_Reform(i, k) = D(States_no(i), Controls_no(k));
   end
end

States  = {'p', 'q', 'r', 'phi', 'theta', 'psi'};
Inputs  = {'M_phi', 'M_theta', 'M_psi'};
Outputs = {'p', 'q', 'r', 'phi', 'theta', 'psi'};
Lin_Sys_Reform = ss(A_Reform, B_Reform, C_Reform, D_Reform,...
         'statename', States, 'inputname',...
         Inputs, 'outputname', Outputs);

%% Analysis Rotational Dynamics [Eigenvalues, Controllability and Observability]
Eig_Reform = eig(A_Reform);

Phi_C_Reform      = ctrb(A_Reform, B_Reform);
Size_Phi_C_Reform = size(Phi_C_Reform);
Rank_Phi_C_Reform = rank(Phi_C_Reform);

Phi_O_Reform      = obsv(A_Reform, C_Reform);
Size_Phi_O_Reform = size(Phi_O_Reform);
Rank_Phi_O_Reform = rank(Phi_O_Reform);

%% Design Full State Feedback Controller
Desired_Poles = [-211 -311+50i -311-50i -6 -2 -4];
K_FSF = place(A_Reform, B_Reform, Desired_Poles);
FSF_CL_Eig = eig(A_Reform-B_Reform*K_FSF); %Close-Loop Eigenvalues

%% Design LQR Controller
Q_p   = deg2rad(1);
Q_q   = deg2rad(1);
Q_r   = deg2rad(1);
Q_phi = deg2rad(1);
Q_the = deg2rad(1);
Q_psi = deg2rad(1);

R_M_phi   = 1;
R_M_theta = 1;
R_M_psi   = 1;

Q = diag([1/(Q_p)^2 1/(Q_q)^2 1/(Q_r)^2 ...
          1/(Q_phi)^2 1/(Q_the)^2 1/(Q_psi)^2]);

R = diag([R_M_phi R_M_theta R_M_psi]);

K_LQR = lqr(A_Reform, B_Reform, Q, R);
LQR_CL_Eig = eig(A_Reform-B_Reform*K_LQR); %Close-Loop Eigenvalues

%% Initial Values
FSF_Inital = [0.1 0.2 0.01 0.5 0.6 0.2];
LQR_Inital = [0.1 0.2 0.01 0.5 0.6 0.2];

%% Run Simulation
out = sim('c_FSF_LQR_Sim.slx');

%% Plot Parameters
p_FSF = out.p_FSF;
q_FSF = out.q_FSF;
r_FSF = out.r_FSF;

phi_FSF = out.phi_FSF;
the_FSF = out.theta_FSF;
psi_FSF = out.psi_FSF;

p_LQR = out.p_LQR;
q_LQR = out.q_LQR;
r_LQR = out.r_LQR;

phi_LQR = out.phi_LQR;
the_LQR = out.theta_LQR;
psi_LQR = out.psi_LQR;

time = out.tout;

LW = 2;

figure(1)
subplot(3, 2, 1)
plot(time, rad2deg(p_FSF), 'linewidth', LW)
hold on
plot(time, rad2deg(p_LQR), '--', 'linewidth', LW)
hold off
grid on
title('Roll Rate')
legend('FSF', 'LQR')

subplot(3, 2, 2)
plot(time, rad2deg(q_FSF), 'linewidth', LW)
hold on
plot(time, rad2deg(q_LQR), '--', 'linewidth', LW)
hold off
grid on
title('Pitch Rate')
legend('FSF', 'LQR')

subplot(3, 2, 3)
plot(time, rad2deg(r_FSF), 'linewidth', LW)
hold on
plot(time, rad2deg(r_LQR), '--', 'linewidth', LW)
hold off
grid on
title('Yaw Rate')
legend('FSF', 'LQR')

subplot(3, 2, 4)
plot(time, rad2deg(phi_FSF), 'linewidth', LW)
hold on
plot(time, rad2deg(phi_LQR), '--', 'linewidth', LW)
hold off
grid on
title('Roll Angle')
legend('FSF', 'LQR')

subplot(3, 2, 5)
plot(time, rad2deg(the_FSF), 'linewidth', LW)
hold on
plot(time, rad2deg(the_LQR), '--', 'linewidth', LW)
hold off
grid on
title('Pitch Angle')
legend('FSF', 'LQR')
xlabel('time - Sec')

subplot(3, 2, 6)
plot(time, rad2deg(psi_FSF), 'linewidth', LW)
hold on
plot(time, rad2deg(psi_LQR), '--', 'linewidth', LW)
hold off
grid on
title('Yaw Angle')
legend('FSF', 'LQR')
xlabel('time - Sec')
