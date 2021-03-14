clc;
clear all;
close all;
format compact;

%% Linearization Plant
sys = linmod('a_Quadrotor_Plant_Sim');

A = sys.a;
B = sys.b;
C = sys.c;
D = sys.d;

%% Extract Linear Rotational Dynamics
% States Order
% X = [ [p q r] [phi theta psi] [u v w] [X Y Z] ] ^ T
% U = [ Thrust M_phi M_theta M_psi ] ^ T
%------------------------------------------
% New States Order [Rotational Dynamics]
% X = [ p q r phi theta psi] ^ T
% U = [ M_phi M_theta M_psi ] ^ T

States_no = [1 2 3 4 5 6];
Control_no = [2 3 4];

for i = 1:length(States_no)
   for j = 1:length(States_no) 
      A_Reform(i, j) = A(States_no(i), States_no(j));
      C_Reform(i, j) = C(States_no(i), States_no(j));
   end
   for k = 1:length(Control_no) 
      B_Reform(i, k) = B(States_no(i), Control_no(k));
      D_Reform(i, k) = D(States_no(i), Control_no(k));
   end
end

%% Analysis Rotational Dynamics [Eigenvalues, Controllability and Observability]
Eig_Reform = eig(A_Reform);

Phi_C_Reform = ctrb(A_Reform, B_Reform);
Size_Phi_C_Reform = size(Phi_C_Reform);
Rank_Phi_C_Reform = rank(Phi_C_Reform);

Phi_O_Reform = obsv(A_Reform, C_Reform);
Size_Phi_O_Reform = size(Phi_O_Reform);
Rank_Phi_O_Reform = rank(Phi_O_Reform);

if size(A_Reform, 2) == rank(Phi_C_Reform)
    disp('System is Controllable')
else
    disp('System is not Controllable')
end

if size(A_Reform, 2) == rank(Phi_O_Reform)
    disp('System is Observable')
else
    disp('System is not Observable')
end

%% Design Controller and Observer Gain [K and L]
poles = [-5 -5 -5 -1 -1 -1];
K = place(A_Reform, B_Reform, poles);

Obpoles = 3*poles;
L = place(A_Reform', C_Reform', Obpoles);
L = L';

%% Run Simulation
Initial_Values = [0.01 0.01 0.06 0.2 0.1 0.3];
out = sim('b_Observer_Design_Sim.slx');

%% Plot Parameters
p_act = out.act_states(:, 1);
q_act = out.act_states(:, 2);
r_act = out.act_states(:, 3);

phi_act = out.act_states(:, 4);
the_act = out.act_states(:, 5);
psi_act = out.act_states(:, 6);

p_est = out.est_states(:, 1);
q_est = out.est_states(:, 2);
r_est = out.est_states(:, 3);

phi_est = out.est_states(:, 4);
the_est = out.est_states(:, 5);
psi_est = out.est_states(:, 6);

time = out.tout;

LW = 2;

figure(1)
subplot(3, 2, 1)
plot(time, rad2deg(p_act), 'linewidth', LW)
hold on
plot(time, rad2deg(p_est), '--', 'linewidth', LW)
hold off
grid on
title('Roll Rate')
legend('Actual', 'Estimated')

subplot(3, 2, 2)
plot(time, rad2deg(q_act), 'linewidth', LW)
hold on
plot(time, rad2deg(q_est), '--', 'linewidth', LW)
hold off
grid on
title('Pitch Rate')
legend('Actual', 'Estimated')

subplot(3, 2, 3)
plot(time, rad2deg(r_act), 'linewidth', LW)
hold on
plot(time, rad2deg(r_est), '--', 'linewidth', LW)
hold off
grid on
title('Yaw Rate')
legend('Actual', 'Estimated')

subplot(3, 2, 4)
plot(time, rad2deg(phi_act), 'linewidth', LW)
hold on
plot(time, rad2deg(phi_est), '--', 'linewidth', LW)
hold off
grid on
title('Roll Angle')
legend('Actual', 'Estimated')

subplot(3, 2, 5)
plot(time, rad2deg(the_act), 'linewidth', LW)
hold on
plot(time, rad2deg(the_est), '--', 'linewidth', LW)
hold off
grid on
title('Pitch Angle')
legend('Actual', 'Estimated')
xlabel('time - Sec')

subplot(3, 2, 6)
plot(time, rad2deg(psi_act), 'linewidth', LW)
hold on
plot(time, rad2deg(psi_est), '--', 'linewidth', LW)
hold off
grid on
title('Yaw Angle')
legend('Actual', 'Estimated')
xlabel('time - Sec')

