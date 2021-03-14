clc;
clear all;
close all;
format compact;

%% Quadrotor Data
g = 9.8;
m = 0.468;
K = 2.98e-06;
L = 0.225;
B = 0.114e-06;

%% Initial Values
p0 = 0;
q0 = 0;
r0 = 0;

phi0 = deg2rad(0);
the0 = deg2rad(0);
psi0 = deg2rad(0);

u0 = 0;
v0 = 0;
w0 = 0;

x0 = 0;
y0 = 0;
z0 = 500;

X0 = [p0 q0 r0 phi0 the0 psi0 u0 v0 w0 x0 y0 z0];