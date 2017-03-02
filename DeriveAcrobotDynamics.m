
clc; close all; clear all;

% Generates Acrobot Parameters symbolically
AcrobotParameters;

% Computes the Inertia, Coriolis and Gravity matrices
display('Computing Acrobot Dynamics Matrices')
[M,C,G] = AcrobotDynamicsMatrices(acr);
% Computes the acceleration of q1 and q2
display('Computing Joint acceleration equations')
DeriveAccel(M,C,G,acr);



