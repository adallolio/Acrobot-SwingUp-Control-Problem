
clc; close all; clear all;

% Generates Acrobot Parameters symbolically
acr = AcrobotParameters('sym');

% Computes the Inertia, Coriolis and Gravity matrices
display('Computing Acrobot Dynamics Matrices')
[M,C,G] = AcrobotDynamicsMatrices(acr);
% Computes the acceleration of q1 and q2
display('Computing Joint acceleration equations')
DeriveJointAcceleration(M,C,G,acr);
% Computes the torque
display('Computing Input torques equations')
DeriveTorques(M, C, G, acr);


