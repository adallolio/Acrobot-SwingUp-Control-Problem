%% Linearization of the AcrobatRobot
clear all; close all; clc;
acr = AcrobotParameters('sym');
acr2 = AcrobotParameters('num');
init = [pi/2, 0, 0, 0,0];

%% Obtain the simbolic dynamic matrices
[M, C, G] = AcrobotDynamicsMatrices(acr);

%% Generic case (Full linearization)
eq1 = M(1,1)*acr.q1dd+M(1,2)*acr.q2dd + C(1) + G(1);
eq2 = M(2,2)*acr.q1dd+M(2,1)*acr.q2dd + C(2) + G(2)-acr.T2;
[q1dd,q2dd] = solve([eq1,eq2],[acr.q1dd,acr.q2dd]);

Adallo = jacobian([acr.q1d,acr.q2d,q1dd,q2dd],[acr.q1,acr.q2,acr.q1d,acr.q2d]);
Bdallo = jacobian([acr.q1d,acr.q2d,q1dd,q2dd],acr.T2);

AGeneric = double(subs(Adallo,[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]));
BGeneric = double(subs(Bdallo,[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]));

%% Computes the Linearization of the non-collocated approach

M21bar = M(2,1)-M(2,2)\M(1,2)*M(1,1);
h1bar = C(2)-M(2,2)\M(1,2)*C(1);
phi1bar = G(2)-M(2,2)\M(1,2)*G(1);
q1dd_juan = (acr.T2 - h1bar -phi1bar)/M21bar;
%Tc1 = d1bar*v1 + h1bar + phi1bar;
%M(1,2)*q2dd + C(1) + G(1) = -M(1,1)*v1;
q2dd_juan = -(M(1,1)*q1dd_juan+C(1)+G(1))/M(1,2);

Ajuan = jacobian([acr.q1d,acr.q2d,q1dd_juan,q2dd_juan],[acr.q1,acr.q2,acr.q1d,acr.q2d]);
Bjuan = jacobian([acr.q1d,acr.q2d,q1dd_juan,q2dd_juan],acr.T2);

ANonColl = double(subs(Ajuan,[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]));
BNonColl = double(subs(Bjuan,[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]));

%% Computes the Linearization of the collocated approach

M22bar = M(2,2)-M(2,1)*1/M(1,1)*M(1,2);
h2bar = C(2)-M(2,1)*1/M(1,1)*C(1);
phi2bar = G(2)-M(2,1)*1/M(1,1)*G(1);
q2dd_juan = (acr.T2 - h2bar -phi2bar)/M22bar;
%Tc1 = d1bar*v1 + h1bar + phi1bar;
%M(1,2)*q2dd + C(1) + G(1) = -M(1,1)*v1;
q1dd_juan = -(M(1,2)*q2dd_juan+C(1)+G(1))/M(1,1);

Ajuan = jacobian([acr.q1d,acr.q2d,q1dd_juan,q2dd_juan],[acr.q1,acr.q2,acr.q1d,acr.q2d]);
Bjuan = jacobian([acr.q1d,acr.q2d,q1dd_juan,q2dd_juan],acr.T2);

AColl = double(subs(Ajuan,[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]));
BColl = double(subs(Bjuan,[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]));

%% Save the State Space matrices
save('SS_Matrices.mat', 'AColl', 'AGeneric', 'ANonColl', 'BColl', 'BGeneric', 'BNonColl');