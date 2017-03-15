function [E,eqn1,eqn2] = DeriveEnergy(acr)

syms k1 k2 real

i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%Rotating reference frames
ref1 = [cos(acr.q1), sin(acr.q1), 0]'; %Just changed it so 0 is horizontal to the right.
ref2 = [cos(acr.q2+acr.q1), sin(acr.q2+acr.q1), 0]';

dref1 = [-sin(acr.q1),cos(acr.q1),0]';
dref2 = [-sin(acr.q2+acr.q1),cos(acr.q2+acr.q1),0]';

%Vectors to significant points
ac1 = acr.d1*ref1; %A is fixed point, B is the elbow, c1 and c2 are COMs
bc2 = acr.d2*ref2;
ab = acr.l1*ref1;
ac2 = ab + bc2;

%Velocities
vc1 = acr.d1*acr.q1d*dref1;
vb = acr.l1*acr.q1d*dref1;
vc2 = vb + acr.d2*(acr.q2d+acr.q1d)*dref2;

%Accelerations
Ac1 = acr.d1*acr.q1dd*dref1 - acr.d1*acr.q1d^2*ref1;
AB = acr.l1*acr.q1dd*dref1 - acr.l1*acr.q1d^2*ref1;
Ac2 = acr.d2*(acr.q2dd+acr.q1dd)*dref2 - acr.d2*(acr.q1d + acr.q2d)^2*ref2 + AB;

P = acr.g0*dot(ac2,j)*acr.m2 + acr.g0*dot(ac1,j)*acr.m1;
K = 1/2*acr.I1*acr.q1d^2 + 1/2*acr.I2*(acr.q2d+acr.q1d)^2 + 1/2*acr.m1*dot(vc1,vc1) + 1/2*acr.m2*dot(vc2,vc2);

E = P + K;

matlabFunction(E, 'file', '/home/dallo/Desktop/URproject/UR/ComputeEnergy');