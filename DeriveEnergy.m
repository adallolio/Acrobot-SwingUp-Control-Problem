syms m1 m2 I1 I2 g0 l1 l2 d1 d2 q1 q2 q1d q2d q1dd q2dd ref1 ref2 dref1 dref2 T1 T2 k1 k2 real

i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%Rotating reference frames
ref1 = [cos(q1), sin(q1), 0]'; %Just changed it so 0 is horizontal to the right.
ref2 = [cos(q2+q1), sin(q2+q1), 0]';

dref1 = [-sin(q1),cos(q1),0]';
dref2 = [-sin(q2+q1),cos(q2+q1),0]';

%Vectors to significant points
ac1 = d1*ref1; %A is fixed point, B is the elbow, c1 and c2 are COMs
bc2 = d2*ref2;
ab = l1*ref1;
ac2 = ab + bc2;

%Velocities
vc1 = d1*q1d*dref1;
vb = l1*q1d*dref1;
vc2 = vb + d2*(q2d+q1d)*dref2;

%Accelerations
Ac1 = d1*q1dd*dref1 - d1*q1d^2*ref1;
AB = l1*q1dd*dref1 - l1*q1d^2*ref1;
Ac2 = d2*(q2dd+q1dd)*dref2 - d2*(q1d + q2d)^2*ref2 + AB;

P = g0*dot(ac2,j)*m2 + g0*dot(ac1,j)*m1;
K = 1/2*I1*q1d^2 + 1/2*I2*(q2d+q1d)^2 + 1/2*m1*dot(vc1,vc1) + 1/2*m2*dot(vc2,vc2);

E = P + K;

matlabFunction(E, 'file', 'ComputeEnergy');