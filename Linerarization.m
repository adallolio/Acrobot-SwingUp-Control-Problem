acr = AcrobotParameters('sym');
%A = zeros(4,4);

[M, C, G] = AcrobotDynamicsMatrices(acr);

eq1 = M(1,1)*acr.q1dd+M(1,2)*acr.q2dd + C(1) + G(1);
eq2 = M(2,2)*acr.q1dd+M(2,1)*acr.q2dd + C(2) + G(2)-acr.T2;
[q1dd,q2dd] = solve([eq1,eq2],[acr.q1dd,acr.q2dd]);


d1bar = M(2,1)-M(2,2)\M(1,2)*M(1,1);
h1bar = C(2)-M(2,2)\M(1,2)*C(1);
phi1bar = G(2)-M(2,2)\M(1,2)*G(1);
q1dd_juan = (acr.T2 - h1bar -phi1bar)/d1bar;
%Tc1 = d1bar*v1 + h1bar + phi1bar;
%M(1,2)*q2dd + C(1) + G(1) = -M(1,1)*v1;
q2dd_juan = -(M(1,1)*q1dd+C(1)+G(1));




Adallo=jacobian([acr.q1d,acr.q2d,q1dd,q2dd],[acr.q1,acr.q2,acr.q1d,acr.q2d]);
Bdallo=jacobian([acr.q1d,acr.q2d,q1dd,q2dd],acr.T2);

Ajuan=jacobian([acr.q1d,acr.q2d,q1dd_juan,q2dd_juan],[acr.q1,acr.q2,acr.q1d,acr.q2d]);
Bjuan=jacobian([acr.q1d,acr.q2d,q1dd_juan,q2dd_juan],acr.T2);

acr2 = AcrobotParameters('num');
init = [pi/2, 0, 0, 0, 0];
Adallo(3,1) = subs(Adallo(3,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Adallo(3,2) = subs(Adallo(3,2),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Adallo(3,3) = subs(Adallo(3,3),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Adallo(3,4) = subs(Adallo(3,4),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Adallo(4,1) = subs(Adallo(4,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Adallo(4,2) = subs(Adallo(4,2),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Adallo(4,3) = subs(Adallo(4,3),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Adallo(4,4) = subs(Adallo(4,4),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);

Bdallo(3,1) = subs(Bdallo(3,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Bdallo(4,1) = subs(Bdallo(4,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);

Adallo
Bdallo 

Ajuan(3,1) = subs(Ajuan(3,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);
Ajuan(3,2) = subs(Ajuan(3,2),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);
Ajuan(3,3) = subs(Ajuan(3,3),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);
Ajuan(3,4) = subs(Ajuan(3,4),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);
Ajuan(4,1) = subs(Ajuan(4,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);
Ajuan(4,2) = subs(Ajuan(4,2),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);
Ajuan(4,3) = subs(Ajuan(4,3),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);
Ajuan(4,4) = subs(Ajuan(4,4),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);

Bjuan(3,1) = subs(Bjuan(3,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);
Bjuan(4,1) = subs(Bjuan(4,1),[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d,acr.T2],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4),init(5)]);

Ajuan
Bjuan
