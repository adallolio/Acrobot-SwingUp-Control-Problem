

acr = AcrobotParameters('sym');
[M, C, G] = AcrobotDynamicsMatrices(acr);


d1bar = M(2,1)-M(2,2)\M(1,2)*M(1,1);
h1bar = C(2)-M(2,2)\M(1,2)*C(1);
phi1bar = G(2)-M(2,2)\M(1,2)*G(1);
q1dd = (acr.T2 - h1bar -phi1bar)/d1bar;
%Tc1 = d1bar*v1 + h1bar + phi1bar;
%M(1,2)*q2dd + C(1) + G(1) = -M(1,1)*v1;
q2dd = -(M(1,1)*q1dd+C(1)+G(1));

% Linearization of q1dd
devq1 = diff(q1dd,acr.q1);
devq2 = diff(q1dd,acr.q2);
devT = diff(q1dd,acr.T2);

acr2 = AcrobotParameters('num');
init = [pi/2, 0, 0, 0];
devq1_n = subs(devq1,[acr.m1,acr.m2, acr.g0,acr.l1,acr.lc1,acr.lc2,acr.I1,acr.I2,acr.q1,acr.q2,acr.q1d,acr.q2d],[acr2.m1,acr2.m2,acr2.g0,acr2.l1,acr2.lc1,acr2.lc2,acr2.I1,acr2.I2,init(1),init(2),init(3),init(4)]);