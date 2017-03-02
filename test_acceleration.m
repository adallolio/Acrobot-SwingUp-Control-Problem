
AcrobotParameters;

% Some values to test the functions
q1 = [0,0,0,0,pi,0, pi/2];
q2 = [0,0,0,0,0,pi, pi/2];
q1d = zeros(1,length(q1));
q2d = zeros(1,length(q1));
T1 = [0,1,0,1,0,0,0,0];
T2 = [0,0,1,1,1,1,1,1];

q1dd = zeros(1,length(q1));
q2dd = zeros(1,length(q1));
Th1dd = zeros(1,length(q1));
Th2dd = zeros(1,length(q1));
for i =1:length(q1)
    q1dd(i) = Compute_q1dd(acr.I1, acr.I2, T1(i), T2(i), acr.g0, acr.l1, acr.lc1, acr.lc2, acr.m1, acr.m2, q1(i),q2(i),q1d(i),q2d(i));
    q2dd(i) = Compute_q2dd(acr.I1, acr.I2, T1(i), T2(i), acr.g0, acr.l1, acr.lc1, acr.lc2, acr.m1, acr.m2, q1(i),q2(i),q1d(i),q2d(i));
    Th1dd(i) = Thdotdot1(acr.I1, acr.I2, T1(i), T2(i), acr.lc1, acr.lc2, acr.g0, acr.l1, acr.m1, acr.m2, q1(i),q2(i),q1d(i),q2d(i));
    Th2dd(i) = Thdotdot2(acr.I1, acr.I2, T1(i), T2(i), acr.lc1, acr.lc2, acr.g0, acr.l1, acr.m1, acr.m2, q1(i),q2(i),q1d(i),q2d(i));
end

% Error between the functions results
error_q1dd = q1dd - Th1dd;
error_q2dd = q2dd - Th2dd;