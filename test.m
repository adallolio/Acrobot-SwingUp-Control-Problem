AcrobotParameters;

% Some values to test the functions
q1 = [0,0,0,0,pi,0, pi/2];
q2 = [0,0,0,0,0,pi, pi/2];
q1d = zeros(1,length(q1));
q2d = zeros(1,length(q1));
%T1 = [0,1,0,1,0,0,0,0];
%T2 = [0,0,1,1,1,1,1,1];
qdes = acr.alpha*atan(q1d);

%q1dd = zeros(1,length(q1));
%q2dd = zeros(1,length(q1));
T1 = zeros(1,length(q1));
T1d = zeros(1,length(q1));
for i =1:length(q1)
	%T1 = ControlTorque1(acr.I1,acr.I2,acr.lc1,acr.lc2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.m1,acr.m2,q1,q2,qdes,q1d,q2d)
    %T1d = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,qdes)

    T2 = ControlTorque2(acr.I1,acr.I2,acr.lc1,acr.lc2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.m1,acr.m2,q1,q2,qdes,q1d,q2d)
    T2d = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,qdes)
end

% Error between the functions results
error_q1dd = T2 - T2d
%error_q2dd = q2dd - Th2dd;