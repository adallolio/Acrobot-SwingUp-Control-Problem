function Tc2 = TorqueCollocated(I1,I2,g0,kd2,kp2,l1,lc1,lc2,m1,m2,q1,q2,q1d,q2d,q2des)
%TORQUECOLLOCATED
%    TC2 = TORQUECOLLOCATED(I1,I2,G0,KD2,KP2,L1,LC1,LC2,M1,M2,Q1,Q2,Q1D,Q2D,Q2DES)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    07-Mar-2017 12:04:50

t2 = lc2.^2;
t3 = m2.*t2;
t5 = cos(q2);
t8 = l1.*lc2.*m2.*t5;
t4 = I2+t3+t8;
t6 = q1+q2;
t7 = cos(t6);
t9 = lc1.^2;
t10 = m1.*t9;
t11 = l1.^2;
t12 = t2+t11;
t13 = m2.*t12;
t14 = l1.*lc2.*m2.*t5.*2.0;
t15 = I1+I2+t10+t13+t14;
t16 = 1.0./t15;
t17 = sin(q2);
Tc2 = -(kd2.*q2d+kp2.*(q2-q2des)).*(I2+t3-t4.^2.*t16)+g0.*lc2.*m2.*t7-g0.*t4.*t16.*(cos(q1).*(l1.*m2+lc1.*m1)+lc2.*m2.*t7)+l1.*lc2.*m2.*q1d.^2.*t17+l1.*lc2.*m2.*t4.*t16.*t17.*(q1d.*q2d.*2.0+q2d.^2);
