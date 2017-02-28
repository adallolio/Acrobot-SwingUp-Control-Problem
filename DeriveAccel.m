function [] = DeriveAccel(M,C,G,T,acr)


[acr.q1dd,acr.q2dd]=solve([-pinv(M(1,1))*(M(1,2)*acr.q2dd+G(1)+C(1)),-pinv(M(2,2))*(M(2,1)*acr.q1dd+G(2)+C(2))+T],[acr.q1dd,acr.q2dd]);

matlabFunction(acr.q1dd, 'file', 'ComputeAccel1');
matlabFunction(acr.q2dd, 'file', 'ComputeAccel2');

end