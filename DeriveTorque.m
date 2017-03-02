function [ Tc ] = DeriveTorque(M, C, G, acr)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

     
        d1bar = M(1,2)-M(2,2)*M(1,1)/M(1,2);
        h1bar = C(2)-M(2,2)*C(1)/M(1,2);
        phi1bar = G(2)-M(2,2)*G(1)/M(1,2);
        v1 = -acr.kd1*acr.q1d + acr.kp1*(acr.qdes - acr.q1);
        Tc1 = d1bar*v1 + h1bar + phi1bar;
    
        d2bar = M(2,2) - M(2,1)*M(1,2)/M(1,1);
        h2bar = C(2) - M(2,1)*C(1)/M(1,1);
        phi2bar = G(2) - M(2,1)*G(1)/M(1,1);
        v2 = -acr.kd2*acr.q2d + acr.kp2*(acr.qdes - acr.q2);
        Tc2 = d2bar*v2 + h2bar + phi2bar;
    

        matlabFunction(Tc1,'file','/home/dallo/Desktop/URproject/UR/ComputeTorque1');
        matlabFunction(Tc2,'file','/home/dallo/Desktop/URproject/UR/ComputeTorque2');

end

