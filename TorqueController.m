function [ Tc ] = TorqueController1(M, C, G, K, q, qd, qdes)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

    kp = K(1);
    kd = K(2);
    
if strcmp(controller_type,'noncollocated') 
    d1bar = M(1,2)-M(2,2)*M(1,1)/M(1,2);
    h1bar = C(2)-M(2,2)*C(1)/M(1,2);
    phi1bar = G(2)-M(2,2)*G(1)/M(1,2);
    v1 = -kd*qd + kp*(qdes - q);
    Tc = d1bar*v1 + h1bar + phi1bar;
else
    d2bar = M(2,2) - M(2,1)*M(1,2)/M(1,1);
    h2bar = C(2) - M(2,1)*C(1)/M(1,1);
    phi2bar = G(2) - M(2,1)*G(1)/M(1,1);
    v2 = kd*-qd + kp*(qdes - q);
    Tc = d2bar*v2 + h2bar + phi2bar;

end

end

