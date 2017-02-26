function [ Tc ] = TorqueController(M, C, G, q, qd, qdes,acr)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
    
if strcmp(acr.controller_type,'noncollocated') 
    d1bar = M(1,2)-M(2,2)*M(1,1)/M(1,2);
    h1bar = C(2)-M(2,2)*C(1)/M(1,2);
    phi1bar = G(2)-M(2,2)*G(1)/M(1,2);
    v1 = -acr.kd1*qd + acr.kp1*(qdes - q);
    Tc = d1bar*v1 + h1bar + phi1bar;
else
    d2bar = M(2,2) - M(2,1)*M(1,2)/M(1,1);
    h2bar = C(2) - M(2,1)*C(1)/M(1,1);
    phi2bar = G(2) - M(2,1)*G(1)/M(1,1);
    v2 = acr.kd2*-qd + acr.kp2*(qdes - q);
    Tc = d2bar*v2 + h2bar + phi2bar;

end

end

