function [ Tc ] = TorqueController(M, C, G, acr, q, qd, qdes)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
if nargin == 4
    if strcmp(acr.controller_type,'noncollocated') 
        d1bar = M(1,2)-M(2,2)*M(1,1)/M(1,2);
        h1bar = C(2)-M(2,2)*C(1)/M(1,2);
        phi1bar = G(2)-M(2,2)*G(1)/M(1,2);
        v1 = -acr.kd1*acr.q1d + acr.kp1*(acr.qdes - acr.q1);
        Tc = d1bar*v1 + h1bar + phi1bar;
    else
        d2bar = M(2,2) - M(2,1)*M(1,2)/M(1,1);
        h2bar = C(2) - M(2,1)*C(1)/M(1,1);
        phi2bar = G(2) - M(2,1)*G(1)/M(1,1);
        v2 = -acr.kd2*acr.q2d + acr.kp2*(acr.qdes - acr.q2);
        Tc = d2bar*v2 + h2bar + phi2bar;
    end
else
    if strcmp(acr.controller_type,'noncollocated')
        q1 = qd;
        q1d = qd;
        d1bar = M(1,2)-M(2,2)*M(1,1)/M(1,2);
        h1bar = C(2)-M(2,2)*C(1)/M(1,2);
        phi1bar = G(2)-M(2,2)*G(1)/M(1,2);
        v1 = -acr.kd1*q1d + acr.kp1*(qdes - q1);
        Tc = d1bar*v1 + h1bar + phi1bar;
    else
        q2 = qd;
        q2d = qd;
        d2bar = M(2,2) - M(2,1)*M(1,2)/M(1,1);
        h2bar = C(2) - M(2,1)*C(1)/M(1,1);
        phi2bar = G(2) - M(2,1)*G(1)/M(1,1);
        v2 = -acr.kd2*q2d + acr.kp2*(qdes - q2);
        Tc = d2bar*v2 + h2bar + phi2bar;
    end
end

end

