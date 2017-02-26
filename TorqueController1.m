function [ Tc ] = TorqueController1(M, C, G, K, q1, q1d, q1des)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

    kp = K(1);
    kd = K(2);
    
    d1bar = M(1,2)-M(2,2)*M(1,1)/M(1,2);
    h1bar = C(2)-M(2,2)*C(1)/M(1,2);
    phi1bar = G(2)-M(2,2)*G(1)/M(1,2);
    v1 = -kd*q1d + kp*(q1des - q1);
    Tc = d1bar*v1 + h1bar + phi1bar;

end

