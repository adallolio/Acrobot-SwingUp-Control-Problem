function [ states ] = AcrobotDynamics(M,C,G,K,sat,Var,qdes)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

kp = K(1);
kd = K(2);

q1 = Var(1);
q2 = Var(2);
q1d = Var(3);
q2d = Var(4);

if strcmp(controller_type,'noncollocated') 
    % NON-COLLOCATED case: Link 1 linearization
    q1des = qdes;
    Tc = TorqueController1(M, C, G, K, q1, q1d, q1des);
    
    % Accelerations
    % NOTE: I don't know from where to obtain q1d2
    q1d2 = 0.0;
    q2d2 = -((C(1)+G(1))+M(1,1)*(-kp*(q1-q1des)-kd*q1d))/M(1,2);
   
else
    % COLLOCATED case: Link 2 linearization
    q2des = qdes;
    Tc = TorqueController2(M, C, G, K, q2, q2d, q2des);
    % Accelerations
    % NOTE: I don't know from where to obtain q2d2
    q1d2 = -((C(1)+G(1))+M(1,2)*(-kp*(q2-q2des)-kd*q2d))/M(1,1);
    q2d2 = 0.0;
    
end

% Actuator saturation, clamp torque if outside max.
if Tc>sat
    Tc = sat;
elseif Tc<-sat;
    Tc = sat;
end

states = [q1d, q2d, q1d2, q2d2];

end

