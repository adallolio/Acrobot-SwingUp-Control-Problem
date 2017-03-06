function [ q1dd, q2dd] = DeriveJointAcceleration( M,C,G,acr )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % From the formula obtained
    q2dd = simplify((M(1,1)*(acr.T2-C(2)-G(2))+M(1,2)*(C(1)+G(1)))/(M(1,1)*M(2,2)-M(1,2)^2));
    q1dd = simplify(-(M(1,2)*q2dd+C(1)+G(1))/(M(1,1)));
    
    matlabFunction(q1dd, 'file', 'Compute_q1dd_new');
    matlabFunction(q2dd, 'file', 'Compute_q2dd_new');


end

