function [ time_array, states_array, Torque] = ComputeDynamics(init, duration, n_samples, acr)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    delta_t = duration/n_samples;
    time_array = 0:delta_t:duration - delta_t;

    q1 = zeros(length(time_array),1);
    q1(1) = init(1);
    q2 = zeros(length(time_array),1);
    q2(1) = init(2);
    q1d = zeros(length(time_array),1);
    q1d(1) = init(3);
    q2d = zeros(length(time_array),1);
    q2d(1) = init(4);
    Torque = zeros(length(time_array),1);

    for i= 2:1:length(time_array)
        if strcmp(acr.controller_type,'noncollocated')
            qdes = acr.goal;
            Torque(i) = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),qdes);

        elseif strcmp(acr.controller_type,'collocated')
            qdes = acr.alpha*atan(q1d(i-1));
            Torque(i) = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),qdes);
        else
            Torque(i) = 0.0;
        end
        
        % Controls the torque saturation
        if Torque(i)>acr.saturation_limit
            Torque(i) = acr.saturation_limit;
        elseif Torque(i)<-acr.saturation_limit;
            Torque(i) = acr.saturation_limit;
        end

        q1dd = Compute_q1dd(acr.I1,acr.I2,acr.T1,Torque(i),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));
        q2dd = Compute_q2dd(acr.I1,acr.I2,acr.T1,Torque(i),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));

        q1d(i) = q1d(i-1) + delta_t*q1dd;
        q2d(i) = q2d(i-1) + delta_t*q2dd;
        q1(i) = q1(i-1) + q1d(i)*delta_t + delta_t*q1d(i);
        %mod(q1(i-1) + q1d(i)*delta_t + 0.5*q1dd*delta_t^2, 2*pi);
        q2(i) = q2(i-1) + delta_t*q2d(i);
        %mod(q2(i-1) + q2d(i)*delta_t + 0.5*q1dd*delta_t^2, 2*pi);
        
    end

states_array = [q1 q2 q1d q2d];

end

