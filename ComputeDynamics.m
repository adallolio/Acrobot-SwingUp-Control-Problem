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
    q1dd = zeros(length(time_array),1);
    q1d(1) = init(3);
    q2d = zeros(length(time_array),1);
    q2dd = zeros(length(time_array),1);
    q2d(1) = init(4);
    Torque = zeros(length(time_array),1);

    for i= 2:1:length(time_array)
 
        if (q1(i-1) < 1.7453 && q1(i-1) > 1.3963 && q2(i-1) < 0.05 && q2(i-1)> -0.05)
            internal_controller = 'LQR';
            %disp('HOlaaa')
        else 
            internal_controller = 'SwingUp';
        end
        
        % Select the type of Strategy for Torque
        if strcmp(acr.controller_type,'noncollocated')
            qdes = acr.goal;
            Torque(i) = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),qdes);

        elseif strcmp(acr.controller_type,'collocated')
            qdes = acr.alpha*atan(q1d(i-1));
            Torque(i) = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),pi/2);
        else
            Torque(i) = 0.0;
        end
        
        % Controls the torque saturation
        if Torque(i)>acr.saturation_limit
            Torque(i) = acr.saturation_limit;
        elseif Torque(i)<-acr.saturation_limit;
            Torque(i) = acr.saturation_limit;
        end

        % Computes accelerations, velocities and positions
        if strcmp(internal_controller,'SwingUp')
            % Joint Accelerations
            q1dd(i) = Compute_q1dd(acr.I1,acr.I2,acr.T1,Torque(i),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));
            q2dd(i) = Compute_q2dd(acr.I1,acr.I2,acr.T1,Torque(i),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));
            % Joint Velocities
            q1d(i) = q1d(i-1) + delta_t*q1dd(i);
            q2d(i) = q2d(i-1) + delta_t*q2dd(i);
            % Joint Positions
            %q1(i) = q1(i-1) + q1d(i)*delta_t + 0.5*q1dd(i)*delta_t^2;
            q1(i) = q1(i-1) + q1d(i)*delta_t;
            %q2(i) = q2(i-1) + q2d(i)*delta_t + 0.5*q1dd(i)*delta_t^2;
            q2(i) = mod(q2(i-1) + q2d(i)*delta_t,2*pi);%angle_normalizer(q2(i-1) + q2d(i)*delta_t);
        elseif strcmp(internal_controller,'LQR')
            % Joint Accelerations
            q1dd(i) = (-553.4018)*(q1(i-1)-qdes)+(-401.9784)*q2(i-1)+(-98.9437)*q1d(i-1)+(-55.0397)*q2d(i-1)+(0.6100)*Torque(i);
            q2dd(i) = (1.2314e+03)*(q1(i-1)-qdes)+(889.0061)*q2(i-1)+(231.9501)*q1d(i-1)+(129.0274)*q2d(i-1)+(-1.4300)*Torque(i);
            % Joint Velocities
            q1d(i) = q1d(i-1)+delta_t*q1dd(i);
            q2d(i) = q2d(i-1)+delta_t*q2dd(i);
            % Joint Positions
            q1(i) = q1(i-1)+delta_t*q1d(i);
            q2(i) = mod(q2(i-1)+delta_t*q2d(i),2*pi);%angle_normalizer(q2(i-1)+delta_t*q2d(i));
        end

        
    end

states_array = [q1 q1d q1dd q2 q2d q2dd];

end

