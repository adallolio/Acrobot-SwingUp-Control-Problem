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
    aux = zeros(length(time_array),1);

    


    %internal_controller = 'SwingUp';
    for i= 2:1:length(time_array)

        [M,C,G] = AcrobotDynamicsMatrices(acr,[q1(i-1),q2(i-1),q1d(i-1),q2d(i-1)]);
        %internal_controller = 'SwingUp';
        aux(i-1) = M(1,1); 
 
        if (angle_normalizer(q1(i-1)) < 1.7453 && angle_normalizer(q1(i-1)) > 1.3963 && angle_normalizer(q2(i-1)) < 0.05 && angle_normalizer(q2(i-1))> -0.05)
            internal_controller = 'LQR'
        else 
            internal_controller = 'SwingUp';
        end
        internal_controller = 'SwingUp';

        % Select the type of Strategy for Torque
        if strcmp(acr.controller_type,'noncollocated')
            q1des = acr.goal;
            Torque(i) = TorqueNonColloacted(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),q1des);

        elseif strcmp(acr.controller_type,'collocated')
            %q2des = (2*acr.alpha/pi)*atan(q1d(i-1)));
            q2des = acr.alpha*atan(q1d(i-1));
            Torque(i) = TorqueCollocated(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),q2des);
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
            q1dd(i-1) = AccelerationJoint1(acr.I1,acr.I2,Torque(i),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));
            q2dd(i-1) = AccelerationJoint2(acr.I1,acr.I2,Torque(i),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));
            % Joint Velocities
            q1d(i) = q1d(i-1) + delta_t*q1dd(i-1);
            q2d(i) = q2d(i-1) + delta_t*q2dd(i-1);
            % Joint Positions
            q1(i) = q1(i-1) + q1d(i)*delta_t;
            q2(i) = q2(i-1) + q2d(i)*delta_t;
        elseif strcmp(internal_controller,'LQR')
            
            if strcmp(acr.controller_type,'noncollocated')
                qdes = acr.goal;
            else
                qdes = 2*acr.alpha/(pi*atan(q1d(i-1)));
            end
            
            
            % Joint Accelerations
            q1dd(i) = (-553.4018)*(q1(i-1)-qdes)+(-401.9784)*q2(i-1)+(-98.9437)*q1d(i-1)+(-55.0397)*q2d(i-1)+(0.6100)*Torque(i);
            q2dd(i) = (1.2314e+03)*(q1(i-1)-qdes)+(889.0061)*q2(i-1)+(231.9501)*q1d(i-1)+(129.0274)*q2d(i-1)+(-1.4300)*Torque(i);
            % Joint Velocities
            q1d(i) = q1d(i-1)+delta_t*q1dd(i);
            q2d(i) = q2d(i-1)+delta_t*q2dd(i);
            % Joint Positions
            q1(i) = q1(i-1)+delta_t*q1d(i);
            q2(i) = q2(i-1) + q2d(i)*delta_t;
        end

        
    end

states_array = [q1 q1d q1dd q2 q2d q2dd Torque,aux];

end

