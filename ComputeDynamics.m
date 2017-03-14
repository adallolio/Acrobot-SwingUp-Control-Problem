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

        [M,~,~] = AcrobotDynamicsMatrices(acr,[q1(i-1),q2(i-1),q1d(i-1),q2d(i-1)]);
        %internal_controller = 'SwingUp';
        aux(i-1) = det(M); 
 
        if (angle_normalizer(q1(i-1)) < 1.7453 && angle_normalizer(q1(i-1)) > 1.3963 && angle_normalizer(q2(i-1)) < 0.1745 && angle_normalizer(q2(i-1))> -0.1745)
            internal_controller = 'LQR';
            q1d(i-1);
            q2d(i-1);
        else 
            internal_controller = 'SwingUp';
        end
        %internal_controller = 'SwingUp';

        % Select the type of Strategy for Torque
        if strcmp(acr.controller_type,'noncollocated')
            q1des = acr.goal;
            Torque(i-1) = TorqueNonColloacted(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),q1des);

        elseif strcmp(acr.controller_type,'collocated')
            %q2des = 2*acr.alpha/(pi*atan(q1d(i-1)));
            q2des = acr.alpha*atan(q1d(i-1));
            Torque(i-1) = TorqueCollocated(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1),q2des);
        else
            Torque(i-1) = 0.0;
        end
        
        % Controls the torque saturation
        if Torque(i-1)>acr.saturation_limit
            Torque(i-1) = acr.saturation_limit;
        elseif Torque(i-1)<-acr.saturation_limit;
            Torque(i-1) = acr.saturation_limit;
        end

        % Computes accelerations, velocities and positions
        if strcmp(internal_controller,'SwingUp')
            % Joint Accelerations
            q1dd(i-1) = AccelerationJoint1(acr.I1,acr.I2,Torque(i-1),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));
            q2dd(i-1) = AccelerationJoint2(acr.I1,acr.I2,Torque(i-1),acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1(i-1),q2(i-1),q1d(i-1),q2d(i-1));
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
                qdes = acr.alpha*atan(q1d(i-1));
            end
            
            %dallo

        
    

            % Joint Accelerations
    
            q1dd(i) = (-342.1693)*(q1(i-1)-qdes)+(-93.4274)*q2(i-1)+(-134.5067)*q1d(i-1)+(-43.8441)*q2d(i-1)+(-2.0645)*Torque(i-1);
            q2dd(i) = (941.0351)*(q1(i-1)-qdes)+(256.5265)*q2(i-1)+(362.7871)*q1d(i-1)+(118.2550)*q2d(i-1)+(5.5683)*Torque(i-1);
 
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

