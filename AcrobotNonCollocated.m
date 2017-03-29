clear all; close all; clc;

acr = AcrobotParameters('num'); 

SS = load('SS_Matrices.mat');
[~, ~, ~, ~, K] = ComputesLQR(SS.AGeneric, SS.BGeneric);

% Initial conditions:
init = [-pi/2+0.1  0   0   0]';

% Simulation duration
duration = 10;
time_step = 1.0e-03;
animationSpeed = 2;

% Define the time vector of time for the system
    time_array = 0:time_step:duration - time_step;

    % Initializes the position, velocity and acceleration arrays
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
    control_action = zeros(length(time_array),1);
    delta_angle = deg2rad(10);
    
    %acr.internal_controller = 'SwingUp';
    
    for i= 2:1:length(time_array)

        [M,C,G] = AcrobotDynamicsMatrices(acr,[q1(i-1),q2(i-1),q1d(i-1),q2d(i-1)]);
        
        aux(i-1) = 1/M(1,1);
        
        %if (angle_normalizer(q1(i-1)) < acr.goal + delta_angle && angle_normalizer(q1(i-1)) > acr.goal - delta_angle && angle_normalizer(q2(i-1)) < 2*delta_angle && angle_normalizer(q2(i-1))> -2*delta_angle)
        if (angle_normalizer(q1(i-1)) < acr.goal + delta_angle && angle_normalizer(q1(i-1)) > acr.goal - delta_angle )
            acr.internal_controller = 'LQR';
            control_action(i-1) = 1;
        else 
            acr.internal_controller = 'SwingUp';
            control_action(i-1) = -1;
        end
        acr.internal_controller = 'SwingUp';
      

		M1bar = M(2,1)-M(2,2)\M(1,2)*M(1,1);
        h1bar = C(2)-M(2,2)\M(1,2)*C(1);
        phi1bar = G(2)-M(2,2)\M(1,2)*G(1);
		
		%{
        M1bar = M(2,1) - (M(2,2)*M(1,1))/M(1,2);
        h1bar = C(2) - (M(2,2)*C(1))/M(1,2);
        phi1bar = G(2) - (M(2,2)*G(1))/M(1,2);
		%}

        if strcmp (acr.internal_controller, 'SwingUp')
            v1 = -acr.kd1*q1d(i-1) + acr.kp1*(pi/2 - q1(i-1));
            Torque(i-1) = M1bar*v1 + h1bar + phi1bar;
        else
            % This the desired value of q1 at equilibrium
            state_vec =[q1(i-1)-pi/2, q2(i-1), q1d(i-1), q2d(i-1)]';
            Torque(i-1) = -K*state_vec;
            
         end
        
        % Controls the torque saturation
        if Torque(i-1)>acr.saturation_limit
            Torque(i-1) = acr.saturation_limit;
        elseif Torque(i-1)<-acr.saturation_limit;
            Torque(i-1) = acr.saturation_limit;
        end

        q2dd(i-1) = M(1,1)*(Torque(i-1)-C(2)-G(2))+M(1,2)*(C(1)+G(1))/(M(1,1)*M(2,2)-M(1,2)^2);
    	q1dd(i-1) = -(M(1,2)*q2dd(i-1)+C(1)+G(1))/(M(1,1));

    	% Joint Velocities
        q1d(i) = q1d(i-1) + time_step*q1dd(i-1);
        q2d(i) = q2d(i-1) + time_step*q2dd(i-1);
        % Joint Positions
        q1(i) = q1(i-1) + q1d(i-1)*time_step;
        q2(i) = q2(i-1) + q2d(i-1)*time_step;
    end
        

zarray = [q1 q1d q1dd q2 q2d q2dd Torque, control_action, aux];

pos1 = zarray(:,1); %for plots
pos2 = zarray(:,4);%angle_normalizer(zarray(:,4)); %for plots
vel1 = zarray(:,2); %for plots
vel2 = zarray(:,5); %for plots
acc1 = zarray(:,3); %for plots
acc2 = zarray(:,6); %for plots
torq = zarray(:,7);
control_action = zarray(:,8);

energy = ComputeEnergy(zarray(:,1),zarray(:,4),zarray(:,2),zarray(:,5));

plotvec = [pos1,pos2,vel1,vel2,acc1,acc2];


% By modifying the first two arguments of this functions positions, 
% velocities and accelerations are plotted.
%makeplot('pos1','pos2',time_array,zarray,animationSpeed,Torque,acr,energy,pos1,pos2,vel1,vel2,acc1,acc2);

%Plotter

figure()
subplot(4,1,1); 
plot(time_array,mod(pos1,2*pi),'b',time_array,mod(pos2,2*pi) ,'r');
title('Joints position')
legend('q1','q2')
%rad2deg(mod(pos1,2*pi))
%rad2deg(mod(pos2,2*pi))

subplot(4,1,2); 
plot(time_array,vel1*57.2957795130824,'b',time_array,vel2*57.2957795130824,'r')
title('Joints Velocity')
legend('q1d','q2d')

subplot(4,1,3); 
plot(time_array,acc1,'b',time_array,acc2,'r')
title('Joints Acceleration')
legend('q1dd','q2dd')

subplot(4,1,4); 
plot(time_array,torq,'r')
title('Torque at second Joint')
legend('Torque')

figure()
plot(time_array,control_action,'r')
title('Active error')
legend('Active controller')

figure()
plot(time_array,aux,'r')
title('M12')
legend('M12')