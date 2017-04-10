clear all; close all; clc;

acr = AcrobotParameters('num'); 

SS = load('SS_Matrices.mat');
[~, ~, ~, ~, K] = ComputesLQR(SS.AGeneric, SS.BGeneric);

% Trajectory that the robot follows to arrive to the final configuration
trajectory = deg2rad([-100, -95, -70, -80, -95, -115, -130, -115, -90, -85, -75,  -50, -75, -90, -120, -140, -150, -120, -90, -70, -50, -30, -10, -50, -90, -120, -160, -175, -150, -130 -110, -90 -70, -50, -30, -10, -30, -50, -90, -120, -150, -185, -200, -210, -220, -250, -280]);

% Initial conditions:
init = [-pi/2  0   0   0]';

% Simulation duration
duration = 15;
time_step = 1.0e-03;
animationSpeed = 2;

% Define the time vector of time for the system
time_array = 0:time_step:duration - time_step;
ref = 0.1*sin(2*pi*time_array + 0).*exp((5/duration)*time_array);

% Initializes the position, velocity and acceleration arrays
q1 = [init(1); zeros(length(time_array)-1,1)];
q2 = [init(2); zeros(length(time_array)-1,1)];
q1d = [init(3); zeros(length(time_array)-1,1)];
q2d = [init(4); zeros(length(time_array)-1,1)];
q1dd = zeros(length(time_array),1);
q2dd = zeros(length(time_array),1);
Torque = zeros(length(time_array),1);
    
aux = zeros(length(time_array),1);
control_action = zeros(length(time_array),1);
delta_angle = deg2rad(20);
delta_angle_trajectory = deg2rad(5);
count = 1;
time_test = [];

    for i= 2:1:length(time_array)

        [M,C,G] = AcrobotDynamicsMatrices(acr,[q1(i-1),q2(i-1),q1d(i-1),q2d(i-1)]);
        
        if (q1(i-1) <= acr.goal_trajectory + delta_angle_trajectory && q1(i-1) > acr.goal_trajectory - delta_angle_trajectory && count < length(trajectory))
            if (time_array(i) > 5)
                delta_angle_trajectory = deg2rad(10);
            end
            count = count+1;
            acr.goal_trajectory = trajectory(count);
            time_test = [time_test; time_array(i)];
            disp(strcat('New control Point:  ',num2str(rad2deg(acr.goal_trajectory))));
        end
        
        %if (angle_normalizer(q1(i-1)) < acr.goal + delta_angle && angle_normalizer(q1(i-1)) > acr.goal - delta_angle && angle_normalizer(q2(i-1)) < 2*delta_angle && angle_normalizer(q2(i-1))> -2*delta_angle)
        if (q1(i-1) <= acr.goal_trajectory + delta_angle && q1(i-1) > acr.goal_trajectory - delta_angle && q2d(i-1)>=-6.28 && q2d(i-1)<6.28 && count == length(trajectory))
            acr.internal_controller = 'LQR';
            control_action(i-1) = 1;
        else 
            acr.internal_controller = 'SwingUp';
            control_action(i-1) = -1;
        end
       %acr.internal_controller = 'SwingUp';
      
		M1bar = M(2,1)-M(2,2)\M(1,2)*M(1,1);
        h1bar = C(2)-M(2,2)\M(1,2)*C(1);
        phi1bar = G(2)-M(2,2)\M(1,2)*G(1);
		  
        if strcmp (acr.internal_controller, 'SwingUp')
            v1 = -acr.kd1*q1d(i-1) + acr.kp1*(acr.goal_trajectory - q1(i-1));
            aux(i-1) = ref(i-1);
            Torque(i-1) = M1bar*v1 + h1bar + phi1bar;
        else
            % This the desired value of q1 at equilibrium
            state_vec =[angle_normalizer(q1(i-1))-pi/2, q2(i-1), q1d(i-1), q2d(i-1)]';
            Torque(i-1) = -K*state_vec;
            
         end
        
        % Controls the torque saturation
        if Torque(i-1)>acr.saturation_trajectory_limit
            Torque(i-1) = acr.saturation_trajectory_limit;
        elseif Torque(i-1)<-acr.saturation_trajectory_limit;
            Torque(i-1) = -acr.saturation_trajectory_limit;
        end

        q1dd(i-1) = (M(1,2)*(Torque(i-1)-C(2)-G(2))+M(2,2)*(C(1)+G(1)))/(M(1,2)^2-M(1,1)*M(2,2));
    	q2dd(i-1) = -(M(1,1)*q1dd(i-1)+C(1)+G(1))/(M(1,2));

    	% Joint Velocities
        q1d(i) = q1d(i-1) + time_step*q1dd(i-1);
        q2d(i) = q2d(i-1) + time_step*q2dd(i-1);
        % Joint Positions
        q1(i) = q1(i-1) + q1d(i-1)*time_step;
        q2(i) = q2(i-1) + q2d(i-1)*time_step;
    end 

zarray = [q1 q1d q1dd q2 q2d q2dd Torque, control_action, aux];

pos1 = rad2deg(zarray(:,1)); %for plots
pos2 = rad2deg(zarray(:,4)); %angle_normalizer(zarray(:,4)); %for plots
vel1 = rad2deg(zarray(:,2)); %for plots
vel2 = rad2deg(zarray(:,5)); %for plots
acc1 = rad2deg(zarray(:,3)); %for plots
acc2 = rad2deg(zarray(:,6)); %for plots
torq = zarray(:,7);
control_action = zarray(:,8);

energy = ComputeEnergy(zarray(:,1),zarray(:,4),zarray(:,2),zarray(:,5));

plotvec = [pos1,pos2,vel1,vel2,acc1,acc2];


% By modifying the first two arguments of this functions positions, 
% velocities and accelerations are plotted.
%makeplot('pos1','pos2',time_array,zarray,animationSpeed,Torque,acr,energy,pos1,pos2,vel1,vel2,acc1,acc2);

Plotter

figure()
set(gcf,'color','w');

subplot(4,1,1); 
plot(time_array,pos1,'b',time_array,mod(pos2,360) ,'r');
title('Joints position')
legend('q1','q2')
%ylim([min(min([pos1,pos2]))-100, max(max([pos1,pos2]))+100])
ylabel('deg','FontSize',16)

subplot(4,1,2); 
plot(time_array,vel1,'b',time_array,vel2,'r')
grid
title('Joints Velocity')
legend('q1d','q2d')
ylabel('deg/sec','FontSize',16)

subplot(4,1,3); 
plot(time_array,acc1,'b',time_array,acc2,'r')
grid
title('Joints Acceleration')
legend('q1dd','q2dd')
ylabel('deg/sec^2','FontSize',16)

subplot(4,1,4); 
plot(time_array,torq,'r')
grid
title('Torque at second Joint')
legend('Torque')
xlabel('Time (s)','FontSize',16)
ylabel('Nm','FontSize',16)

%{
figure()
set(gcf,'color','w');

subplot(2,1,1); 
plot(time_array,pos1,'b',time_array,mod(pos2,360) ,'r');
grid
title('Joints position')
legend('q1','q2')
%ylim([min(min([pos1,pos2]))-100, max(max([pos1,pos2]))+100])
ylabel('deg','FontSize',16)

subplot(2,1,2); 
plot(time_array,torq,'r')
grid
title('Torque at second Joint')
legend('Torque')
xlabel('Time (s)','FontSize',16)
ylabel('Nm','FontSize',16)
%}

%{
figure()

subplot(2,1,1)
plot(time_array,aux,'r')
grid
title('Control input v1')
xlim([0 duration])
ylim([min(aux)-1 max(aux)+1])

subplot(2,1,2)
plot(time_array,control_action,'r')
grid
title('Controller type')
legend('Active controller')
xlim([0 duration])
ylim([min(control_action)-1 max(control_action)+1])
xlabel('Time (s)','FontSize',16)
%}