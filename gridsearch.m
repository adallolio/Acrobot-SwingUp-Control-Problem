%% GRID SEARCH FOR TUNING

clear all; close all; clc;

acr = AcrobotParameters('num'); 

SS = load('SS_Matrices.mat');
[~, ~, ~, ~, K] = ComputesLQR(SS.AGeneric, SS.BGeneric);

% Initial conditions:
init = [-pi/2  0   0   0]';

% Simulation duration
duration = 15;
time_step = 1.0e-03;
delta_angle = deg2rad(10);
time_array = 0:time_step:duration - time_step;

kappa=[];
avg=[];

for k=0.25:0.25:15
	acr.kd1 = k;
    for j=0.25:0.25:15
		acr.kp1 = j;
		q1 = [init(1); zeros(length(time_array-1),1)];
    	q2 = [init(2); zeros(length(time_array-1),1)];
    	q1d = [init(3); zeros(length(time_array-1),1)];
    	q2d = [init(4); zeros(length(time_array-1),1)];

    	Torque = zeros(length(time_array),1);

    	control_action = zeros(length(time_array),1);

        for i= 2:1:length(time_array)
            [M,C,G] = AcrobotDynamicsMatrices(acr,[q1(i-1),q2(i-1),q1d(i-1),q2d(i-1)]);
            
            %if (angle_normalizer(q1(i-1)) < acr.goal + delta_angle && angle_normalizer(q1(i-1)) > acr.goal - delta_angle && angle_normalizer(q2(i-1)) < 2*delta_angle && angle_normalizer(q2(i-1))> -2*delta_angle)
            if (angle_normalizer(q1(i-1)) <= acr.goal + delta_angle && angle_normalizer(q1(i-1)) > acr.goal - delta_angle && q2d(i-1)>=-6.28 && q2d(i-1)<6.28)
                acr.internal_controller = 'LQR';
                control_action(i-1) = 1;
            else 
                acr.internal_controller = 'SwingUp';
                control_action(i-1) = -1;
            end
            %acr.internal_controller = 'SwingUp';

            %%{
            M1bar = M(2,1)-M(2,2)\M(1,2)*M(1,1);
            h1bar = C(2)-M(2,2)\M(1,2)*C(1);
            phi1bar = G(2)-M(2,2)\M(1,2)*G(1);
            %%}
            
            %{
            M1bar = M(2,1)-M(2,2)*(1/M(1,2))*M(1,1);
            h1bar = C(2)-M(2,2)*(1/M(1,2))*C(1);
            phi1bar = G(2)-M(2,2)*(1/M(1,2))*G(1);
            %}

            if strcmp (acr.internal_controller, 'SwingUp')
                v1 = -acr.kd1*q1d(i-1) + acr.kp1*(pi/2 - q1(i-1));
                aux(i-1) = v1;
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
                Torque(i-1) = -acr.saturation_limit;
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

        endlimit = 15000;
        startlimit = 10000;
        avg(1) = mean(q1(startlimit:endlimit));
        avg(2) = mean(q1d(startlimit:endlimit));
        avg(3) = mean(q2d(startlimit:endlimit));

        if(avg(2)<=2*pi && avg(2)>-2*pi && avg(3)<=2*pi && avg(3)>-2*pi )
            if(max(q1d(startlimit:endlimit))<=2*pi && min(q1d(startlimit:endlimit))>-2*pi && max(q2d(startlimit:endlimit))<=2*pi && min(q2d(startlimit:endlimit))>-2*pi )
                if(angle_normalizer(avg(1)) <= acr.goal + delta_angle && angle_normalizer(avg(1)) > acr.goal - delta_angle)
                    kappa = [kappa;[k,j]]; % kd, kp
                end
            end
        end
    end
end
save('kappa.mat','kappa')

