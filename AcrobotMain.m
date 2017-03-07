%clear all; close all; clc;

acr = AcrobotParameters('num'); 
% Choose collocated or non-collocated implementation.
acr.controller_type = 'noncollocated'; % Choose: noncollocated, collocated.

% Initial conditions:
init = [-pi/2  0   0   0]';

% Simulation duration
duration = 10;
animationSpeed = 2;

[tarray, zarray, Tc] = ComputeDynamics(init, duration, 10000, acr);
% Computes the Energy
energy = ComputeEnergy(zarray(:,1),zarray(:,4),zarray(:,2),zarray(:,5));
    
% Plot variables
pos1 = zarray(:,1); %for plots
pos2 = zarray(:,4);%angle_normalizer(zarray(:,4)); %for plots
vel1 = zarray(:,2); %for plots
vel2 = zarray(:,5); %for plots
acc1 = zarray(:,3); %for plots
acc2 = zarray(:,6); %for plots
torq = zarray(:,7);
plotvec = [pos1,pos2,vel1,vel2,acc1,acc2];


% By modifying the first two arguments of this functions positions, 
% velocities and accelerations are plotted.
%makeplot('pos1','pos2',tarray,zarray,animationSpeed,Tc,acr,energy,pos1,pos2,vel1,vel2,acc1,acc2);


figure()
subplot(4,1,1); 
plot(tarray,pos1,'b',tarray,pos2,'r');
title('Joints position')
legend('q1','q2')

subplot(4,1,2); 
plot(tarray,vel1,'b',tarray,vel2,'r')
title('Joints Velocity')
legend('q1d','q2d')

subplot(4,1,3); 
plot(tarray,acc1,'b',tarray,acc2,'r')
title('Joints Acceleration')
legend('q1dd','q2dd')

subplot(4,1,4); 
plot(tarray,torq,'r')
title('Torque at second Joint')
legend('Torque')
%{
[tarray_, zarray_, Tc] = ComputeDynamics(init, duration, 2000, acr);
figure(2)
grid on 
hold on 
%plot(tarray,zarray(:,6),'r')
plot(tarray_,zarray_(:,1),'b')  
plot(tarray_,mod(zarray_(:,4),-2*pi),'r') 
hold off
%}

%% QUESTIONS FOR THE TUTTOR
%{
1. Is it correct the fact that we are not using the space state model?
2. Which is better? Time integration or using a sort of ODE?
3. Should we normalize the angle between pi to -pi or 0 to 2*pi or -2*pi to 0?
4. Why the collocated case is not working at all?
%}
