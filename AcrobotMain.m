clear all;
close all;
clc;

acr = AcrobotParameters('num'); 
% Choose collocated or non-collocated implementation.
acr.controller_type = 'noncollocated'; % Choose: noncollocated, collocated.

acr
% Initial conditions:
init = [-pi/2    0    0   0]';

% Simulation duration
duration = 10;
animationSpeed = 1;

%{
options1 = odeset('AbsTol', 1e-6,'RelTol',1e-6); %Transition from swing up to linear balance controller when conditions met.
[tarray, zarray] = ode15s(@CLsystem, [0 duration], init, options1, acr);


%Tc = ones(length(tarray),1);

if strcmp(acr.controller_type,'noncollocated') 
	qdes = acr.goal;
	Tc = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4),qdes);
elseif strcmp(acr.controller_type,'collocated') 
	qdes = acr.alpha*atan(zarray(:,2));
	Tc = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4),qdes);
end
%}

[tarray, zarray, Tc] = ComputeDynamics(init, duration, 10000, acr);

energy = ComputeEnergy(zarray(:,1),zarray(:,4),zarray(:,2),zarray(:,5));

pos1 = zarray(:,1); %for plots
pos2 = zarray(:,4); %for plots
vel1 = zarray(:,2); %for plots
vel2 = zarray(:,5); %for plots
acc1 = zarray(:,3); %for plots
acc2 = zarray(:,6); %for plots

plotvec = [pos1,pos2,vel1,vel2,acc1,acc2];

%{
figure(1)
grid on 
hold on 
plot(tarray,zarray(:,1),'b')  
plot(tarray,zarray(:,4),'r') 
hold off 
%}

% By modifying the first two arguments of this functions positions, 
% velocities and accelerations are plotted.
makeplot('pos1','pos2',tarray,zarray,animationSpeed,Tc,acr,energy,pos1,pos2,vel1,vel2,acc1,acc2);



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