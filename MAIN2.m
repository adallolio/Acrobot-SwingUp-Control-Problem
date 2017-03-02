clear all;
close all;
clc;

acr = AcrobotParameters('num'); 
% Choose collocated or non-collocated implementation.
acr.controller_type = 'noncollocated'; % Choose: noncollocated, collocated.
% 1) noncollocated controller is really crazy and can stabilize to any target
% angle! The downside is that it requires basically boundless torque.
% 2) collocated controller does a reasonable "pumping" motion for swing-up.
% The downside is that it can't stabilize without some linear controller to
% keep it at the top.
% 3) none - no controller mode. Play around with the free pendulum.% Choose: noncollocated, collocated, none


% Initial conditions:
init = [-pi/2    0    0   0]';

% Simulation duration
duration = 20;
animationSpeed = 1;

options1 = odeset('AbsTol', 1e-6,'RelTol',1e-6); %Transition from swing up to linear balance controller when conditions met.
[tarray, zarray] = ode15s(@CLsystem, [0 duration], init, options1, acr);


Tc = ones(length(tarray),1);
qdes = acr.goal;



if strcmp(acr.controller_type,'noncollocated') 
	Tc = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4),qdes);
    % NON-COLLOCATED linearization
    %for i = 1:length(tarray)
    %    qdes = acr.goal;
    %	 [M,C,G] = AcrobotDynamicsMatrices(acr,zarray(i,:));
    %    Tc(i) = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,zarray(i,1),zarray(i,3),zarray(i,2),zarray(i,4),qdes);
    %end
elseif strcmp(acr.controller_type,'collocated') 
	Tc = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4),qdes);
    % COLLOCATED linearization
    %for i = 1:length(tarray)
    %    qdes = acr.alpha*atan(zarray(i,3));
    %    %[M,C,G] = AcrobotDynamicsMatrices(acr,zarray(i,:));
    %    %Tc(i) = TorqueController(M, C, G, acr, zarray(i,3), zarray(i,4), qdes);
    %end
end


Tc(Tc>acr.saturation_limit) = acr.saturation_limit;
Tc(Tc<-acr.saturation_limit) = -acr.saturation_limit;


energy = ComputeEnergy(zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4));
makeplot