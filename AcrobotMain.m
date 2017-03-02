%% ACROBOT simulation with swing-up control.
% The acrobot is a two-link robot in which only the second joint (elbw) is actuated.

% Paper to refer: "Partial Feedback Linearization of Underactuated Mechanical Systems" by Mark Spong.
% We will implement both kind of controllers: collocated and non-collocated.



%% IMPLEMENTATION

clc; close all; clear all;

%% Animation Parameters
duration = 2;
animationSpeed = 1;

AcrobotParameters;

%{
% Input Torque, Accelerations and system Energy derivation in symbolic form
display('Acrobot Dynamics')
[M,C,G] = AcrobotDynamicsMatrices(acr)
pause
display('Input Torque')
if strcmp(acr.controller_type,'noncollocated')
    qdes = acr.goal;
    T = TorqueController(M, C, G, acr)
else 
    qdes = acr.alpha*atan(q1d);
    T = TorqueController(M, C, G, acr)
end
pause
%DeriveAccel(M,C,G,acr);  never to be used again, only once!!!
display('System Energy')
[E] = DeriveEnAcc(acr)
%}

% [q1, q2, q1d, q2d] è giusto??
init = [-pi/2    0    0   0]';


%% Acrobot dynamics

options1 = odeset('AbsTol', 1e-6,'RelTol',1e-6);
[tarray, zarray] = ode15s(@CLsystem, [0 duration], init, options1, acr);



%% Controllers

Tc = ones(length(tarray),1);
if strcmp(acr.controller_type,'noncollocated') 
    % NON-COLLOCATED linearization
    for i = 1:length(tarray)
        qdes = acr.goal;
        [M,C,G] = AcrobotDynamicsMatrices(acr,zarray(i,:));
        Tc(i) = TorqueController(M, C, G, acr, zarray(i,1), zarray(i,2), qdes);
    end
elseif strcmp(acr.controller_type,'collocated') 
    % COLLOCATED linearization
    for i = 1:length(tarray)
        qdes = acr.alpha*atan(zarray(i,3));
        [M,C,G] = AcrobotDynamicsMatrices(acr,zarray(i,:));
        Tc(i) = TorqueController(M, C, G, acr, zarray(i,3), zarray(i,4), qdes);
    end
end

Tc(Tc>acr.saturation_limit) = acr.saturation_limit;
Tc(Tc<-acr.saturation_limit) = -acr.saturation_limit;

%{

energy = ComputeEnergy(zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4));

makeplot
%}
%{

%% Dynamics 
[states] = AcrobotDynamics(M,C,G,[kp1,kd1],saturation_limit,[q1,q1d,q2,q2d],pi/2);

%% Phase portrait

Acrobot_PhasePortrait(m1,m2,I1,I2,lc1,lc2,l1,l2,g0);


%Assemble the state vector derivatives.
zdot = [q1d
    q1dd
    q2d
    q2dd
    ];




%% Animation
duration = 20;
animationSpeed = 0.5;
samples=[];
i=1;

while (duration~=0)
	duration = duration-0.05;
    samples(i)=duration;
    i=i+1;
end
%}
