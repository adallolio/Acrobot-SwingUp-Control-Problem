%% ACROBOT simulation with swing-up control.
% The acrobot is a two-link robot in which only the second joint (elbw) is actuated.

% Paper to refer: "Partial Feedback Linearization of Underactuated Mechanical Systems" by Mark Spong.
% We will implement both kind of controllers: collocated and non-collocated.



%% IMPLEMENTATION

clc; close all; clear all;

% Choose collocated or non-collocated implementation.
acr.controller_type = 'noncollocated'; % Choose: noncollocated, collocated, none
% 1) noncollocated controller is really crazy and can stabilize to any target
% angle! The downside is that it requires basically boundless torque.
% 2) collocated controller does a reasonable "pumping" motion for swing-up.
% The downside is that it can't stabilize without some linear controller to
% keep it at the top.
% 3) none - no controller mode. Play around with the free pendulum.

% Acrobot Parameters

%syms q1 q2 q1d q2d q1dd q2dd v tau kd kp real;

% Acrobot parameters
acr.m1 = 1.0;
acr.m2 = 1.0;
acr.I1 = 0.2;
acr.I2 = 1.0;
acr.lc1 = 0.5;
acr.lc2 = 0.5;
acr.l1 = 1.0;
acr.l2 = 1.0;
acr.d1 = acr.l1/2; % Center of mass distance along link 1 from the fixed joint.
acr.d2 = acr.l2/2; % Center of mass distance along link 2 from the fixed joint.
acr.g0 = 9.81;
% Actuator Saturation
acr.saturation_limit = 10000;
duration = 10;
animationSpeed = 1;
acr.T1 = 0;
acr.T2 = 0;

%q = [q1;q2];
%qD = [q1d;q2d];
%qDD = [q1dd;q2dd];

% [q1, q2, q1d, q2d]
init = [-pi/2    0    0   0]';

% For link 1 linearization (noncollocated):
acr.goal = pi/2;
acr.kd1 = 6.4;
acr.kp1 = 15;

% For link 2 linearization (collocated):
acr.alpha = pi/6; % "pumping" angle
acr.kd2 = 200;
acr.kp2 = 2000;


%% Acrobot dynamics
%[M,C,G] = AcrobotDynamicsMatrices(acr,init);

options1 = odeset('AbsTol', 1e-6,'RelTol',1e-3);
[tarray, zarray] = ode15s(@CLsystem, [0 duration], init, options1, acr);



%% Controllers

Tc = ones(length(tarray),1);
if strcmp(acr.controller_type,'noncollocated') 
    % NON-COLLOCATED linearization
    for i = 1:length(tarray)
        q1des = acr.goal;
        [M,C,G] = AcrobotDynamicsMatrices(acr,zarray(i,:));
        Tc(i) = TorqueController(M, C, G, zarray(i,1), zarray(i,3), q1des, acr);
    end
elseif strcmp(acr.controller_type,'collocated') 
    % COLLOCATED linearization
    for i = 1:length(tarray)
        q2des = acr.alpha*atan(zarray(i,3));
        [M,C,G] = AcrobotDynamicsMatrices(acr,zarray(i,:));
        Tc(i) = TorqueController(M, C, G, zarray(i,2), zarray(i,4), q2des, acr);
    end
end

Tc(Tc>acr.saturation_limit) = acr.saturation_limit;
Tc(Tc<-acr.saturation_limit) = -acr.saturation_limit;


energy = TotEnergy(acr.I1,acr.I2,acr.d1,acr.d2,acr.g0,acr.l1,acr.m1,acr.m2,zarray(:,1),zarray(:,3),zarray(:,2),zarray(:,4));

Plotter

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