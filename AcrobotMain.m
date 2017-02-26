%% ACROBOT simulation with swing-up control.
% The acrobot is a two-link robot in which only the second joint (elbw) is actuated.

% Paper to refer: "Partial Feedback Linearization of Underactuated Mechanical Systems" by Mark Spong.
% We will implement both kind of controllers: collocated and non-collocated.



%% IMPLEMENTATION

clc; close all; clear all;

% Choose collocated or non-collocated implementation.
controller_type = 'noncollocated'; % Choose: noncollocated, collocated, none
% 1) noncollocated controller is really crazy and can stabilize to any target
% angle! The downside is that it requires basically boundless torque.
% 2) collocated controller does a reasonable "pumping" motion for swing-up.
% The downside is that it can't stabilize without some linear controller to
% keep it at the top.
% 3) none - no controller mode. Play around with the free pendulum.

% Acrobot Parameters

syms q1 q2 q1d q2d q1dd q2dd v tau kd kp real;

% Acrobot parameters
m1 = 1.0;
m2 = 1.0;
I1 = 0.2;
I2 = 1.0;
lc1 = 0.5;
lc2 = 0.5;
l1 = 1.0;
l2 = 1.0;
d1 = l1/2; % Center of mass distance along link 1 from the fixed joint.
d2 = l2/2; % Center of mass distance along link 2 from the fixed joint.
g0 = 9.81;
% Actuator Saturation
saturation_limit = 10000;


T1 = 0;
T2 = 0;

%q = [q1;q2];
%qD = [q1d;q2d];
%qDD = [q1dd;q2dd];

q1 =0; q2 = 0; q1d = 0; q2d = 0;

% [q1, q2, q1d, q2d]
init = [-pi/2    0    0   0]';

% For link 1 linearization (noncollocated):
kd1 = 6.4;
kp1 = 15;
target_1 = pi/2; % Target stabilization angle

% For link 2 linearization (collocated):
alpha = pi/6; % "pumping" angle
kd2 = 200;
kp2 = 2000;

%{
%% Acrobot dynamics
[M,C,G] = AcrobotDynamicsMatrices(m1, m2, I1, I2, lc1, lc2, l1, l2, g0, q1, q2, q1d, q2d);

%% Controllers
if strcmp(controller_type,'noncollocated') 
    % NON-COLLOCATED linearization
    q1des = pi/2;
    Tc = TorqueController1(M, C, G, [kp1, kd1], q1, q1d, q1des);
elseif strcmp(controller_type,'collocated') 
    % COLLOCATED linearization
    alpha = 1;
    q2des = 2*alpha/(pi*atan(q1d));
    Tc = TorqueController2(M, C, G, [kp2, kd2], q2, q2d, q2des);
end

%% Dynamics 
[states] = AcrobotDynamics( M, C, G, [kp1, kd1], controller_type, saturation_limit, [q1, q1d, q2, q2d], pi/2);

%% Phase portrait
%}
Acrobot_PhasePortrait(m1, m2, I1, I2, lc1, lc2, l1, l2, g0, controller_type);


%{


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