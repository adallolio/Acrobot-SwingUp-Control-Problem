%% 	Acrobot Parameters
%	This file contains the acrobot parameters, saved into the acr structure.
%%  

% Choose collocated or non-collocated implementation.
acr.controller_type = 'noncollocated'; % Choose: noncollocated, collocated, none
% 1) noncollocated controller is really crazy and can stabilize to any target
% angle! The downside is that it requires basically boundless torque.
% 2) collocated controller does a reasonable "pumping" motion for swing-up.
% The downside is that it can't stabilize without some linear controller to
% keep it at the top.
% 3) none - no controller mode. Play around with the free pendulum.

% Acrobot Parameters

syms q1 q2 q1d q2d q1dd q2dd qdes real;

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

acr.q1 = q1;
acr.q2 = q2;
acr.q1d = q1d;
acr.q1dd = q1dd;
acr.q2d = q2d;
acr.q2dd = q2dd;
acr.qdes = qdes;

acr.saturation_limit = 10000; % Actuator Saturation

% For link 1 linearization (noncollocated):
acr.goal = pi/2;
acr.kd1 = 6.4;
acr.kp1 = 15;

% For link 2 linearization (collocated):
acr.alpha = pi/6; % "pumping" angle
acr.kd2 = 200;
acr.kp2 = 2000;

acr.T1 = 0;
acr.T2 = 0;

%q = [q1;q2];
%qD = [q1d;q2d];
%qDD = [q1dd;q2dd];