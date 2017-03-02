function [acr] = AcrobotParameters(type)

%	Acrobot Parameters function
%	This file contains the acrobot parameters, saved into the acr structure.
%  

% Choose collocated or non-collocated implementation.
acr.controller_type = 'noncollocated'; % Choose: noncollocated, collocated, none
% 1) noncollocated controller is really crazy and can stabilize to any target
% angle! The downside is that it requires basically boundless torque.
% 2) collocated controller does a reasonable "pumping" motion for swing-up.
% The downside is that it can't stabilize without some linear controller to
% keep it at the top.
% 3) none - no controller mode. Play around with the free pendulum.

% Acrobot Parameters Selection

%%NUMERICAL PARAMETERS
if strcmp(type,'num')
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

%%SYMBOLIC PARAMETERS
else
    syms m1 m2 I1 I2 lc1 lc2 l1 l2 g0 T1 T2 kd1 kp1 kd2 kp2 q1 q2 q1d q2d q1dd q2dd qdes real;
    acr.m1 = m1;
    acr.m2 = m2;
    acr.I1 = I1;
    acr.I2 = I2;
    acr.lc1 = lc1;
    acr.lc2 = lc2;
    acr.l1 = l1;
    acr.l2 = l2;
    acr.d1 = acr.l1/2; % Center of mass distance along link 1 from the fixed joint.
    acr.d2 = acr.l2/2; % Center of mass distance along link 2 from the fixed joint.
    acr.g0 = g0;


    acr.T1 = T1;
    acr.T2 = T2;


    acr.q1 = q1;
    acr.q2 = q2;
    acr.q1d = q1d;
    acr.q1dd = q1dd;
    acr.q2d = q2d;
    acr.q2dd = q2dd;
    acr.qdes = qdes;

    % For link 1 linearization (noncollocated):
    acr.kd1 = kd1;
    acr.kp1 = kp1;

    % For link 2 linearization (collocated):
    acr.kd2 = kd2;
    acr.kp2 = kp2;
end

acr.saturation_limit = 10000; % Actuator Saturation

end


%q = [q1;q2];
%qD = [q1d;q2d];
%qDD = [q1dd;q2dd];