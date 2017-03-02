clear all;
close all;
clc;

acr = AcrobotParameters('num'); % Choose: noncollocated, collocated, none
% 1) noncollocated controller is really crazy and can stabilize to any target
% angle! The downside is that it requires basically boundless torque.
% 2) collocated controller does a reasonable "pumping" motion for swing-up.
% The downside is that it can't stabilize without some linear controller to
% keep it at the top.
% 3) none - no controller mode. Play around with the free pendulum.
%
% NOTE: This code does not transition to LQR at the top.

% Initial conditions:
init = [-pi    0    0   0]';

% Simulation duration
duration = 20;
animationSpeed = 1;

options1 = odeset('AbsTol', 1e-6,'RelTol',1e-6); %Transition from swing up to linear balance controller when conditions met.
[tarray, zarray] = ode15s(@CLsystem, [0 duration], init, options1, acr);