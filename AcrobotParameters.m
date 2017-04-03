function [acr] = AcrobotParameters(type)

%	Acrobot Parameters function
%	This file contains the acrobot parameters, saved into the acr structure.


% Acrobot Parameters Selection

    %%NUMERICAL PARAMETERS
    if strcmp(type,'num')
        % Acrobot parameters
        acr.m1 = 1.0;
        acr.m2 = 1.0;
        acr.I1 = 0.083;
        acr.I2 = 0.33;
        acr.l1 = 1.0;
        acr.l2 = 1.0;
        acr.lc1 = acr.l1/2;% Center of mass distance along link 1 from the fixed joint.
        acr.lc2 = acr.l2/2;% Center of mass distance along link 2 from the fixed joint.
        acr.g0 = 9.8;

        % For link 1 linearization (noncollocated):
        % NOTE: Best parameters for pseudoinverse case  kd1 = 1, kp1 = 5.
        % Other values could be kd = 2, kp = 14
        % NOTE: Best parameters for inverse case  kd1 = 1.25, kp1 = 5.
        acr.goal = pi/2;
        acr.kd1 = 1.25;
        acr.kp1 = 5.5;
        
        

        % For link 2 linearization (collocated):
        acr.alpha = 1; % "pumping" angle
        acr.kd2 = 200;
        acr.kp2 = 2000;

        acr.T2 = 0.0;

    %%SYMBOLIC PARAMETERS
    else
        syms m1 m2 I1 I2 lc1 lc2 l1 l2 g0 T1 T2 kd1 kp1 kd2 kp2 q1 q2 q1d q2d q1dd q2dd q1des q2des real;
        acr.m1 = m1;
        acr.m2 = m2;
        acr.I1 = I1;
        acr.I2 = I2;
        acr.l1 = l1;
        acr.l2 = l2;
        acr.lc1 = lc1;
        acr.lc2 = lc2;
        acr.d1 = acr.l1/2; % Center of mass distance along link 1 from the fixed joint.
        acr.d2 = acr.l2/2; % Center of mass distance along link 2 from the fixed joint.
        acr.g0 = g0;

        acr.T2 = T2;

        acr.q1 = q1;
        acr.q2 = q2;
        acr.q1d = q1d;
        acr.q1dd = q1dd;
        acr.q2d = q2d;
        acr.q2dd = q2dd;
        acr.q1des = q1des;
        acr.q2des = q2des;

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