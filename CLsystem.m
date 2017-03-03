function [vec] = CLsystem(t,vars,acr)

    q1 = vars(1);
    q1d = vars(2);
    q2 = vars(3);
    q2d = vars(4);

%[M,C,G] = AcrobotDynamicsMatrices(acr,vars);

    if strcmp(acr.controller_type,'noncollocated')

        qdes = acr.goal;
        T = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,qdes);
        %T = TorqueController(M, C, G, acr, q1, q1d, qdes);


        
    elseif strcmp(acr.controller_type,'collocated')
        qdes = acr.alpha*atan(q1d);
        T = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,qdes);

    end

    if T > acr.saturation_limit
        T = acr.saturation_limit;
    elseif T < -acr.saturation_limit;
        T = acr.saturation_limit;
    end

    q1dd = Compute_q1dd(acr.I1,acr.I2,acr.T1,T,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d);
    q2dd = Compute_q2dd(acr.I1,acr.I2,acr.T1,T,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d);

    vec = [q1d,q1dd,q2d,q2dd]';

end