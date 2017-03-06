function [vec] = CLsystem(t,vars,acr)

    q1 = vars(1);
    q1d = vars(2);
    q2 = vars(3);
    q2d = vars(4);

    %[M,C,G] = AcrobotDynamicsMatrices(acr,vars);
    
    %{
    if (q1 < 1.7453 && q1 > 1.3963 && q2 < figure(1)
        grid on 
        hold on 
        plot(tarray,zarray(:,1),'b')  
        plot(tarray,zarray(:,2),'r') 
        hold off 0.05 && q2 > -0.05)
        internal_controller = 'LQR';
        disp('HOlaaa')
    else 
         internal_controller = 'SwingUp';
    end
    %}

    internal_controller = 'SwingUp';
    
    if strcmp(acr.controller_type,'noncollocated')
        q1des = acr.goal;
        T = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,q1des);
        %T = TorqueController(M, C, G, acr, q1, q1d, qdes);
    elseif strcmp(acr.controller_type,'collocated')
        q2des = acr.alpha*atan(q1d);
        T = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,q2des);
    end

    if T > acr.saturation_limit
        T = acr.saturation_limit;
    elseif T < -acr.saturation_limit;
        T = acr.saturation_limit;
    end

    if strcmp(internal_controller,'SwingUp')
        q1dd = Compute_q1dd(acr.I1,acr.I2,acr.T1,T,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d);
        q2dd = Compute_q2dd(acr.I1,acr.I2,acr.T1,T,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d);
    elseif strcmp(internal_controller,'LQR')
        q1dd = (-553.4018)*(q1-qdes)+(-401.9784)*q2+(-98.9437)*q1d+(-55.0397)*q2d+(0.6100)*T;
        q2dd = (1.2314e+03)*(q1-qdes)+(889.0061)*q2+(231.9501)*q1d+(129.0274)*q2d+(-1.4300)*T;
    end

    vec = [q1d,q1dd,q2d,q2dd]';

end