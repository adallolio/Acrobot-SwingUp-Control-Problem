function [vec] = CLsystem(t,vars,acr)

%z = [];
%zd = [];
%ni = [];
%nid = [];
    q1 = vars(1);
    q1d = vars(2);
    q2 = vars(3);
    q2d = vars(4);

%[M,C,G] = AcrobotDynamicsMatrices(acr,vars);

    if strcmp(acr.controller_type,'noncollocated')

        qdes = acr.goal;
        T = ComputeTorque1(acr.I1,acr.I2,acr.g0,acr.kd1,acr.kp1,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,qdes);
        %T = TorqueController(M, C, G, acr, q1, q1d, qdes);
        %q1dd = -(100*sin(q2)*q1d^2 - 250*T + 981*cos(q1 + q2))/(50*(2*cos(q2) + 5));
        %q2dd = -(- 100*sin(q2)*q2d^2 - 200*q1d*sin(q2)*q2d + 981*cos(q1 + q2) + 2943*cos(q1))/(50*(2*cos(q2) + 5));
        %{
        ni(1,1) = q1-q1des;
        ni(2,1) = q1d;
        z(1,1) = q2;
        z(2,1) = q2d;
        v1 = - acr.kp1*ni(1,1) - acr.kd1*ni(2,1);
        nid(1,1) = ni(2,1);
        nid(2,1) = v1;
        zd(1,1) = z(2,1);
        zd(2,1) = - pinv(M(1,2)) * (C(1)+G(1)) - pinv(M(1,2))*M(1,1) * v1;
        %zd(2,1) = -(1/M(1,2))*(C(1) + G(1) + M(1,1)*v1);

%}
        
    elseif strcmp(acr.controller_type,'collocated')
        qdes = acr.alpha*atan(q1d);
        T = ComputeTorque2(acr.I1,acr.I2,acr.g0,acr.kd2,acr.kp2,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d,qdes);
        %q1dd = -(100*sin(q2)*q1d^2 - 250*T + 981*cos(q1 + q2))/(50*(2*cos(q2) + 5));
        %q2dd = -(- 100*sin(q2)*q2d^2 - 200*q1d*sin(q2)*q2d + 981*cos(q1 + q2) + 2943*cos(q1))/(50*(2*cos(q2) + 5));
        %{
        ni(1,1) = q1;
        ni(2,1) = q1d;
        z(1,1) = q2-q2des;
        z(2,1) = q2d;
        v2 = - acr.kp2*z(1,1) - acr.kd2*z(2,1);
        zd(1,1) = z(2,1);
        zd(2,1) = v2;
        nid(1,1) = ni(2,1);
        nid(2,1) = -inv(M(1,1))*(C(1) + G(1)) - inv(M(1,1))*M(1,2)*v2;
        %nid(2,1) = (-1/M(1,1))*(C(1) + G(1) + M(1,2)*v2);
        %}
    else
        T = 0.0;
    end

    if T > acr.saturation_limit
        T = acr.saturation_limit;
    elseif T < -acr.saturation_limit;
        T = acr.saturation_limit;
    end

    q1dd = Compute_q1dd(acr.I1,acr.I2,acr.T1,T,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d);
    q2dd = Compute_q2dd(acr.I1,acr.I2,acr.T1,T,acr.g0,acr.l1,acr.lc1,acr.lc2,acr.m1,acr.m2,q1,q2,q1d,q2d);

    vec = [q1d,q1dd,q2d,q2dd]';
    %vec = [ni(2,1),nid(2,1),z(2,1),zd(2,1)]';
end