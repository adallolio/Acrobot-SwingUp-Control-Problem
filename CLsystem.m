function [vec] = CLsystem(t,vars,acr)

z = [];
zd = [];
ni = [];
nid = [];
q1 = vars(1);
q1d = vars(3);
q2 = vars(2);
q2d = vars(4);

q1des = acr.goal;
q2des = acr.alpha*atan(q1d);

[M,C,G] = AcrobotDynamicsMatrices(acr,vars);

if strcmp(acr.controller_type,'noncollocated')


        T = TorqueController(M, C, G, q1, q1d, q1des, acr);

        if T > acr.saturation_limit
                T = acr.saturation_limit;
        elseif T < -acr.saturation_limit;
                T = acr.saturation_limit;
        end
        
        q1dd = ComputeAccel1(T,q1,q2,q1d);
        q2dd = ComputeAccel2(q1,q2,q1d,q2d);

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
        
else

        T = TorqueController(M, C, G, q2, q2d, q2des, acr);

        q1dd = -(100*sin(q2)*q1d^2 - 250*T + 981*cos(q1 + q2))/(50*(2*cos(q2) + 5));
        q2dd = -(- 100*sin(q2)*q2d^2 - 200*q1d*sin(q2)*q2d + 981*cos(q1 + q2) + 2943*cos(q1))/(50*(2*cos(q2) + 5));


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

end

vec = [q1d,q1dd,q2d,q2dd]';

%vec = [ni(2,1),nid(2,1),z(2,1),zd(2,1)]';


end