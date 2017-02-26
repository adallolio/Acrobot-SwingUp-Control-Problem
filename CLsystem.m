function [vec] = CLsystem(t,vars,acr)

z = zeros(2,1);
zd = zeros(2,1);
ni = zeros(2,1);
nid = zeros(2,1);
q1 = vars(1);
q1d = vars(3);
q2 = vars(2);
q2d = vars(4);

q1des = acr.goal;
q2des = acr.alpha*atan(q1d);

[M,C,G] = AcrobotDynamicsMatrices(acr,vars);

if strcmp(acr.controller_type,'noncollocated')
        ni(1) = q1-q1des;
        ni(2) = q1d;
        z(1) = q2;
        z(2) = q2d;
        v1 = - acr.kp1*ni(1) - acr.kd1*ni(2);
        nid(1) = ni(2);
        nid(2) = v1;
        zd(1) = z(2);
        zd(2) = -(1/M(1,2))*(C(1) + G(1) + M(1,1)*v1);
else
        ni(1) = q1;
        ni(2) = q1d;
        z(1) = q2-q2des;
        z(2) = q2d;
        v2 = - acr.kp2*z(1) - acr.kd2*z(2);
        zd(1) = z(2);
        zd(2) = v2;
        nid(1) = ni(2);
        nid(2) = (-1/M(1,1))*(C(1) + G(1) + M(1,2)*v2);
end


vec = [ni(2),nid(2),z(2),zd(2)]';


end