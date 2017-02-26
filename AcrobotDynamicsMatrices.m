function [ M, C, G ] = AcrobotDynamicsMatrices(acr,vars)
q1 = vars(1);
q1d = vars(3);
q2 = vars(2);
q2d = vars(4);
    a1 = acr.m1*acr.lc1^2 + acr.I1 + acr.I2 + acr.m2*(acr.l1^2 + acr.lc2^2);
    a2 = acr.m2*acr.l1*acr.lc2;
    a3 = acr.m2*acr.lc2^2 + acr.I2;

    M = [a1 + 2*a2*cos(q2), a3 + a2*cos(q2);
        a3 + a2*cos(q2), a3];
    C = [-a2*sin(q2)*(q2d^2+2*q1d*q2d);
        a2*sin(q2)*q1d^2];

    G = acr.g0*[(acr.m1*acr.lc1 + acr.m2*acr.l1)*cos(q1) + acr.m2*acr.lc2*cos(q1+q2);
            acr.m2*acr.lc2*cos(q1+q2)];

end

