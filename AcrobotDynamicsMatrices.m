function [ M, C, G ] = AcrobotDynamicsMatrices(m1, m2, I1, I2, lc1, lc2, l1, l2, g0, q1, q2, q1d, q2d)

    a1 = m1*lc1^2 + I1 + I2 + m2*(l1^2 + lc2^2);
    a2 = m2*l1*lc2;
    a3 = m2*lc2^2 + I2;

    M = [a1 + 2*a2*cos(q2), a3 + a2*cos(q2);
        a3 + a2*cos(q2), a3];
    C = [-a2*sin(q2)*(q2d^2+2*q1d*q2d);
        a2*sin(q2)*q1d^2];

    G = g0*[(m1*lc1 + m2*l1)*cos(q1) + m2*lc2*cos(q1+q2);
            m2*lc2*cos(q1+q2)];

end

