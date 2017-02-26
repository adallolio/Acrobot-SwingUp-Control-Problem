function [ Tc ] = TorqueController2( M, C, G, K, q2, q2d, q2des)

    kp = K(1);
    kd = K(2);

    d2bar = M(2,2) - M(2,1)*M(1,2)/M(1,1);
    h2bar = C(2) - M(2,1)*C(1)/M(1,1);
    phi2bar = G(2) - M(2,1)*G(1)/M(1,1);
    v2 = kd*-q2d + kp*(q2des - q2);
    Tc = d2bar*v2 + h2bar + phi2bar;
end

