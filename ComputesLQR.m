function [ Ac, Bc, Cc, Dc, K ] = ComputesLQR( A, B )

    C = eye(4);
    D = zeros(4,1);

    % Q and R definition (Parameters for the controller)
    Q = eye(4);
    R = 1;

    K = lqr(A,B,Q,R);

    Ac = (A-B*K);
    Bc = B;
    Cc = C;
    Dc = D;


end

