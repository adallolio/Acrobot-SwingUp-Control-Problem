function [] = DeriveAccel(M,C,G,acr)

% Dynamics of 2R robot!
eqn1forthdotdot1 = solve(M(1,1)*acr.q1dd+M(1,2)*acr.q2dd+C(1)+G(1)-acr.T1, acr.q1dd);
eqn2forthdotdot2 = solve(M(2,1)*acr.q1dd+M(2,2)*acr.q2dd+C(2)+G(2)-acr.T2, acr.q2dd);

% Solve for q1dd and q2dd
eqn1 = simplify(solve(subs(eqn1forthdotdot1, acr.q2dd, eqn2forthdotdot2)-acr.q1dd,acr.q1dd)); 
eqn2 = simplify(solve(subs(eqn2forthdotdot2, acr.q1dd, eqn1forthdotdot1)-acr.q2dd,acr.q2dd));

acr.q1dd = eqn1;
acr.q2dd = eqn2;

matlabFunction(acr.q1dd, 'file', '/home/juan-laptop/GitHub/UR/Compute_q1dd');
matlabFunction(acr.q2dd, 'file', '/home/juan-laptop/GitHub/UR/Compute_q2dd');

end
