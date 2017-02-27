function [Q1DD,Q2DD] = solver(M,C,G,T)

syms q1dd q2dd real;

[Q1DD,Q2DD]=solve([-(1/M(1,1))*(M(1,2)*q2dd+G(1)+C(1)),-(1/M(2,2))*(M(2,1)*q1dd+G(2)+C(2))+T],[q1dd,q2dd]);

end