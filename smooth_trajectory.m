function [out] = smooth_trajectory(t)

f = 0.8;
F1 = sin(2*pi*f*t+pi/2);

K = 3;
a = 0.1;
F2 = K*exp(a*t)-1;

out = F1.*F2-90;

end