function [gMax, N, dx] = determine_N_P5D_Dubins(Nfactor)
% Function for determining problem parameters favorable for the CFL
% condition (using trial and error).

vOther = 0.1;
wMax = 1.5;
dMax = [0.02; 0.02; 0.2; 0.02];

aRange = [-0.5; 0.5];
alphaMax = 6;

gMax = [0.35;  0.35;  pi;  2.5*vOther;  2.5*wMax];

u = zeros(5,1);

u(1) = vOther + gMax(4) + wMax*gMax(2) + dMax(1);
u(2) = gMax(4) + wMax*gMax(1) + dMax(2);
u(3) = gMax(5) + wMax;
u(4) = max(abs(aRange)) + dMax(3);
u(5) = alphaMax + dMax(4);

N = round(Nfactor*(gMax./u));

dx = gMax./N;
end