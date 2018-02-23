function r = Q8D_Q4D_grs(s, p)
% r = Q8D_Q4D_grs(s, p)
%    Get relative state (grs) r for a 8D quadrotor with state s and 4D double
%    integrator (2D position space) with state p

Q = zeros(8,4);
Q(1,1) = 1;
Q(2,2) = 1;
Q(5,3) = 1;
Q(6,4) = 1;

% relative state
r = s - Q*p; 


end