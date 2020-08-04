function mpc = mpcNextState(x0, xF, N, N_opt, Ts, vOb, tt, TEB_list, tau, mpc_stored, iter)
% Planning with MPC (Ver. Lite)

if nargin < 1
    x0 = [0;0;0;0];
end

if iscolumn(x0)
    x0 = x0';
end

if nargin < 2
    xF = [10;10];
end

if iscolumn(xF)
    xF = xF';
end

if nargin < 3
    load('vOb.mat');    
end

lOb = getlOb([vOb{1,1};vOb{1,2};vOb{1,3};vOb{1,4}],...
             [vOb{2,1};vOb{2,2};vOb{2,3};vOb{2,4}],...
             [vOb{3,1};vOb{3,2};vOb{3,3};vOb{3,4}]);

if nargin < 4
  delta_x = 0.1;
end

if nargin < 5
  mpcSoFar = [];
end

if nargin < 6
  vis = true;
end

% runs everything
mpc = MpcPlanner(x0, xF, N, N_opt, Ts, lOb, tt, TEB_list, tau);

mpc.Run(mpc_stored,iter)

end