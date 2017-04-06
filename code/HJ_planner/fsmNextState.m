function xNext = fsmNextState(start, goal, obs, delta_x, vis)

if nargin < 1
  start = [-0.75; -0.75; pi/6];
end

if nargin < 2
  goal = [0.5; 0.5; pi/2];
end

%% Constants
L = 1;
g = createGrid([-L; -L; 0], [L; L; 2*pi], [35; 35; 35], 3);

% numerical infinity
numInfty = 1e6;

% Semi-axes of ellipsoid target
semi_axes = [0.1; 0.1; pi/6];
target = shapeEllipsoid(g, goal, semi_axes);

if nargin < 3
  obs = shapeCylinder(g, 3, [0;0;0], 0.2);
end

if nargin < 4
  delta_x = 0.01;
end

if nargin < 5
  vis = true;
end

%% Problem parameters
dubin = 1; % 1 for Dubin's car, -1 for RS's car

v = ones(g.N'); % velocity, uniform except at obstacles
v(obs<0) = 0;

turn_radius = 0.05;
p = turn_radius*ones(g.N');   % turning radius, uniform for now

% Target set
u = numInfty * ones(g.N');
u(target<0) = 0;

%% Solver parameters
% === fast marching parameters ===
march = 0; % 1 to enable fast marching, 0 to disable

% fast marching update scheme
%   0: semi-Lagrangian only
%   1: finite difference only
%   2: both semi-Lagrangian and finite difference
march_method = 0;

% === fast sweeping parameters ===
sweep = 1; % 1 to enable fast sweeping, 0 to disable

% fast sweeping update scheme
%   0: semi-Lagrangian only
%   1: finite difference only
%   2: both semi-Lagrangian and finite difference
sweep_method = 1;

%% Check for errors
% this reduces the chance of the C++ code crashing
params = {dubin, march, march_method, sweep, sweep_method};
error_check(p, g, params, vis);

%% run mex code to calculate value function
% disp(' ')
% mex CCMotion.cpp;

tic;
uf = CCMotion(u, v, p, L, numInfty, dubin, march, sweep, march_method, ...
  sweep_method);
toc;

%% Compute gradient
% use central differencing to calculate Du3
P = computeGradients(g, uf);

C = 2*pi*turn_radius; % Amount of distance to travel in time T
speed = 0.1;
T = C / speed;
turn_rate = 2*pi / T;
delta_t = delta_x / speed;

traj = start;
maxIter = 500;
small = 0.1;
for i = 1:maxIter
  % Compute local gradient
  x = traj(:,end);
  x(3) = wrapTo2Pi(x(3));
  p3 = eval_u(g, P{3}, x);
  
  % Update
  xNext = [x(1) + speed*cos(x(3)) * delta_t; ...
           x(2) + speed*sin(x(3)) * delta_t; ...
           x(3) - numSgn(p3) * turn_rate * delta_t];
  
  traj = cat(2, traj, xNext);
                  
  if eval_u(g, target, xNext) <= small
    break
  end
end
      
if vis
  %% Level set
  reachSetFig = figure;
  figure(reachSetFig);
  
  level = 1;
  visSetIm(g, uf, 'g', level);
  hold on;
  
  % dim = 3;
  %% theta contour
  figure
  levels = 0:0.1:5;
  color = 'flat';
  
  subplot(2,2,1)
  thetaVal = 0;
  [g2D, data2D] = proj(g, uf, [0 0 1], thetaVal);
  visSetIm(g2D, data2D, color, levels);
  
  subplot(2,2,2);
  thetaVal = pi;
  [g2D, data2D] = proj(g, uf, [0 0 1], thetaVal);
  visSetIm(g2D, data2D, color, levels);
  
  subplot(2,2,3)
  thetaVal = start(3);
  [g2D, data2D] = proj(g, uf, [0 0 1], thetaVal);
  visSetIm(g2D, data2D, color, levels);
 
  %% min over theta contour
  subplot(2,2,4)
  [g2D, data2D] = proj(g, uf, [0 0 1]);
  visSetIm(g2D, data2D, color, levels);
  hold on
  plot(traj(1,:), traj(2,:), 'r.')
end
end

function error_check(p, g, params, vis)
% error_check(p, g, params, vis)

[dubin, march, march_method, sweep, sweep_method] = deal(params{:});

%% Check problem parameters
% Car type
if dubin ~= 1 && dubin ~= -1
  error('The variable dubin must be 1 or -1!')
end

% Fast marching
if march ~= 1 && march ~= 0
  error('The variable march must be 0 or 1!')
end

if march_method ~= 0 && march_method ~= 1 && march_method ~= 2
  error('the variable march_method must be 0, 1, or 2!')
end

% Fast sweeping
if sweep ~= 1 && sweep ~= 0
  error('The variable sweep must be 0 or 1!')
end

if sweep_method ~= 0 && sweep_method ~= 1 && sweep_method ~= 2
  error('the variable sweep_method must be 0, 1, or 2!')
end

% Stability condition
maxp = max(p(:));
dx = g.dx(1);
dy = g.dx(2);
dth = g.dx(3);

if min([dx dy]) < maxp*sin(dth)
  error('Stability condition not satisfied!');
end

%% Display information
if vis
  if dubin == 1
    disp('Dubins car')
  elseif dubin == -1
    disp('Reeds-Shepp car')
  end
  
  if march == 1
    disp('Fast marching enabled!')
  elseif march == 0
    disp('Fast marching disabled.')
  end
  
  if march_method == 0
    if march
      disp('     using semi-Lagrangian update')
    end
  elseif march_method == 1
    if march
      disp('     using finite difference update')
    end
  elseif march_method == 2
    if march
      disp('     using both updates')
    end
  end
  
  if sweep == 1
    disp('Fast sweeping enabled!')
  elseif sweep == 0
    disp('Fast sweeping disabled.')
  end
  
  if sweep_method == 0
    if sweep
      disp('     using semi-Lagrangian update')
    end
  elseif sweep_method == 1
    if sweep
      disp('     using finite difference update')
    end
  elseif sweep_method == 2
    if sweep
      disp('     using both updates')
    end
  end
  
  disp(['maxp*sin(dtheta)/min(dx,dy) = ' num2str(maxp*sin(dth)/min([dx dy]))])
end
end

function out = numSgn(in)
% numerical signum function

if abs(in) > 0.01
    out = sign(in);
else
    out = 0;
end

end