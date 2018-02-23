function [TEB_R,Value_R,sD] = FaSTrack_DoubleInt(gN, gMin, gMax, TEB_NN,...
    accuracy, pMax, thrustRange, angleRange, dRangeV, dRangeA)

if nargin <5
    % how high accuracy?
    sD.accuracy = 'veryHigh';
end
if nargin <6
    pMax = .25;
end
if nargin <7
    thrustRange = [9.8-2 9.8+2];
end
if nargin <8
    angleRange = [-.1 .1];% in radians
end
if nargin <9
    % min and max disturbance velocity
    dRangeV = [-.25; .25];
end
if nargin <10
    % min and max disturbance acceleration
    dRangeA = [0;0];
end

small = .1;
level = TEB_NN;
dims = [1,4];
gN = double(gN);

% min and max control for tracker
uMin = [angleRange(1); angleRange(1); thrustRange(1)];
uMax = [angleRange(2); angleRange(2); thrustRange(2)];

% min and max velocities for planner (in x and y)
min_planner_speed = -pMax;
max_planner_speed = pMax;

% min and max disturbances
dMin = [dRangeV(1)*ones(3,1); ...
    dRangeA(1)*ones(3,1)];
dMax = [dRangeV(2)*ones(3,1); ...
    dRangeA(2)*ones(3,1)];

% create dynamic systems
sD.dynSys = Q6D_Q3D_Rel([], uMin, uMax, dMin, dMax, ...
    min_planner_speed, max_planner_speed, dims);

% create grid
sD.grid = createGrid(gMin, gMax, gN);

% Cost Function
% make cost dependent on position states

pdim = intersect(dims,sD.dynSys.pdim);
if ~isempty(pdim)
    idx = find(dims==pdim(1));
    data0 = sD.grid.xs{idx}.^2;
end
for jj = 2:length(pdim)
    idxNext = find(dims==pdim(jj));
    data0 = data0 + sD.grid.xs{idxNext}.^2;
end

% is tracker minimizing or maximizing?
sD.uMode = 'min';

% is planner minimizing or maximizing?
sD.dMode = 'max';

% how accurate?
sD.accuracy = accuracy;

% time step
dt = 0.01;

% Max time willing to wait
tMax = 10;

% Vector of times
tau = 0:dt:tMax;

% stop when function has converged
extraArgs.stopConverge = true;

% converges when function doesn't change by more than dt each time step
extraArgs.convergeThreshold = dt/2;

extraArgs.keepLast = 1;
extraArgs.quiet = 1;
extraArgs.visualize = 1;
extraArgs.RS_level = level;
extraArgs.fig_num = 2;
f = figure(2);
extraArgs.deleteLastPlot = false;

[datas, ~] = HJIPDE_solve(data0, tau, ...
    sD, 'maxWithTarget', extraArgs);

Value_R = sqrt(datas);
% grab tracking error bound, add a small number to it
TEB_R = min(Value_R(:))+small;
end
