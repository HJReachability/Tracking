function [datas,tau,sD,trackingErrorBound]=Q6D_Q3D_RS(pMax, thrustRange, angleRange, gN, visualize)

small = 0.1;
dims = 1:6;
subDims = {dims(1:2), dims(3:4),dims(5:6)};
subDimNames = ['x','y','z'];

if nargin <1
    pMax = .5;
end

if nargin <2
    thrustRange = [4 16];
end

if nargin <3
    angleRange = [-15 15];% in degrees
end

if nargin < 4
    gN = 125*ones(1,length(dims));
end

if nargin < 5
    visualize = 0;
end

%% Dynamical system

% min and max controls for the tracker
angleRangeRad = deg2rad(angleRange);
uMin = [angleRangeRad(1); angleRangeRad(1); thrustRange(1)];
uMax = [angleRangeRad(2); angleRangeRad(2); thrustRange(2)];

% min and max velocities for planner (in x and y)

min_planner_speed = -pMax*ones(length(subDims),1);
max_planner_speed = pMax*ones(length(subDims),1);

% min and max disturbance for wind (in x and y)
dRange = [0; 0];
dMin = dRange(1)*ones(1,length(subDims));
dMax = dRange(2)*ones(1,length(subDims));

% create dynamic systems
sD = cell(1,length(subDims));
for ii = 1:length(subDims)
    sD{ii}.dynSys = Q6D_Q3D_Rel([], uMin, uMax, dMin, dMax, ...
        min_planner_speed, max_planner_speed, subDims{ii});
end

%% Grid and cost

% Grid Bounds
gMin = -5*ones(1,length(dims));
gMax = 5*ones(1,length(dims));

% createGrid takes in grid bounds, grid number, and periodic dimensions
for ii = 1:length(subDims)
    sD{ii}.grid = createGrid(gMin(subDims{ii}), gMax(subDims{ii}), gN(subDims{ii}));
end

% Cost Function
data0 = cell(1,length(subDims));

% make cost dependent on position states in each subsystem
pdim = cell(1,length(subDims));
for ii = 1:length(subDims)
    pdim{ii} = intersect(subDims{ii},sD{ii}.dynSys.pdim);
    if ~isempty(pdim{ii})
        idx = find(subDims{ii}==pdim{ii}(1));
        data0{ii} = sD{ii}.grid.xs{idx}.^2;
    end
    for jj = 2:length(pdim{ii})
        idxNext = find(subDims{ii}==pdim{ii}(jj));
        data0{ii} = data0{ii} + sD{ii}.grid.xs{idxNext}.^2;
    end
end

if visualize
    level = 1;
    figure(1)
    clf
    hInit = cell(1,length(subDims));
    for ii = 1:length(subDims)
        subplot(1,3,ii)
        hInit{ii} = visSetIm(sD{ii}.grid, data0{ii}, 'b', level);
    end
end



%% Other Parameters

for ii = 1:length(subDims)
    % is tracker minimizing or maximizing?
    sD{ii}.uMode = 'min';
    
    % is planner minimizing or maximizing?
    sD{ii}.dMode = 'max';
    
    % how high accuracy?
    sD{ii}.accuracy = 'veryHigh';
end

if visualize
    % set visualize to true
    extraArgs.visualize = true;
    
    % set slice of function to visualize
    extraArgs.RS_level = level;
    
    % figure number
    extraArgs.fig_num = 2;
    f = figure(2);
    %   set(f, 'Position', [400 400 450 400]);
    
    % delete previous time step's plot
    extraArgs.deleteLastPlot = false;
end

% time step
dt = 0.01;

% Max time
tMax = 10;

% Vector of times
tau = 0:dt:tMax;

% stop when function has converged
extraArgs.stopConverge = true;

% converges when function doesn't change by more than dt each time step
extraArgs.convergeThreshold = dt;

extraArgs.keepLast = 1;

extraArgs.quiet = 1;

% solve backwards reachable set
datas = cell(1,length(subDims));
for ii = 1:length(subDims)
    [datas{ii}, ~] = HJIPDE_solve(data0{ii}, tau, ...
        sD{ii}, 'max_data0', extraArgs);
end

trackingErrorBound = zeros(1,3);
for ii = 1:length(subDims)
    trackingErrorBound(ii) = min(sqrt(datas{ii}(:)))+small;
end

if visualize
    figure(3)
    clf
    h = cell(1,length(subDims));
    h0 = cell(1,length(subDims));
    for ii = 1:length(subDims)
        subplot(1,3,ii)
        h{ii} = visSetIm(sD{ii}.grid, sqrt(datas{ii}), 'r', trackingErrorBound(ii));
        hold on
        h0{ii} = visSetIm(sD{ii}.grid, sqrt(data0{ii}), 'b', trackingErrorBound(ii));
    end
end

matlabFolder = '/Users/sylvia/Documents/MATLAB';
plannerFolder = sprintf('%s/planner_RRT3D', matlabFolder);
if ~exist(plannerFolder, 'dir')
  mkdir(plannerFolder);
end

speedFolder = [plannerFolder '/speed_' ...
    num2str(pMax*10) '_tenths'];
if ~exist(speedFolder, 'dir')
  mkdir(speedFolder);
end

plannerFolderMatlab = sprintf('%s/planner_RRT3D_Matlab', matlabFolder);
if ~exist(plannerFolderMatlab, 'dir')
  mkdir(plannerFolderMatlab);
end

for ii = 1:length(sD)
derivs{ii} = computeGradients(sD{ii}.grid,datas{ii});
end

for ii = 1:length(subDims)
    data = datas{ii};
    grid_min = gMin(subDims{ii})';
    grid_max = gMax(subDims{ii})';
    grid_N = uint64(gN(subDims{ii}))';
    teb = zeros(length(subDims{ii}),1);
    idx = find(subDims{ii}==pdim{ii});
    teb(idx) = trackingErrorBound(ii);
    x_dims = uint64(subDims{ii}-1)'; %0-index
    u_dims = (uint64(ii)-1)';
    u_min = uMin(ii)';
    u_max = uMax(ii)';
    deriv0 = derivs{ii}{1};
    deriv1 = derivs{ii}{2};
    % note DON'T use -v7.3 extension for compression. it fucks up the C++
    % stuff
    save([speedFolder '/subsystem_' subDimNames(ii) '.mat'], ...
        'data','grid_min','grid_max','grid_N','teb','x_dims','u_dims',...
        'u_min','u_max', 'max_planner_speed','deriv0', 'deriv1')
end

if ~exist([plannerFolderMatlab '/speed_' ...
    num2str(pMax*10) '_tenths.mat'], 'file')
      save([plannerFolderMatlab '/speed_' ...
    num2str(pMax*10) '_tenths.mat'], ...
        'datas','tau','sD','trackingErrorBound','derivs');
end
end

