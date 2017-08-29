function [data_switch,tau,sD_switch]=Q6D_Q3D_Planner_Switch(planner_speed_small, planner_speed_big, extraGridPoints, visualize)
if nargin <1 
    planner_speed_small = .1;
end

if nargin <2
    planner_speed_big = 1;
end

if nargin <3
    %set grid to larger than bigger tracking error bound by some padding
    extraGridPoints = 20;
end

if nargin <4
    visualize = true;
end

surfFig = 1;
sliceFig = 2;
boxFig = 3;
surfFigAxis = [-.5 .5 -.5 .5 -.5 .5];
level = 0;
az = 0;
el = 0;

workingDirectory = '/Users/sylvia/Documents/MATLAB';
plannerFolder = [workingDirectory '/planner_RRT3D_Matlab'];
S = load([plannerFolder '/speed_' num2str(planner_speed_small*10) '_tenths.mat']);
B = load([plannerFolder '/speed_' num2str(planner_speed_big*10) '_tenths.mat']);

% shift datas down so that TEB is 0-level set


figure(surfFig)
clf

for ii = 1:length(S.sD)
    subplot(3,length(S.sD),ii)
    levelSBefore{ii} = surf(S.sD{ii}.grid.xs{1},S.sD{ii}.grid.xs{2},sqrt(S.datas{ii}));
    levelSBefore{ii}.FaceColor = 'b';
    levelSBefore{ii}.FaceAlpha = .25;
    levelSBefore{ii}.LineStyle = 'none';
    hold on
    
    levelBBefore{ii} = surf(B.sD{ii}.grid.xs{1},B.sD{ii}.grid.xs{2},sqrt(B.datas{ii}));
    levelBBefore{ii}.FaceAlpha = .25;
    levelBBefore{ii}.FaceColor = 'r';
    levelBBefore{ii}.LineStyle = 'none';
    
    axis(surfFigAxis)
    view(az,el)
    
    subplot(3,length(S.sD),length(S.sD)+ii)
    data0{ii} = sqrt(S.datas{ii}) - S.trackingErrorBound(ii);
    levelS{ii} = surf(S.sD{ii}.grid.xs{1},S.sD{ii}.grid.xs{2},data0{ii});
    levelS{ii}.FaceColor = 'b';
    levelS{ii}.FaceAlpha = .25;
    levelS{ii}.LineStyle = 'none';
    hold on
    
    dataF{ii} = sqrt(B.datas{ii}) - B.trackingErrorBound(ii);
    levelB{ii} = surf(B.sD{ii}.grid.xs{1},B.sD{ii}.grid.xs{2},dataF{ii});
    levelB{ii}.FaceAlpha = .25;
    levelB{ii}.FaceColor = 'r';
    levelB{ii}.LineStyle = 'none';
    
    axis(surfFigAxis)
    view(az,el)
end

%% Make switching scheme data

sD_switch = cell(1,length(S.sD));

for ii = 1:length(sD_switch)
   
sD_switch{ii}.dynSys = S.sD{ii}.dynSys; % set dynamic system to smaller bound (i.e. smaller planner dynamics/controls)
sD_switch{ii}.uMode = 'min'; % tracker tries to minimize
sD_switch{ii}.dMode = 'max'; % planner tries to maximize
sD_switch{ii}.accuracy = 'veryHigh';
end

%% Make Grid

% Add these grid points to bigger grid
for ii = 1:length(sD_switch)
    sD_switch{ii}.grid = B.sD{ii}.grid;
    
    % dx = (S.sD{ii}.grid.max - S.sD{ii}.grid.max)./S.sD{ii}.grid.N;
    % grid_min = B.sD{ii}.grid.min - extraGridPoints*dx;
    % grid_max = B.sD{ii}.grid.max + extraGridPoints*dx;
    % gN = B.sD{ii}.grid.N + 2*extraGridPoints;
    % sD_switch{ii}.grid = createGrid(grid_min, grid_max, gN);
end

%% Cost Function
% set the cost to the smaller data with its TEB as the zero-level set
% data0 = cell(1,length(sD_switch));
% for ii = 1:length(sD_switch)
%     data0{ii} = S.datas{ii};
%     %data0{ii} = migrateGrid(S.sD{ii}.grid,S.datas{ii},sD_switch{ii}.grid,'max');
% end
% 
% dataF = cell(1,length(sD_switch));
% for ii = 1:length(sD_switch)
%     dataF{ii} = B.datas{ii};
%     %dataF{ii} = migrateGrid(B.sD{ii}.grid,B.datas{ii},sD_switch{ii}.grid,'max');
% end


%% Visualize sets

if visualize
    figure(sliceFig);
    clf
    hInit = cell(1,length(sD_switch));
    hFinal = cell(1,length(sD_switch));
    for ii = 1:length(sD_switch)
        subplot(1,length(sD_switch),ii)
        hInit{ii} = visSetIm(sD_switch{ii}.grid, data0{ii}, 'b', level);
        hold on
        hFinal{ii} = visSetIm(sD_switch{ii}.grid, dataF{ii}, 'r', level);
    end
end

%% Set up tau
% time step
dt = 0.01;

% Max time
tMax = 10;

% Vector of times
tau = 0:dt:tMax;
%% Prep extraArgs
if visualize
    % set visualize to true
    extraArgs.visualize = true;
    % set slice of function to visualize
    extraArgs.RS_level = level;
    % figure number
    extraArgs.fig_num = 3;
    % delete previous time step's plot
    extraArgs.deleteLastPlot = false;
end

extraArgs.keepLast = 1;

% solve backwards reachable set
data_switch = cell(1,length(sD_switch));
for ii = 1:length(sD_switch)
    extraArgs.stopSetInclude = dataF{ii};
    [data_switch{ii}, ~] = HJIPDE_solve(data0{ii}, tau, ...
        sD_switch{ii}, 'min_data0', extraArgs);
end

if visualize
    figure(sliceFig)
    
    hold on
    hSwitch = cell(1,length(sD_switch));
    xlabels = {'x','y','z'};
    ylabels = {'vx','vy','vz'};
    for ii = 1:length(sD_switch)
        subplot(1,length(sD_switch),ii)
        hSwitch{ii} = visSetIm(sD_switch{ii}.grid, data_switch{ii}, 'k', level);
        legend('Small', 'Big', 'Switch')
        xlabel(xlabels{ii})
        ylabel(ylabels{ii})
    end
    
    
    figure(surfFig)
    hold on
    levelSwitch = cell(1,length(sD_switch));
    for ii = 1:length(sD_switch)
        subplot(3,length(sD_switch),ii+length(sD_switch))
        hold on
        levelSwitch{ii} = surf(sD_switch{ii}.grid.xs{1},sD_switch{ii}.grid.xs{2},data_switch{ii});
        levelSwitch{ii}.FaceColor = 'k';
        levelSwitch{ii}.FaceAlpha = .25;
        levelSwitch{ii}.LineStyle = 'none';
    end
end

%% visualize 3D boxes

% first project down to Position dimensions
dims = {1, 2, 3};
vfs0.tau = 0;
vfs0.dims = dims;

vfsF.tau = 0;
vfsF.dims = dims;

vfs_switch.tau = 0;
vfs_switch.dims = dims;

figure(surfFig)
for ii = 1:length(sD_switch)
    subplot(3,length(sD_switch),ii+2*length(sD_switch))
    [vfs0.gs{ii}, vfs0.datas{ii}] = proj(S.sD{ii}.grid, data0{ii}, [0 1], 'min');
    line0{ii} = visSetIm(vfs0.gs{ii},vfs0.datas{ii},'b',0);
    hold on
    [vfsF.gs{ii}, vfsF.datas{ii}] = proj(B.sD{ii}.grid, dataF{ii}, [0 1], 'min');
    lineF{ii} = visSetIm(vfsF.gs{ii},vfsF.datas{ii},'r',0);
    
    [vfs_switch.gs{ii}, vfs_switch.datas{ii}] = proj(sD_switch{ii}.grid, data_switch{ii}, [0 1], 'min');
    line_switch{ii} = visSetIm(vfs_switch.gs{ii},vfs_switch.datas{ii},'k',0);
    axis(surfFigAxis(3:end))
end

range_lower = [vfs0.gs{1}.min, vfs0.gs{2}.min, vfs0.gs{3}.min];
range_upper = [vfs0.gs{1}.max, vfs0.gs{2}.max, vfs0.gs{3}.max];

minOverTime = 0;


constrType = 'max';
% then recombine position dimensions
figure(boxFig)
clf

vf0 = reconSC(vfs0, range_lower, range_upper, minOverTime, constrType);
box0 = visSetIm(vf0.g,vf0.data,'b',0);
hold on

vfF = reconSC(vfsF, range_lower, range_upper, minOverTime, constrType);
boxF = visSetIm(vf0.g,vfF.data,'r',0);
boxF.FaceAlpha = .3;

vf_switch = reconSC(vfs_switch, range_lower, range_upper, minOverTime, constrType);
box_switch = visSetIm(vf0.g,vf_switch.data,'k',0);
box_switch.FaceAlpha = .2;
axis(surfFigAxis)
xlabel('x')
ylabel('y')
zlabel('z')
legend('Small','Big','Switch')
title('Tracking Error Bound in Position Space')

%% Save
% 
% matlabFolder = '/Users/sylvia/Documents/MATLAB';
% plannerFolder = sprintf('%s/planner_RRT3D', matlabFolder);
% if ~exist(plannerFolder, 'dir')
%   mkdir(plannerFolder);
% end
% 
% speedFolder = [plannerFolder '/speed_' ...
%     num2str(max_planner_speed*10) '_tenths'];
% if ~exist(speedFolder, 'dir')
%   mkdir(speedFolder);
% end
% 
% plannerFolderMatlab = sprintf('%s/planner_RRT3D_Matlab', matlabFolder);
% if ~exist(plannerFolderMatlab, 'dir')
%   mkdir(plannerFolderMatlab);
% end
% 
% 
% for ii = 1:length(subDims)
%     data = datas{ii};
%     grid_min = gMin(subDims{ii});
%     grid_max = gMax(subDims{ii});
%     grid_N = uint64(gN(subDims{ii}));
%     teb = zeros(1,length(subDims{ii}));
%     idx = find(subDims{ii}==pdim{ii});
%     teb(idx) = trackingErrorBound(ii);
%     x_dims = uint64(subDims{ii}-1); %0-index
%     u_dims = uint64(ii)-1;
%     u_min = uMin(ii);
%     u_max = uMax(ii);
%     save([speedFolder '/subsystem_' subDimNames(ii) '.mat'], ...
%         'data','grid_min','grid_max','teb','x_dims','u_dims',...
%         'u_min','u_max', 'max_planner_speed','-v7.3')
% end
%     save([plannerFolderMatlab '/speed_' ...
%     num2str(max_planner_speed*10) '_tenths.mat'], ...
%         'datas','tau','sD','trackingErrorBound');
end
