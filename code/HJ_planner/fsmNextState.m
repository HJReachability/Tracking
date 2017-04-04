function xNext = fsmNextState()
%% Constants
g.Nx  = 41;   % x grid number
g.Ny = 41;    % y grid number
g.Nt = 15;    % theta grid number
g.N = g.Nx;     % Nx = Ny for now

% domain [-L,L]^2
g.L = 1;

% grid
g.X  = linspace(-g.L,g.L,g.N);
g.Y  = linspace(-g.L,g.L,g.N);
g.TH = linspace(0,2*pi,g.Nt+1);
[g.x,g.y,g.th] = ndgrid(g.X,g.Y,g.TH(1:end-1));

% numerical infinity
numInfty = 1e6;

%% Problem parameters
dubin = 1; % 1 for Dubin's car, -1 for RS's car

v = ones(g.N,g.N,g.Nt);       % velocity, uniform for now
p = 0.1*ones(g.N,g.N,g.Nt);   % turning radius, uniform for now

% Target set
u = create_target('cylinder', 0.2, g, numInfty);

%% Solver parameters
% === fast marching parameters ===
march = 0; % 1 to enable fast marching, 0 to disable

% fast marching update scheme
%   0: semi-Lagrangian only
%   1: finite difference only
%   2: both semi-Lagrangian and finite difference
march_method = 0;
% =================================

% === fast sweeping parameters ===
sweep = 1; % 1 to enable fast sweeping, 0 to disable

% fast sweeping update scheme
%   0: semi-Lagrangian only
%   1: finite difference only
%   2: both semi-Lagrangian and finite difference
sweep_method = 1;
% =================================

%% Check for errors
% this reduces the chance of the C++ code crashing
params = {dubin, march, march_method, sweep, sweep_method};
error_check(p,g,params);

%% run mex code to calculate value function
disp(' ')
mex CCMotion.cpp;

tic;
[uf,sc] = CCMotion(u,v,p,g.L,numInfty,dubin,...
    march,sweep,march_method, sweep_method);
toc;

ii = 2:g.N-1; % interior nodes in x,y
ufint = uf(ii,ii,:);

%% Level set
reachSetFig = figure;
figure(reachSetFig);

level = 0.5;
plot_3D(g, uf, ii,level);
hold on;

% dim = 3;
%% theta contour
sliceFig = figure;
figure(sliceFig); subplot(2,2,1)
thetaVal = 0;
plot_slice(g,ufint, ii,thetaVal);

figure(sliceFig); subplot(2,2,2); hold on
thetaVal = 180;
plot_slice(g,ufint, ii,thetaVal);

figure(sliceFig); subplot(2,2,3)
thetaVal = 135;
plot_slice(g,ufint, ii,thetaVal);

%% min over theta contour
figure(sliceFig); subplot(2,2,4)
plot_min_slice(g,ufint,ii)
end

function u = create_target(target_type, param, g, numInfty)
% function to create target set
% Grid size

% initial matrix
u = numInfty*ones(g.Nx,g.Ny,g.Nt);

switch target_type
    case 'cylinder'
        radius = param;
        u(sqrt((g.x).^2+(g.y).^2) < radius) = 0;
        
    case 'square'
        side = param;
        u(max(abs(g.x),abs(g.y)) < side) = 0;
        
    case 'off center cylinder'
        radius = param(1);
        xoffset = param(2);
        yoffset = param(3);
        u(sqrt((g.x-xoffset).^2+(g.y-yoffset).^2) < radius) = 0;
        
    case 'x half plane'
        u(g.x <= 0) = 0;
        
    case 'y half plane'
        u(g.y <= 0) = 0;

    case 'diagonal half plane'
        u(g.x<=g.y) = 0;
        
    case 'boundary'
        thickness = param;
        u(max(abs(g.x),abs(g.y)) > g.L-thickness) = 0;
        
    otherwise
        error(['Target of type ' target_type ' not recognized!'])
end

end

function plot_3D(g, uf, ii,level)

[x, y, th, uf] = nd2mesh(g.x, g.y, g.th, uf);

% p = patch(isosurface(x(ii,ii,:), y(ii,ii,:), th(ii,ii,:)*180/pi, valFunc, level));
% isonormals(x(ii,ii,:), y(ii,ii,:), th(ii,ii,:)*180/pi, valFunc,p)

isosurface(x,y,th*180/pi, uf,level);

% isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, valFunc,level);

% isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, valFunc,0.5);
% 
% camlight headlight
% 
% drawnow;
% % hold on;
% 
% % for kk = 1:10
% %     isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, FinalValue,0.7*kk);
% % end
% 
% %isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, FinalValue,0.4);
% %isosurface(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, FinalValue,numInfty/2);
% xlabel('y')
% ylabel('x')
% axis([-L,L,-L,L,0,360]);
% title(['N = ' num2str(N), ', N_\theta = ',num2str(Nt)]);
% axis square;

camlight headlight
drawnow;

xlabel('x')
ylabel('y')

axis([-g.L,g.L,-g.L,g.L,0,360]);
title(['N = ' num2str(g.N), ', N_\theta = ',num2str(g.Nt)]);
axis square;
end

function plot_slice(g, u, ii,theta)
% function to plot a theta slice of the value function

u = nd2mesh(u); % convert to meshgrid...

thetaValRad = theta*pi/180;
[~, kk] = min(abs(g.TH - thetaValRad));

contour(g.X(ii), g.Y(ii), u(:,:,kk), 0:0.05:5);
% [x,y,th] = meshgrid(X,Y,TH(1:end-1));
% contourslice(x(ii,ii,:),y(ii,ii,:),th(ii,ii,:)*180/pi, valFunc,[], [], theta);

xlabel('x'); ylabel('y')
title(['theta = ' num2str(g.TH(kk)*180/pi) ' degrees'])
xlim([-g.L, g.L]); ylim([-g.L, g.L])
axis square; grid on
end

function plot_min_slice(g,u, ii)
% function to plot the minimum over theta values in x-y space

u = nd2mesh(u); % convert to meshgrid...

% min over theta contour
FinalValue2 = min(u,[],3);
contour(g.X(ii),g.Y(ii),FinalValue2,0:0.05:5);
xlabel('x'); ylabel('y'); title('min over theta')
axis square;
end

function plotTraj(T,Z,g,u,dim)
% function to visualize trajectory simulations

u = nd2mesh(u); % convert to meshgrid...

switch dim
    case 2
        plot(Z(1,1),Z(1,2),'.'); hold on
        plot(Z(:,1),Z(:,2)); hold on

        xlabel('x'); ylabel('y');
        xlim([-g.L g.L]); ylim([-g.L g.L]);
        axis square; grid on    
    case 3
        plot3(Z(1,1),Z(1,2),Z(1,3)*180/pi,'.'); hold on
        plot3(Z(:,1),Z(:,2),Z(:,3)*180/pi); hold on
        xlabel('x'); ylabel('y'); zlabel('\theta')
        xlim([-g.L g.L]); ylim([-g.L g.L]); zlim([0 360]);

        axis square; grid on
    otherwise
        error('Trajectory plots must be in 2 or 3 dimensions!')
end
    

end

function error_check(p,g,params)

[dubin, march, march_method, sweep, sweep_method] = deal(params{:});
%% Check problem parameters
% Car type
if dubin == 1
    disp('Dubins car')
elseif dubin == -1
    disp('Reeds-Shepp car')
else
    error('The variable dubin must be 1 or -1!')
end

% Fast marching
if march == 1
    disp('Fast marching enabled!')
elseif march == 0
    disp('Fast marching disabled.')
else
    error('The variable march must be 0 or 1!')
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
else
    error('the variable march_method must be 0, 1, or 2!')
end

% Fast sweeping
if sweep == 1
    disp('Fast sweeping enabled!')
elseif sweep == 0
    disp('Fast sweeping disabled.')
else
    error('The variable sweep must be 0 or 1!')
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
else
    error('the variable sweep_method must be 0, 1, or 2!')
end
% Stability condition
maxp = max(p(:));
dth = g.TH(2)-g.TH(1);
dx = g.X(2)-g.X(1);
dy = g.Y(2)-g.Y(1);

disp(['maxp*sin(dtheta)/min(dx,dy) = ' num2str(maxp*sin(dth)/min([dx dy]))])

if min([dx dy]) < maxp*sin(dth)
    error('Stability condition not satisfied!');
end
end

function [umesh, xmesh, ymesh, thmesh] = nd2mesh(und, xnd, ynd, thnd)
% function to convert ndgrid to meshgrid

% Computation should be done on ndgrid, but visualization needs to be done
% on meshgrid
if nargin < 2
    umesh = permute(und,[2,1,3]);
else
    umesh = permute(und,[2,1,3]);
    xmesh = permute(xnd,[2,1,3]);
    ymesh = permute(ynd,[2,1,3]);
    thmesh = permute(thnd,[2,1,3]);
end
end