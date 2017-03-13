function [data,g,tau,runtime] = planeCapture(radius, g, tau, accuracy)
%% Input: grid, target, time
if nargin <1
  radius = 2;
%   dataMin = [-10 -inf -10 -inf];
%   dataMax = [10 inf 10 inf];
end

if nargin <2
  g_min = [-radius-2; -radius-2; -5; -pi];
  g_max = [radius+2; radius+2; 5; pi];
  g_N = 35*ones(length(g_min),1);
  g = createGrid(g_min,g_max, g_N, [], true);
end


  %data0 = shapeRectangleByCorners(g, ...
  %  dataMin, dataMax);
  data0 = shapeCylinder(g,[3,4],[0,0,0,0],radius);
  data0 = shapeComplement(data0);

if nargin <3
  % time vector
  t0 = 0;
  tMax = 20;
  dt = 0.025;
  tau = t0:dt:tMax;
end

if nargin<4
  accuracy = 'low';
end
% If intermediate results are not needed, use tau = [t0 tMax];

%% Input: Problem Parameters
aMax = 2;
aMin = -aMax;
uMax = 0.5;
uxMax = uMax;
uyMax = uMax;
uxMin = -uMax;
uyMin = -uMax;
wMax = 2*pi/10;
wMin = -wMax;


%% Input: SchemeDatas
schemeData.grid = g;
schemeData.aMax = aMax;
schemeData.aMin = aMin;
schemeData.uxMax = uxMax;
schemeData.uyMax = uyMax;
schemeData.uxMin = uxMin;
schemeData.uyMin = uyMin;
schemeData.wMax = wMax;
schemeData.wMin = wMin;
schemeData.hamFunc = @planeCaptureHam;
schemeData.partialFunc = @planeCapturePartial;
schemeData.accuracy = accuracy;

%% Run
tic;
%extraArgs.visualize = 'true';
extraArgs.stopInit = [0,0,0,0];
extraArgs.stopConverge = 1;
[data, tau] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero',extraArgs);
runtime = toc;


%% Save 
% if ndims(data) > 4
%   data = min(data,[],5);
% end
% save(['CDC_pl3_abs_target_' num2str(g.N(1)) 'x' num2str(g.N(1)) '_'...
%   num2str(tMax) 'sec_' schemeData.accuracy 'acc_Goal.mat'],...
%   'data','g','tau','runtime','-v7.3');
end

function hamValue = planeCaptureHam(t, data, deriv, schemeData)
% HamFunc: analytic Hamiltonian for collision avoidance.
%
% hamValue = HamFunc(t, data, deriv, schemeData)
%
% This function implements the hamFunc prototype for the three dimensional
%   aircraft collision avoidance example (also called the game of
%   two identical vehicles).
%
% It calculates the analytic Hamiltonian for such a flow field.
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   deriv	 Cell vector of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%
%   hamValue	 The analytic hamiltonian.
%
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):

checkStructureFields(schemeData, 'aMax', 'wMax', 'uxMin','uyMin', 'grid');

grid = schemeData.grid;
aMax = schemeData.aMax;
wMax = schemeData.wMax;
aMin = schemeData.aMin;
uxMax = schemeData.uxMax;
uyMax = schemeData.uyMax;
uxMin = schemeData.uxMin;
uyMin = schemeData.uyMin;

% Dynamics:
% \dot{x_r} = v_x - u_x
% \dot{v_x} = a_x
% \dot{y_r} = v_y - u_y
% \dot{v_y} = a_y

% plane minimizes over w, disturbance maximizes over v
hamValue = deriv{1}.*grid.xs{2} + deriv{3}.*grid.xs{4} ...
    +    abs(deriv{1}).*uxMin + abs(deriv{3}).*uyMin ...
    + abs(deriv{2}).*aMax + abs(deriv{4}).*wMax;

%for making outside target
% hamValue = -deriv{1}.*grid.xs{2} - deriv{3}.*grid.xs{4} ...
%     +    abs(deriv{1}).*uxMin + abs(deriv{3}).*uyMin ...
%     + abs(deriv{2}).*axMax + abs(deriv{4}).*ayMax;
  
% backwards reachable set
hamValue = -hamValue;
end

function alpha = planeCapturePartial(t, data, derivMin, derivMax, schemeData, dim)
% PartialFunc: Hamiltonian partial fcn.
%
% It calculates the extrema of the absolute value of the partials of the
%   analytic Hamiltonian with respect to the costate (gradient).
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   derivMin	 Cell vector of minimum values of the costate (\grad \phi).
%   derivMax	 Cell vector of maximum values of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%   dim          Dimension in which the partial derivatives is taken.
%
%   alpha	 Maximum absolute value of the partial of the Hamiltonian
%		   with respect to the costate in dimension dim for the
%                  specified range of costate values (O&F equation 5.12).
%		   Note that alpha can (and should) be evaluated separately
%		   at each node of the grid.


checkStructureFields(schemeData, 'aMax', 'wMax', 'uxMax','uyMax', 'grid');

grid = schemeData.grid;
aMax = schemeData.aMax;
wMax = schemeData.wMax;
uxMax = schemeData.uxMax;
uyMax = schemeData.uyMax;

switch dim
  case 1
    alpha = abs(grid.xs{3}.*cos(grid.xs{4}))+ abs(uxMax);
    
  case 2
    alpha = abs(grid.xs{3}.*sin(grid.xs{4}))+ abs(uyMax);
    
  case 3
    alpha = abs(aMax);
    
  case 4
    alpha = abs(wMax);
       
  otherwise
    error([ 'Partials only exist in dimensions 1-4' ]);
end
end