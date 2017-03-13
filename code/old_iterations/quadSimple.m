function [data,g,tau,runtime] = quadSimple(g, dataMin, dataMax, tau, accuracy)
%% Input: grid, target, time

if nargin <1
  g_min = [0; -5];
  g_max = [5; 5];
  g_N = [25;50];
  g = createGrid(g_min,g_max, g_N, [], true);
end

if nargin <2
  %radius = 2;
   dataMin = [4 -5];
   dataMax = [5 5];
end
  data0 = shapeRectangleByCorners(g,dataMin, dataMax);
  %data0 = shapeCylinder(g,[2,4],[0,0,0,0],radius);
  %data0 = shapeComplement(data0);

if nargin <4
  % time vector
  t0 = 0;
  tMax = 30;
  dt = 0.025;
  tau = t0:dt:tMax;
end

if nargin<5
  accuracy = 'high';
end
% If intermediate results are not needed, use tau = [t0 tMax];

%% Input: Problem Parameters
uMax = 0.5;
uxMax = uMax;
uyMax = uMax;
uxMin = -uMax;
uyMin = -uMax;


%% Input: SchemeDatas
schemeData.grid = g;
schemeData.uxMax = uxMax;
schemeData.uyMax = uyMax;
schemeData.uxMin = uxMin;
schemeData.uyMin = uyMin;
schemeData.hamFunc = @quadSimpleHam;
schemeData.partialFunc = @quadSimplePartial;
schemeData.accuracy = accuracy;

%% Run
tic;
%extraArgs.visualize = 'true';
extraArgs.stopInit = [0,0];
%extraArgs.stopConverge = 1;
[data, tau] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero',extraArgs,1);
runtime = toc;


%% Save 
% if ndims(data) > 4
%   data = min(data,[],5);
% end
% save(['CDC_pl3_abs_target_' num2str(g.N(1)) 'x' num2str(g.N(1)) '_'...
%   num2str(tMax) 'sec_' schemeData.accuracy 'acc_Goal.mat'],...
%   'data','g','tau','runtime','-v7.3');
end

function hamValue = quadSimpleHam(t, data, deriv, schemeData)
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

checkStructureFields(schemeData, 'uxMin','uyMin');

uxMin = schemeData.uxMin;
uyMin = schemeData.uyMin;

% Dynamics:
% \dot{x} = u_x
% \dot{y} = u_y

hamValue = abs(deriv{1}).*uxMin + abs(deriv{2}).*uyMin;
% backwards reachable set
hamValue = -hamValue;
end

function alpha = quadSimplePartial(t, data, derivMin, derivMax, schemeData, dim)
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


checkStructureFields(schemeData, 'uxMax','uyMax');

switch dim
  case 1
    alpha = abs(schemeData.uxMax);
    
  case 2
    alpha = abs(schemeData.uyMax);
       
  otherwise
    error([ 'Partials only exist in dimensions 1-2' ]);
end
end
