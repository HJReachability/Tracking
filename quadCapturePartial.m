function alpha = quadCapturePartial(t, data, derivMin, derivMax, schemeData, dim)
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


checkStructureFields(schemeData, 'axMax', 'ayMax', 'uxMax','uyMax', 'grid');

grid = schemeData.grid;
axMax = schemeData.axMax;
ayMax = schemeData.ayMax;
uxMax = schemeData.uxMax;
uyMax = schemeData.uyMax;

switch dim
  case 1
    alpha = abs(grid.xs{2})+abs(uxMax);
    
  case 2
    alpha = abs(axMax);
    
  case 3
    alpha = abs(grid.xs{4})+abs(uyMax);
    
  case 4
    alpha = abs(ayMax);
       
  otherwise
    error([ 'Partials only exist in dimensions 1-4' ]);
end
end