function hamValue = quadCaptureHam(t, data, deriv, schemeData)
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

checkStructureFields(schemeData, 'axMin', 'ayMin', 'uxMax','uyMax', 'grid');

grid = schemeData.grid;
axMax = schemeData.axMax;
ayMax = schemeData.ayMax;
axMin = schemeData.axMin;
ayMin = schemeData.ayMin;
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
    + abs(deriv{2}).*axMax + abs(deriv{4}).*ayMax;

%for making outside target
% hamValue = -deriv{1}.*grid.xs{2} - deriv{3}.*grid.xs{4} ...
%     +    abs(deriv{1}).*uxMin + abs(deriv{3}).*uyMin ...
%     + abs(deriv{2}).*axMax + abs(deriv{4}).*ayMax;
  
% backwards reachable set
hamValue = -hamValue;
end