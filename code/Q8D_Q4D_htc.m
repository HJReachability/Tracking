function u = Q8D_Q4D_htc(rel_sys, uMode, s, p, g, deriv, deriv_ind)
% u = Q8D_Q4D_htc(s, p, deriv)
%     Hybrid tracking controller (htc) for 8D quadrotor tracking 4D double
%     integrator (2D position space)
%
% Inputs:
%     rel_sys - relative system object
%     s       - 8D system state
%     p       - 4D system state
%     g       - grid on which deriv is represented
%     derivX  - safety controller (gradient) look-up table, X components
%     derivY  - safety controller (gradient) look-up table, Y components
%
% Output:
%     u     - tracking controller
%
% Mo Chen, 2018-02-19

% Constants
XDims = 1:4;
YDims = 5:8;

indX = deriv_ind.x;
indY = deriv_ind.y;

% Relative state
r = Q8D_Q4D_grs(s, p);

% Access the correct indices for X and Y gradients
derivX = cell(4,1);
derivY = cell(4,1);
for k = 1:4
  derivX{k} = deriv{k}(:,:,:,:,indX);
  derivY{k} = deriv{k}(:,:,:,:,indY);
end

% Find optimal control of relative system (no performance control)
pX = eval_u(g, derivX, r(XDims));
pY = eval_u(g, derivY, r(YDims));

uX = rel_sys.optCtrl([], r(XDims), pX, uMode);
uY = rel_sys.optCtrl([], r(YDims), pY, uMode);

u = [uX uY];
end