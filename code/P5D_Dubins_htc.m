function u = P5D_Dubins_htc(rel_sys, uMode, s, p, g, deriv)

% Relative dynamics matrix
Q = zeros(5,3);
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 1;

rel_x = s - Q*p;

rot_mat = [cos(p(3)) sin(p(3)); -sin(p(3)) cos(p(3))];

rel_x(1:2) = rot_mat*rel_x(1:2);


dV4 = eval_u(g, deriv{4}, rel_x);
dV5 = eval_u(g, deriv{5}, rel_x);

dV = [0; 0; 0; dV4; dV5];

% Find optimal control of relative system (no performance control)
u = rel_sys.optCtrl([], rel_x, dV, uMode);

end