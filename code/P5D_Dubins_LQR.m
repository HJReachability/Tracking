function u = P5D_Dubins_LQR(trueCar, p)

wOther = 0;
s = trueCar.x;

% Relative dynamics matrix
Q = zeros(5,3);
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 1;

rel_x = s - Q*p;

rot_mat = [cos(p(3)) sin(p(3)); -sin(p(3)) cos(p(3))];

rel_x(1:2) = rot_mat*rel_x(1:2);

rel_x(3) = wrapToPi(rel_x(3));

theta_r = rel_x(3);
v = rel_x(4);

grad_x_f = [0       wOther -v*sin(theta_r) cos(theta_r) 0; 
            -wOther 0       v*cos(theta_r) sin(theta_r) 0; 
            0       0       0              0            1;
            0       0       0              0            0;
            0       0       0              0            0];
          
grad_u_f = [0 0;
            0 0;
            0 0;
            1 0;
            0 1];

Q = eye(5);
R = eye(2);
N = zeros(5,2);
K = lqr(grad_x_f, grad_u_f, Q, R, N);

u = K*rel_x;

% Control saturation
if u(1) > max(trueCar.aRange)
  u(1) = max(trueCar.aRange);
end

if u(1) < min(trueCar.aRange)
  u(1) = min(trueCar.aRange);
end

if u(2) > trueCar.alphaMax
  u(2) = trueCar.alphaMax;
end

if u(2) < -trueCar.alphaMax
  u(2) = -trueCar.alphaMax;
end


end