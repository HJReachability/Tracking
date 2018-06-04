%% Discussion 7: Problem 1
%clear; close all; clc;
function [u] = LQR_Q2D(x,q)
if nargin <1
    x=[10; 0];
end

if nargin <2
    q = 1;
end

if numel(x) > 2
    x = x(1:2);
end
%A = [0 1; 0 0];
%B = [0;1];

K = [1/q, sqrt(2/q)];
u = -K*x;
end

% function [u_hist,x_hist ] = LQR_Q10D(x_init)
% if nargin <1
%     x_init = [10; 0];
% end
% A = [0 1; 0 0];
% B = [0;1];
% 
% q = 10;
% 
% K = [1/q, sqrt(2/q)];
% x_hist = x;
% y_hist = y;
% z_hist = z;
% u_hist = [x(2);y(2);z(2)];
% for i = 1:20
% ux = -K*x;
% uy = -K*y;
% uz = -K*z;
% 
% x = A*x + B*ux;
% y = A*y + B*uy;
% z = A*z + B*uz;
% 
% x_hist = [x_hist,x];
% y_hist = [y_hist,y];
% z_hist = [z_hist,z];
% u = [ux; uy; uz];
% u_hist = [u_hist, u];
% end
% 
% figure(1),clf
% plot(x_hist(1,:))
% end




% if nargin < 1
%     x_init = [-10 ; -10];%; 10; 10; 10; 10];
% end
% 
% % Define sytem matrices
% A = [0 1; 0 0];
% B = [0;1];
% % A = [0 1 0 0 0 0;
% %      0 0 0 0 0 0;
% %      0 0 0 1 0 0;
% %      0 0 0 0 0 0;
% %      0 0 0 0 0 1;
% %      0 0 0 0 0 0];
% % B = [0 0 0;
% %      1 0 0;
% %      0 0 0;
% %      0 1 0;
% %      0 0 0;
% %      0 0 1];
% C = [];
% 
% % Define horizon and initial state
% N = 20;
% 
% 
% % Cost matrices
% nX = size(A, 2);
% nU = size(B, 2);
% Q = eye(nX);
% Q_f = Q;
% R = eye(nU);
% [K] = lqr(A,B,Q,R);
% 
% % x = x_init;
% % x_hist = [];
% % u_hist = [];
% 
% % for i = 1:N
% %     u = -K*x;
% %     x = A*x + B*u;
% %     figure(1)
% %     plot(x(1),x(3),'o')
% %     hold on
% %     x_hist = [x_hist,x];
% %     u_hist = [u_hist,u];
% % end
% 
% %Compute K and P matrices
% [K1, P1] = lqr_finite_horizon_solution(A, B, Q, R, Q_f, N);
% %[K2, P2] = lqr_finite_horizon_solution(A, B, Q, 10*R, Q_f, N);
% 
% %Simulate the trajectory
% [x, u, J] = simulate_trajectory(K1, P1, A, B, N, x_init);
% %[x2, u2, J2] = simulate_trajectory(K2, P2, A, B, N, x_init);
% 
% 
% figure(2),clf
% plot(x(1,:), 'r-'); hold on; plot(x(2,:), 'b--'); 
% title('first'); legend('x', 'y');
% 
% % Let's do some plotting
% % figure(1),clf
% % plot(u1(1,:), 'r-'); hold on; plot(u2(1,:), 'b--'); 
% % title('Used Control'); legend('rho_R = 1', 'rho_R = 10');
% % 
% % figure(2),clf
% % plot(x1(1,:), 'r-'); hold on; plot(x2(1,:), 'b--'); 
% % title('first'); legend('rho_R = 1', 'rho_R = 10');
% % 
% % figure(3),clf
% % plot(x1(2,:), 'r-'); hold on; plot(x2(2,:), 'b--'); 
% % title('second'); legend('rho_R = 1', 'rho_R = 10');
% % 
% % figure(4),clf
% % plot(J1(1,:), 'r-'); hold on; plot(J2(1,:), 'b--'); 
% % title('optimal cost to go'); legend('rho_R = 1', 'rho_R = 10');
% % 
% function [K, P] = lqr_finite_horizon_solution(A, B, Q, R, Q_f, N)
% 
% 
% P{N+1} = Q_f;
% for i=1:N
%   K{N-i+1} = -1*inv(R + B'*P{N-i+2}*B)*B'*P{N-i+2}*A;
%   P{N-i+1} = Q + K{N-i+1}'*R*K{N-i+1} + (A+B*K{N-i+1})'*...
%     P{N-i+2}*(A+B*K{N-i+1});
% end
% 
% end
% 
% function [x, u, J] = simulate_trajectory(K, P, A, B, N, x_init)
% 
% x(:,1) = x_init;
% for i=1:N
%   u(:,i) = K{i}*x(:,i);
%   x(:,i+1) = A*x(:,i) + B*u(:,i);
%   J(:,i) = x(:,i)' * P{i} * x(:,i);
% end
% 
% end
% 
% end

