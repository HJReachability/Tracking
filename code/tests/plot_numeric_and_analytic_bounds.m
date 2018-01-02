%
% Copyright (c) 2017, The Regents of the University of California (Regents).
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%    1. Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%
%    2. Redistributions in binary form must reproduce the above
%       copyright notice, this list of conditions and the following
%       disclaimer in the documentation and/or other materials provided
%       with the distribution.
%
%    3. Neither the name of the copyright holder nor the names of its
%       contributors may be used to endorse or promote products derived
%       from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% Please contact the author(s) of this library if you have any questions.
% Authors: Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Script to plot analytic tracking error bounds for a point mass tracking
% a simple-motion reference and compare with numerically obtained level sets.
% The current code assumes symmetric input bounds for planner and tracker.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% constants and data files

g = 9.81;

fprintf(['\nEnsure correct data file has been loaded into workspace.\n'...
        'If no data file is loaded, only the analytic set will be plotted.\n\n'])


%% user input

% prompt user for reference velocity
vertical_dim = input('Are we doing vertical dynamics? (0/1) ');
v_ref = input('Enter v_ref: '); if isempty(v_ref),  v_ref = .4; end
if vertical_dim
    T_max = input('Enter T_max: '); if isempty(T_max),  T_max = g+2; end
else
    u_max = input('Enter u_max: '); if isempty(u_max),  u_max = .1; end
end
d_a   = input('Enter d_a: ');   if isempty(d_a),    d_a   = .1; end
d_v   = input('Enter d_v: ');   if isempty(d_v),    d_v   = max(.2, v_ref/2); end
v_exp = input('Enter v_exp: '); if isempty(v_exp),  v_exp = .1; end

switching = input('Plot switching bound? (0/1) ');

if switching
    v_ref_in = input('v_ref_in: '); if isempty(v_ref_in),  v_ref_in = .7; end
    if vertical_dim
        T_max_in = input('Enter T_max_in: '); if isempty(T_max_in),  T_max_in = g+2; end
    else
        u_max_in = input('Enter u_max_in: '); if isempty(u_max_in),  u_max_in = .1; end
    end
    d_a_in   = input('Enter d_a_in: '); if isempty(d_a_in),d_a_in = .1; end
    d_v_in = input('d_v_in: '); if isempty(d_v_in), d_v_in = max(.2, v_ref_in/2); end
    v_exp_in = input('Enter v_exp_in: '); if isempty(v_exp_in),  v_exp_in = .1; end
end

% if exist('data','var')
%     levels = 0:0.1:1;
%     [c,~] = contour(linspace(grid_min(1),grid_max(1),grid_N(1)),...
%                 linspace(grid_min(2),grid_max(2),grid_N(2)),...
%                 data,...
%                 levels); % plots dim 1 on vertical axis, dim 2 on horizontal
% else
%    g = 9.81;
%     u_max = deg2rad(15); % default tilt value for roll and pitch
% end


%% value computation

% tracking control authority (assumes subsystem x or y)
if vertical_dim
    a_max = T_max - g;
    if switching, a_max_in = T_max_in - g; end
else
    a_max = g*tan(u_max);
    if switching, a_max_in = g*tan(u_max_in); end
end

% disturbed reference velocity
v_ref_d = v_ref + d_v;
if switching, v_ref_d_in = v_ref_in + d_v_in; end

% expansion of set boundaries in the position dimension
x_exp = v_exp*(2*v_ref_d + 0.5*v_exp) /(a_max - d_a);
if switching
    x_exp_in = v_exp_in*(2*v_ref_d_in + 0.5*v_exp_in) /(a_max_in - d_a_in); 
end

% position error bound

x_bound = v_ref_d.^2./ (a_max - d_a) + x_exp;
if switching
    x_bound_in = v_ref_d_in.^2./ (a_max_in - d_a_in) + x_exp_in;
end
% velocity bound
v_bound = sqrt(v_ref_d^2 + 2*(a_max - d_a)*x_exp);
if switching, v_bound_in = sqrt(v_ref_d_in^2 + 2*(a_max_in - d_a_in)*x_exp_in); end

% switching velocity and position
if switching
    x_switch = v_ref_d_in * v_bound_in / (a_max_in-d_a_in);
    k_switch = x_switch + 1/2*(-v_bound_in+v_ref_d).^2/(a_max - d_a);
    v_switch = sqrt( (a_max - d_a)*(x_exp + k_switch ));
end

% analytic equations of critical characteristics (invariant set boundaries)
X_A = @(v) ( 1/2*(v-v_ref_d).^2 - v_ref_d^2)./(a_max - d_a) - x_exp;
X_B = @(v) (-1/2*(v+v_ref_d).^2 + v_ref_d^2)./(a_max - d_a) + x_exp;
if switching
    X_A_in = @(v) ( 1/2*(v-v_ref_d_in).^2 - v_ref_d_in^2)./(a_max_in - d_a_in) - x_exp_in;
    X_B_in = @(v) (-1/2*(v+v_ref_d_in).^2 + v_ref_d_in^2)./(a_max_in - d_a_in) + x_exp_in;
    % pre-switch approach parabola

    X_A_a = @(v)  1/2*(v-v_ref_d).^2./(a_max - d_a) - k_switch;
    % X_B_a = @(v) -1/2*(v+v_ref_d).^2./(a_max - d_a) ...
    %               + (v_ref_d_in.*v_bound_in)./(a_max_in - d_a_in) ...
    %               + 1/2*(v_bound_in+v_ref_d).^2./(a_max - d_a);
    X_B_a = @(v) - 1/2*(v+v_ref_d).^2/(a_max - d_a) + k_switch;
end


%% plotting

% plot numeric value contours if data is available in workspace
figure('Position',[500,500,800,600]);
hold on

% plot analytic boundaries (exact zero level set of value function)
fplot(X_A,[-v_bound,v_bound],'LineWidth',2.5);
fplot(X_B,[-v_bound,v_bound],'LineWidth',2.5);
if switching
    % incoming set boundaries
    fplot(X_A_in,[-v_bound_in,v_bound_in],'LineWidth',2.5);
    fplot(X_B_in,[-v_bound_in,v_bound_in],'LineWidth',2.5);
    % DEBUG: plot switching point
    % plot(-v_bound_in, x_switch, 'x');
    % plot(-v_bound_in, X_A_in(-v_bound_in),'o');
    % plot(-v_bound_in, k_switch -1/2*(-v_bound_in+v_ref_d).^2./(a_max - d_a) , 's');
    % cruise parabola (pre-switch)
    fplot(X_A_a, [ v_bound_in, v_switch],'--','LineWidth', 1.5);
    fplot(X_B_a, [-v_switch,-v_bound_in],'--','LineWidth', 1.5);
    % entry parabola (post-switch)
    fplot(X_A, [-v_switch,-v_bound],'--','LineWidth', 1.5);
    fplot(X_B, [ v_bound, v_switch],'--','LineWidth', 1.5);
end
%axis([-2*v_ref, 2*v_ref, -1, 1]);
%axis equal

% Set up figure format and properties
ax = gca;
ax.FontSize=14;
xlabel('v_x')
ylabel('x')
title({sprintf('Tracking error bound for v_{ref} = %.1f',v_ref);...
       'Numeric level sets and analytic boundary'},...
       'FontWeight','Normal');
legend_  = {'numeric value',...
            '(-1/2*(v+v_{ref})^2 + v_{ref}^2) / (a_{max} - d_{acc})',...
            '(1/2*(v-v_{ref})^2 - v_{ref}^2) / (a_{max} - d_{acc})'};
if exist('data','var')
    clabel(c,levels);
else
    legend_ = legend_(2:end);
end
legend(legend_);
       