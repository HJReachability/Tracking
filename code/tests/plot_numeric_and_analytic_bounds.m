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

fprintf(['\nEnsure correct data file has been loaded into workspace.\n'...
        'If no data file is loaded, only the analytic set will be plotted.\n\n'])

% prompt user for reference velocity
v_ref = input('Enter v_ref: ');
d_acc = input('Enter d_acc: ');

% plot numeric value contours if data is available in workspace
figure('Position',[500,500,800,600]);
hold on

if exist('data','var')
    levels = 0:0.1:1;
    [c,~] = contour(linspace(grid_min(1),grid_max(1),grid_N(1)),...
                linspace(grid_min(2),grid_max(2),grid_N(2)),...
                data,...
                levels); % plots dim 1 on vertical axis, dim 2 on horizontal
else
    g = 9.81;
    u_max = deg2rad(15); % default tilt value for roll and pitch
end

% tracking control authority (assumes subsystem x or y)
a_max = g*tan(u_max);

% analytic equations of critical characteristics (invariant set boundaries)
X_A = @(v) (1/2*(v-v_ref).^2 - v_ref^2)./(a_max - d_acc);
X_B = @(v) (-1/2*(v+v_ref).^2 + v_ref^2)./(a_max - d_acc);

% plot analytic boundaries (exact zero level set of value function)
fplot(X_B,[-v_ref,v_ref],'LineWidth',2.5);
fplot(X_A,[-v_ref,v_ref],'LineWidth',2.5);

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
       