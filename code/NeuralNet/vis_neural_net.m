function [fig1info,fig2info,fig3info] = vis_neural_net(NN_file,R_file)
%% Load NN stuff

if nargin <1
    load('/Users/sylvia/Documents/MATLAB/NeuralNetReachability/reachability_neural_net_data.mat',...
        'Control_Actions','Disturb_Actions', 'lower_bound', ...
        'slice_points_y_vy_z_vz', 'spacing_x_vx', 'upper_bound', 'Value');
else
    load(NN_file, 'Control_Actions','Disturb_Actions', 'lower_bound', ...
        'slice_points_y_vy_z_vz', 'spacing_x_vx', 'upper_bound', 'Value');
end
Value_NN = Value;
Control_NN = Control_Actions;
Disturb_NN = Disturb_Actions;
gMin = lower_bound;
gMax = upper_bound;
gN = double(spacing_x_vx);
clear Value Control_Actions Disturb_Actions lower_bound upper_bound ...
    spacing_x_vx

% make relevant variables
TEB_NN = Value_NN(gN(1)/2, gN(2)/2);
g_NN = createGrid(gMin, gMax, double(gN));

%% Load reachability stuff
if nargin < 2
    
    %[TEB_R,Value_R,sD] = FaSTrack_DoubleInt(gN, gMin, gMax, TEB_NN,...
    %    accuracy, pMax, thrustRange, angleRange, dRangeV, dRangeA)
    
    [TEB_R,Value_R,sD] = FaSTrack_DoubleInt(flip(gN), flip(gMin), ...
        flip(gMax), TEB_NN, 'veryHigh');
    
    save('reachability_data.mat','TEB_R','Value_R','sD')
else
    load(R_file, 'TEB_R','Value_R','sD');
end

% rotate axes so x axis is velocity to match Vince's stuff
sD.grid = createGrid(gMin, gMax, gN);
Value_R = Value_R';

% make relevant variables
Deriv = computeGradients(sD.grid, Value_R);
u = sD.dynSys.optCtrl(0, sD.grid.xs, Deriv, sD.uMode);
Control_R = u{1};

%% General Visualization Parameters

% what colormap to use? parula, hsv, hot, winter
cmap = colormap(winter);

%how many levels to show? starting from TEB_NN
levels = linspace(ceil(TEB_NN),2,4);

% color for each level
delta_color_NN = length(cmap)/length(levels);

%font sizes
font_size_axes = 20;
font_size_other = 15;

%transparency
face_alpha = .7;

%viewing angles
az = 65;
el = 10;

%labels
x_axis='$s_{vx}$';
y_axis='$r_x$';

%% Overlay NN and Reachability slice @ TEB_NN

figure(1)
clf
level = TEB_NN;
l_overlay = legend;
h_overlay{1} = contour(g_NN.xs{1}, g_NN.xs{2}, Value_NN, ...
        [level level],'DisplayName','Neural Net',...
        'color','blue','lineWidth',2);
    legend show
    hold on

h_overlay{2} = contour(sD.grid.xs{1}, sD.grid.xs{2}, Value_R, ...
        [level level],'DisplayName','HJ Reachability',...
        'color','red','lineWidth',2,'lineStyle','--');
legend show
axis square
set(gca,'FontSize',font_size_other)
xlabel(x_axis,'interpreter','latex','FontSize',font_size_axes)
ylabel(y_axis,'interpreter','latex','FontSize',font_size_axes)
set(gcf,'Color','white')

%% Value Functions with control side-by-side
figure(2)
clf
subplot(1,2,1) % Neural net value function
 actions_NN = unique(Control_NN);

 delta_color_NN = floor(length(cmap)/length(actions_NN));
 actionlabels = {'A','B','C','D','E','F'};
 for ii = 1:length(actions_NN)
     action = actions_NN(ii);
     temp = Control_NN;
     
     % do some shit to make value only display at control action
     temp(temp<action) = max(actions_NN)+1;
     temp(temp>action)= max(actions_NN)+1;
     temp = (temp<(max(actions_NN)+1));
     Value_disp = Value_NN.*temp;
     Value_disp(Value_disp<0.01) = NaN;
     
     % plot
     h_value_NN{ii} = surf(g_NN.xs{1}, g_NN.xs{2}, Value_disp,...
         'DisplayName',num2str(actionlabels{ii}));
     hold on
     h_value_NN{ii}.EdgeColor = 'none';
     h_value_NN{ii}.FaceColor = cmap(ii*delta_color_NN,:);
     h_value_NN{ii}.FaceAlpha = face_alpha;
     
     % make flat surface on the bottom
     flat = ones(size(temp));
     flat = flat.*temp;
     flat(flat<1) = NaN;
     flat = flat -1;
     h_value_NN_flat{ii} = surf(g_NN.xs{1}, g_NN.xs{2}, flat);
     h_value_NN_flat{ii}.FaceColor = cmap(ii*delta_color_NN,:);
     h_value_NN_flat{ii}.EdgeColor = 'none';   
 end
view(az,el)
axis square
l_value_NN = legend([h_value_NN{:}]);
l_value_NN.Location = 'northeast';
grid off
set(gca,'FontSize',font_size_other)
xlabel(x_axis,'interpreter','latex','FontSize',font_size_axes)
ylabel(y_axis,'interpreter','latex','FontSize',font_size_axes)
zlabel('$V_{NN}$','interpreter','latex','FontSize',font_size_axes)

subplot(1,2,2) %reachability value function
actions_R = unique(Control_R);
delta_color_R = length(cmap)/length(actions_R);

for ii = 1:length(actions_R)
    action = actions_R(ii);
     temp = Control_R;
     
     % do some shit to make value only display at control action
     temp(temp<action) = max(actions_R)+1;
     temp(temp>action)= max(actions_R)+1;
     temp = (temp<(max(actions_R)+1));
     Value_disp = Value_R.*temp;
     Value_disp(Value_disp<0.0001) = NaN;
     
     % plot
     h_value_R{ii} = surf(sD.grid.xs{1}, sD.grid.xs{2}, Value_disp,...
         'DisplayName',['u_{sx} = ' num2str(actions_R(ii))]);
     hold on
     h_value_R{ii}.EdgeColor = 'none';
     h_value_R{ii}.FaceColor = cmap(ii*delta_color_R,:);
     h_value_R{ii}.FaceAlpha = face_alpha; 

     flat = ones(size(temp));
     flat = flat.*temp;
     flat(flat<1) = NaN;
     flat = flat -1;
     h_value_R_flat{ii} = surf(sD.grid.xs{1}, sD.grid.xs{2}, flat,...
         'DisplayName',['u = ' num2str(actions_R(ii))]);
     h_value_R_flat{ii}.FaceColor = cmap(ii*delta_color_R,:);
     h_value_R_flat{ii}.EdgeColor = 'none';
     h_value_R_flat{ii}.FaceAlpha = face_alpha;
     %c2 = camlight;
     %c2.Position = [30 60 -80];
end
view(az,el)
axis square
l_value_R = legend([h_value_R{:}]);
l_value_R.Location = 'northeast';
grid off
set(gca,'FontSize',font_size_other)
xlabel(x_axis,'interpreter','latex','FontSize',font_size_axes)
ylabel(y_axis,'interpreter','latex','FontSize',font_size_axes)
zlabel('$V_{R}$','interpreter','latex','FontSize',font_size_axes)

%% new visualization with level set curves and control actions
figure(3)
clf
linestyles = {'-','--','-.',':'};
% neural net stuff
subplot(1,2,1)
actions_NN = unique(Control_NN);
actionlabels = {'A','B','C','D','E','F'};
  
for jj = 1:length(actions_NN)
     action = actions_NN(jj);
     temp = Control_NN;
     temp(temp<action) = max(actions_NN)+1;
     temp(temp>action)= max(actions_NN)+1;
     temp = (temp<(max(actions_NN)+1));
     flat = ones(size(temp));
     flat = flat.*temp;
     flat(flat<1) = NaN;
     flat = flat -1;
     h_slice_NN_flat{jj} = surf(g_NN.xs{1}, g_NN.xs{2}, flat,'DisplayName',...
         [num2str(actionlabels{jj})]);
     h_slice_NN_flat{jj}.FaceColor = cmap(jj*delta_color_NN,:);
     h_slice_NN_flat{jj}.EdgeColor = 'none';
     h_slice_NN_flat{jj}.FaceAlpha = face_alpha;
     %c2 = camlight;
     %c2.Position = [30 60 -80];
axis square
hold on
 end

for ii = 1:length(levels)
    [~, h_slice_NN{ii}] = contour(g_NN.xs{1}, g_NN.xs{2}, Value_NN, ...
        [levels(ii) levels(ii)], 'LineStyle', '-',...
        'color','black','LineWidth',2);
    hold on
end
view(0,90)
l_slice_NN = legend([h_slice_NN_flat{:}]);
l_slice_NN.Location = 'northeast';
grid off
axis square
axis square
set(gca,'FontSize',font_size_other)
xlabel(x_axis,'interpreter','latex','FontSize',font_size_axes)
ylabel(y_axis,'interpreter','latex','FontSize',font_size_axes)


% reachability stuff
subplot(1,2,2)

for jj = 1:length(actions_R)
     action = actions_R(jj);
     temp = Control_R;
     temp(temp<action) = max(actions_R)+1;
     temp(temp>action)= max(actions_R)+1;
     temp = (temp<(max(actions_R)+1));
     flat = ones(size(temp));
     flat = flat.*temp;
     flat(flat<1) = NaN;
     flat = flat -1;
     h_slice_R_flat{jj} = surf(sD.grid.xs{1}, sD.grid.xs{2}, flat,'DisplayName',...
         ['u = ' num2str(actions_R(jj))]);
     h_slice_R_flat{jj}.FaceColor = cmap(jj*delta_color_R,:);
     h_slice_R_flat{jj}.EdgeColor = 'none';
     h_slice_R_flat{jj}.FaceAlpha = face_alpha;
     %c2 = camlight;
     %c2.Position = [30 60 -80];
axis square
hold on
 end

for ii = 1:length(levels)
    [~, h_slice_R{ii}] = contour(sD.grid.xs{1}, sD.grid.xs{2}, Value_R, ...
        [levels(ii) levels(ii)], 'LineStyle', '-',...
        'color','black','LineWidth',2,...
        'DisplayName',['V_{R} = ' num2str(levels(ii))]);
    hold on
end
view(0,90)
l_slice_R = legend([h_slice_R_flat{:}]);
l_slice_R.Location = 'northeast';
grid off
axis square
axis square
set(gca,'FontSize',font_size_other)
xlabel(x_axis,'interpreter','latex','FontSize',font_size_axes)
ylabel(y_axis,'interpreter','latex','FontSize',font_size_axes)

set(gcf,'Color','white')

%% Save
fig1info = {h_overlay, l_overlay};
fig2info = {h_value_R, h_value_R_flat,l_value_R,...
    h_value_NN,h_value_NN_flat, l_value_NN};
fig3info = {h_slice_R, h_slice_R_flat, l_slice_R, ...
    h_slice_NN, h_slice_NN_flat, l_slice_NN};
end
