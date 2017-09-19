gif_out_filename = './visualize_Q6D_Q3D_tracking.gif';
avi_out_filename = './visualize_Q6D_Q3D_tracking.avi';

v = VideoWriter(avi_out_filename);
v.FrameRate = 30;
open(v)
%clear
% load('/Users/sylvia/Documents/MATLAB/planner_RRT3D_video/speed_4_tenths/subsystem_x.mat',...
%     'data','grid_min','grid_max','grid_N');
% datas{1} = data;
% grid_N = double(grid_N);
% sD{1}.grid = createGrid(grid_min, grid_max, grid_N);
% timesteps = size(data,3);
% 
% load('/Users/sylvia/Documents/MATLAB/planner_RRT3D_video/speed_4_tenths/subsystem_y.mat',...
%     'data','grid_min','grid_max','grid_N')
% datas{2} = data;
% grid_N = double(grid_N);
% sD{2}.grid = createGrid(grid_min, grid_max, grid_N);
% 
% load('/Users/sylvia/Documents/MATLAB/planner_RRT3D_video/speed_4_tenths/subsystem_z.mat',...
%     'data','grid_min','grid_max','grid_N')
% datas{3} = data;
% grid_N = double(grid_N);
% sD{3}.grid = createGrid(grid_min, grid_max, grid_N);
% timesteps2 = size(data,3);

axis_val = [-1 1 -1 1 0 1];
axis_val_2 = [-1 1 -1 1];
surfColor = [.4 .4 .4];
trans = .7;
cameraPos = [-10 -80 -80];
teb = [.5485 .5485 .2553];
ind = 1;
fontsize = 30;
az = 140;
el = 30;
linewidth = 3;
color = [0.5843 0.8157 0.9882]; %line color

figure(1)
while ind <= timesteps
clf

% X DIMENSION
subplot(3,2,1)
hsurf_x = surf(sD{1}.grid.xs{2},sD{1}.grid.xs{1},datas{1}(:,:,ind));
hsurf_x.LineStyle = 'none';
hsurf_x.FaceLighting = 'phong';
axis(axis_val)
hsurf_x.FaceColor = surfColor;
hsurf_x.FaceAlpha = trans;
cx = camlight;
%cx.Position = cameraPos;
set(gca,'FontSize',fontsize)
%view(az,el)
%axis square
hold on

hcontour_x = visSetIm(sD{1}.grid,datas{1}(:,:,ind)', color,teb(1));
hcontour_x.ContourZLevel = teb(1);
hcontour_x.LineWidth = linewidth;
xlabel('$$r_{vx}$$','Interpreter','Latex')
ylabel('$$r_{x}$$','Interpreter','Latex')
%zlabel('Value function')
set(gca,'FontSize',fontsize)

subplot(3,2,2)
hslice_x = visSetIm(sD{1}.grid,datas{1}(:,:,ind)',color,teb(1));
hslice_x.LineWidth = linewidth;
%hslice_x0 = visSetIm(sD{1}.grid,data0{1},?k?,teb(1));
xlabel('$$r_{vx}$$','Interpreter','Latex')
ylabel('$$r_{x}$$','Interpreter','Latex')
set(gca,'FontSize',fontsize)
axis(axis_val_2)

% Y DIMENSION
subplot(3,2,3)
hsurf_y = surf(sD{2}.grid.xs{2},sD{2}.grid.xs{1},datas{2}(:,:,ind));
hsurf_y.LineStyle = 'none';
hsurf_y.FaceLighting = 'phong';
axis(axis_val)
hsurf_y.FaceColor = surfColor;
hsurf_y.FaceAlpha = trans;
cy = camlight;
%view(az,el)
%cy.Position = cameraPos;
%axis square
hold on
hcontour_y = visSetIm(sD{2}.grid,datas{2}(:,:,ind)', color,teb(2));
hcontour_y.ContourZLevel = teb(2);
hcontour_y.LineWidth = linewidth;
xlabel('$$r_{vy}$$','Interpreter','Latex')
ylabel('$$r_{y}$$','Interpreter','Latex')
%zlabel('error function')
set(gca,'FontSize',fontsize)

subplot(3,2,4)
hslice_y = visSetIm(sD{2}.grid,datas{2}(:,:,ind)',color,teb(2));
hslice_y.LineWidth = linewidth;
%hslice_y0 = visSetIm(sD{2}.grid,data0{2},?k?,teb(2));
xlabel('$$r_{vy}$$','Interpreter','Latex')
ylabel('$$r_{y}$$','Interpreter','Latex')
set(gca,'FontSize',fontsize)
axis(axis_val_2)

% Z DIMENSION
subplot(3,2,5)
hsurf_z = surf(sD{3}.grid.xs{2},sD{3}.grid.xs{1},datas{3}(:,:,ind));
hsurf_z.LineStyle = 'none';
hsurf_z.FaceLighting = 'phong';
axis(axis_val)
hsurf_z.FaceColor = surfColor;
hsurf_z.FaceAlpha = trans;
cz = camlight;
%cz.Position = cameraPos;
%view(az,el)
%axis square
hold on
hcontour_z = visSetIm(sD{3}.grid,datas{3}(:,:,ind)', color,teb(3));
hcontour_z.ContourZLevel = teb(3);
hcontour_z.LineWidth = linewidth;
xlabel('$$r_{vz}$$','Interpreter','Latex')
ylabel('$$r_{z}$$','Interpreter','Latex')
%zlabel('error function')
set(gca,'FontSize',fontsize)

subplot(3,2,6)
hslice_z = visSetIm(sD{3}.grid,datas{3}(:,:,ind)',color,teb(3));
hslice_z.LineWidth = linewidth;
%hslice_z0 = visSetIm(sD{3}.grid,data0{3},?k?,teb(3));
xlabel('$$r_{vz}$$','Interpreter','Latex')
ylabel('$$r_{z}$$','Interpreter','Latex')
axis(axis_val_2)

set(gcf,'color','w')
set(gca,'FontSize',fontsize)

frame = getframe(gcf); 
image_data = frame2im(frame);
[imind,cm] = rgb2ind(image_data,256);

if ind == 1
 imwrite(imind,cm,gif_out_filename,'gif', 'Loopcount',inf); 
else
imwrite(imind,cm,gif_out_filename,'gif','WriteMode','append');
end

writeVideo(v,image_data)

ind = ind+1;
end

close(v)