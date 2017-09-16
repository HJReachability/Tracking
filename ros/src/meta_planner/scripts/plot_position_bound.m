%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Plot planner/tracker position and tracking bound, in each dimension.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Read all the data.
planner_xs = csvread('planner_xs.csv');
planner_ys = csvread('planner_ys.csv');
planner_zs = csvread('planner_zs.csv');

tracker_xs = csvread('tracker_xs.csv');
tracker_ys = csvread('tracker_ys.csv');
tracker_zs = csvread('tracker_zs.csv');

bound_xs = csvread('bound_xs.csv');
bound_ys = csvread('bound_ys.csv');
bound_zs = csvread('bound_zs.csv');

times = csvread('times.csv');

%% Plot subfigures, sharing the same x-axis (time).
set(gca, 'fontsize', 20);

figure;
subplot(3, 1, 1);
hold on; grid on;
plot(times, planner_xs, 'k', 'LineWidth', 2);
plot(times, tracker_xs, 'b-', 'LineWidth', 2);
plot(times, planner_xs + bound_xs, 'r.', 'LineWidth', 2);
plot(times, planner_xs - bound_xs, 'r.', 'LineWidth', 2);
hold off; grid off;
ylabel('$x$ (m)', 'Interpreter', 'latex');

subplot(3, 1, 2);
hold on; grid on;
plot(times, planner_ys, 'k', 'LineWidth', 2);
plot(times, tracker_ys, 'b-', 'LineWidth', 2);
plot(times, planner_ys + bound_ys, 'r.', 'LineWidth', 2);
plot(times, planner_ys - bound_ys, 'r.', 'LineWidth', 2);
hold off; grid off;
ylabel('$y$ (m)', 'Interpreter', 'latex');

subplot(3, 1, 3);
hold on; grid on;
plot(times, planner_zs, 'k', 'LineWidth', 2);
plot(times, tracker_zs, 'b-', 'LineWidth', 2);
plot(times, planner_zs + bound_zs, 'r.', 'LineWidth', 2);
plot(times, planner_zs - bound_zs, 'r.', 'LineWidth', 2);
hold off; grid off;
ylabel('$z$ (m)', 'Interpreter', 'latex');
xlabel('Time (s)');