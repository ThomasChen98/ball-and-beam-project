function plot_states(ts, xs, ref_ps, ref_vs, theta_ds)

if nargin < 5
    theta_ds = []; 
end

fig = figure();
set(gcf,'Position',[100 100 1200 900]);
% Set up font size.
set(fig, 'DefaultAxesFontSize', 35);
% Set up font name
set(fig, 'DefaultTextFontName', 'Times New Roman');
% Set up interpreter
set(fig, 'DefaultTextInterpreter', 'latex');

subplot(4, 1, 1);
plot(ts, 100 * xs(1, :), 'LineWidth', 2);
hold on;
plot(ts, 100 * ref_ps, '-.', 'LineWidth', 2);
ylabel('$z$ [cm]', 'Interpreter', 'latex');
grid on;
title('State History');


subplot(4, 1, 2);
plot(ts, 100 * xs(2, :), 'LineWidth', 2);
hold on;
plot(ts, 100 * ref_vs, '-.', 'LineWidth', 2);
grid on;
ylabel('$\dot{z}$ [cm / s]', 'Interpreter', 'latex');

subplot(4, 1, 3);
plot(ts, 180 * xs(3, :) / pi, 'LineWidth', 2);
ylabel('$\theta$ [deg]', 'Interpreter', 'latex');
if ~isempty(theta_ds)
    hold on;
    plot(ts, 180 * theta_ds / pi, 'r:', 'LineWidth', 2);
    grid on;
end

subplot(4, 1, 4);
plot(ts, 180 * xs(4, :) / pi, 'LineWidth', 2);
ylabel('$\dot{\theta}$ [deg/s]', 'Interpreter', 'latex');
xlabel('$t$ [sec]', 'Interpreter', 'latex');
grid on;

saveas(gca, './plots/states','epsc');
end