function plot_errors_and_controls(ts, us, ps, ref_ps)

fig = figure();
set(gcf,'Position',[100 100 600 550]);
% Set up font size.
set(fig, 'DefaultAxesFontSize', 35);
% Set up font name
set(fig, 'DefaultTextFontName', 'Times New Roman');
% Set up interpreter
set(fig, 'DefaultTextInterpreter', 'latex');

subplot(211)
plot(ts, (100 * abs(ps - ref_ps)), 'LineWidth', 2);
ylabel('$|z-z_r|$ [cm]');
xlabel('$t$ [sec]', 'Interpreter', 'latex');
grid on;
title('Output error');

subplot(212)
plot(ts, us, 'LineWidth', 2);
ylabel('$u$ [V]');
xlabel('$t$ [sec]');
grid on;    
title('Control Input History');
saveas(gca, './plots/errors_controls','epsc');
