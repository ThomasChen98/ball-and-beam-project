%%% ME C237 Final Report Plots
%%% Yuxin Chen
%%% Apr 30
clc, clear, close all
figuresize = [50 50 1200 800];
fontsize = 30;
%% IO
load('IO_trial1_3.43.mat');
% plot output
figure('Position', figuresize,...
    'GraphicsSmoothing', 'on',...
    'Renderer','painters');
yyaxis left
plot(ts, ps*100, Linewidth=3, DisplayName='$z$');
yyaxis right
plot(ts, thetas/pi*180, Linewidth=3, DisplayName='$\theta$');
grid on
% title('Output of Input/Output Linearization with Total Score = 3.430',...
%     'Interpreter','latex');
xlabel('Time $t$ [s]','Interpreter','latex');
yyaxis left
ylabel('Ball Position $z$ [cm]','Interpreter','latex');
yyaxis right
ylabel('Motor Angle $\theta$ [$^\circ$]','Interpreter','latex');
set(gca,'fontsize',fontsize);
saveas(gca, 'IO_output','epsc');
% plot control
figure('Position', figuresize,...
    'GraphicsSmoothing', 'on',...
    'Renderer','painters');
plot(ts, us, Linewidth=3, DisplayName='$V$');
grid on
% title('Input of Input/Output Linearization with Total Score = 3.430',...
%     'Interpreter','latex');
xlabel('Time $t$ [s]','Interpreter','latex');
ylabel('Motor Input $V$ [$V$]','Interpreter','latex');
set(gca,'fontsize',fontsize);
saveas(gca, 'IO_input','epsc');

%% LQI
load('LQI_trial1_2.2598.mat');
% plot output
figure('Position', figuresize,...
    'GraphicsSmoothing', 'on',...
    'Renderer','painters');
yyaxis left
plot(ts, ps*100, Linewidth=3, DisplayName='$z$');
yyaxis right
plot(ts, thetas/pi*180, Linewidth=3, DisplayName='$\theta$');
grid on
% title('Output of Linear Quadratic Integral with Total Score = 2.2598',...
%     'Interpreter','latex');
xlabel('Time $t$ [s]','Interpreter','latex');
yyaxis left
ylabel('Ball Position $z$ [cm]','Interpreter','latex');
yyaxis right
ylabel('Motor Angle $\theta$ [$^\circ$]','Interpreter','latex');
set(gca,'fontsize', fontsize);
saveas(gca, 'LQI_output','epsc');
% plot control
figure('Position', figuresize,...
    'GraphicsSmoothing', 'on',...
    'Renderer','painters');
plot(ts, us, Linewidth=3, DisplayName='$V$');
grid on
% title('Input of Linear Quadratic Integral with Total Score = 2.2598',...
%     'Interpreter','latex');
xlabel('Time $t$ [s]','Interpreter','latex');
ylabel('Motor Input $V$ [$V$]','Interpreter','latex');
set(gca,'fontsize', fontsize);
saveas(gca, 'LQI_input','epsc');

%% LQR
load('LQR_trial1_2.0689.mat');
% plot output
figure('Position', figuresize,...
    'GraphicsSmoothing', 'on',...
    'Renderer','painters');
yyaxis left
plot(ts, ps*100, Linewidth=3, DisplayName='$z$');
yyaxis right
plot(ts, thetas/pi*180, Linewidth=3, DisplayName='$\theta$');
grid on
% title('Output of Linear Quadratic Regulator with Total Score = 2.0689',...
%     'Interpreter','latex');
xlabel('Time $t$ [s]','Interpreter','latex');
yyaxis left
ylabel('Ball Position $z$ [cm]','Interpreter','latex');
yyaxis right
ylabel('Motor Angle $\theta$ [$^\circ$]','Interpreter','latex');
set(gca,'fontsize', fontsize);
saveas(gca, 'LQR_output','epsc');
% plot control
figure('Position', figuresize,...
    'GraphicsSmoothing', 'on',...
    'Renderer','painters');
plot(ts, us, Linewidth=3, DisplayName='$V$');
grid on
% title('Input of Linear Quadratic Regulator with Total Score = 2.0689',...
%     'Interpreter','latex');
xlabel('Time $t$ [s]','Interpreter','latex');
ylabel('Motor Input $V$ [$V$]','Interpreter','latex');
set(gca,'fontsize', fontsize);
saveas(gca, 'LQR_input','epsc');