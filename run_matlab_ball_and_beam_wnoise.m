close all
clear all

%% General Settings.
% Initial state.
x0 = [-0.19; 0.00; 0; 0];
t0 = 0;
% Simulation time.
T = 40;
% Sampling time of the controller
dt = 0.01;
% ode function to use.
ode_func = @ode45;
% print log for each timestep if true.
verbose = false;
% plot animation if true.
plot_animation = false;
% save animation to video if true.
save_video = false;
% Noise is on if true.
hasnoise = true;

controller_handle = studentControllerInterface_lqi_ekf();
% controller_handle = studentControllerInterface_IO_ekf();
u_saturation = 10;

% Noise setting
x_sig = [.05/100,.05/100,.5*pi/180,.5*pi/180];
w_sig = [.1/100,1*pi/180];

% Initialize traces.
xs = x0;
ts = t0;
us = [];
theta_ds = [];
[p_ball_ref, v_ball_ref] = get_ref_traj(t0);
ref_ps = p_ball_ref;
ref_vs = v_ball_ref;

% Initialize state & time.
if hasnoise
    x = x0 + normrnd(0,x_sig,1,4)'; % Noise
else
    x = x0;
end
t = t0;
end_simulation = false;

%% Run simulation.
% _t indicates variables for the current loop.
tstart = tic;
while ~end_simulation
    %% Determine control input.
    tstart = tic; % DEBUG   
    if hasnoise
        [u, theta_d] = controller_handle.stepController(t, x(1) + normrnd(0,w_sig(1)), x(3) + normrnd(0,w_sig(2))); % Noise
    else
        [u, theta_d] = controller_handle.stepController(t, x(1), x(3));
    end
    u = min(u, u_saturation);
    u = max(u, -u_saturation);
    if verbose
        print_log(t, x, u);    
    end
    tend = toc(tstart);    
    us = [us, u];          
    theta_ds = [theta_ds, theta_d];
    %% Run simulation for one time step.
    t_end_t = min(t + dt, t0+T);
    ode_opt = odeset('Events', @event_ball_out_of_range);
    [ts_t, xs_t, t_event] = ode_func( ...
        @(t, x) ball_and_beam_dynamics(t, x, u), ...
        [t, t_end_t], x, ode_opt);
    end_simulation = abs(ts_t(end) - (t0 + T))<1e-10 || ~isempty(t_event);
    end_with_event = ~isempty(t_event); 
    t = ts_t(end);
    if hasnoise
        x = xs_t(end, :)' + normrnd(0,x_sig,1,4)'; % Noise
    else
        x = xs_t(end, :)';
    end
    %% Record traces.
    xs = [xs, x];
    ts = [ts, t];
    [p_ball_ref, v_ball_ref] = get_ref_traj(t);
    ref_ps = [ref_ps, p_ball_ref];
    ref_vs = [ref_vs, v_ball_ref];    
end % end of the main while loop
%% Add control input for the final timestep.
if hasnoise
    [u, theta_d] = controller_handle.stepController(t, x(1) + normrnd(0,w_sig(1)), x(3) + normrnd(0,w_sig(2))); % Noise
else
    [u, theta_d] = controller_handle.stepController(t, x(1), x(3));
end
u = min(u, u_saturation);
u = max(u, -u_saturation);
us = [us, u];
theta_ds = [theta_ds, theta_d];
if verbose
    print_log(t, x, u);    
end
ps = xs(1, :);
thetas = xs(3, :);

% Evaluate the score of the controller.
score = get_controller_score(ts, ps, thetas, ref_ps, us);

%% Plots
% Plot states.
plot_states(ts, xs, ref_ps, ref_vs, theta_ds);
% Plot output errors.
% plot_tracking_errors(ts, ps, ref_ps);        
% Plot control input history.
% plot_controls(ts, us);
plot_errors_and_controls(ts,us,ps,ref_ps);

if plot_animation
    animate_ball_and_beam(ts, ps, thetas, ref_ps, save_video);
end
% figure(5)
% subplot(4,1,1)
% plot(ts,xs(1,:)-xms(1,:))
% subplot(4,1,2)
% plot(ts,xs(2,:)-xms(2,:))
% subplot(4,1,3)
% plot(ts,xs(3,:)-xms(3,:))
% subplot(4,1,4)
% plot(ts,xs(4,:)-xms(4,:))
% xlabel('t')

function print_log(t, x, u)
        fprintf('t: %.3f, \t x: ', t);
        fprintf('%.2g, ', x);
        fprintf('\t u: ');
        fprintf('%.2g, ', u);
        % Add custom log here.
        fprintf('\n');
end