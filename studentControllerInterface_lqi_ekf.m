classdef studentControllerInterface_lqi_ekf < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        last_theta = 0;
        last_p = -0.19;
        rg = 0.0254; L = 0.4255; g = 9.81; 
        K = 1.5; tau = 0.025;
%         K = 10; tau = 0.01;

        dt = 0.01;
        integrator_p = 0;

        input_type = 0;
        amplitude = 0.06;
        period = 6;

        %% For EKF
        u_prev = 0;
        xm_prev = [-0.19; 0.00; 0; 0];
        Pm_prev = diag([.1/100,.1/100,2*pi/180,2*pi/180]);
        H = [1 0 0 0;
            0 0 1 0];
        L_ekf = eye(4);
        M = eye(2);
        svv = diag([.1/100,.1/100,1*pi/180,1*pi/180]); % From noise settings
        sww = diag([.3/100,2*pi/180]);

    end
    methods(Access = public)
        function obj = studentControllerInterface_lqi_ekf(input_type, amplitude, period, fid);
            fprintf("Using LQI with KF. \nInput_type %d\tAmplitude %1.2f\t Period %d\n", input_type, amplitude, period);
            fprintf(fid, "Using LQI with KF. \nInput_type %d\tAmplitude %1.2f\t Period %d\n", input_type, amplitude, period);
            obj.input_type = input_type;
            obj.amplitude = amplitude;
            obj.period = period;
        end
    end
    methods(Access = protected)
        function setupImpl(obj)
            disp("You can use this function for initializaition.");
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            %% EKF
            t_prev = obj.t_prev;
            xm_prev = obj.xm_prev;
            Pm_prev = obj.Pm_prev;
            u_prev = obj.u_prev;


            % Prior Update
            a = 0.4183;
            b = 5.4151e-04;
            c = .0025;
            tau = 0.025;
            Ts = obj.dt;

            dx = ball_and_beam_dynamics(t_prev, xm_prev, u_prev);
            xp = xm_prev + dx*Ts;

            Ad = [1, Ts,  0, 0;
                  Ts*c*xm_prev(4)^2*cos(xm_prev(3))^2, 1, Ts*(a*cos(xm_prev(3)) + b*xm_prev(4)^2*sin(2*xm_prev(3)) -...
                  c*xm_prev(1)*xm_prev(4)^2*sin(2*xm_prev(3))), -2*Ts*xm_prev(4)*cos(xm_prev(3))^2*(b - c*xm_prev(1));
                  0,  0,  1, Ts;
                  0,  0,  0, 1 - Ts/tau];
            Pp = Ad*Pm_prev*Ad' + obj.L_ekf*obj.svv*obj.L_ekf';

            % Meas Update
            K_ekf = Pp*obj.H'*inv(obj.H*Pp*obj.H' + obj.M*obj.sww*obj.M');
            xm = xp + K_ekf*([p_ball;theta]-obj.H*xp);
            Pm = (eye(4) - K_ekf*obj.H)*Pp;

            %% LQI
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t, obj.input_type, obj.amplitude, obj.period);

            v_ball = (p_ball - obj.last_p) / (t - obj.t_prev);
            dtheta = (theta - obj.last_theta) / (t - obj.t_prev);

            c1 = 5 * obj.g / 7 * obj.rg / obj.L;
            c2 = 5 / 7 * (obj.rg / obj.L) ^ 2;
            c3 = obj.L / 2 - p_ball_ref;
            A = [0, 1, 0, 0;
                c2 * dtheta^2 * cos(theta)^2, 0, c1 * cos(theta) + 2 * c2 * (c3 - p_ball) * dtheta^2 * cos(theta) * sin(theta), -2 * c2 * (c3 - p_ball) * dtheta * cos(theta)^2;
                0, 0, 0, 1;
                0, 0, 0, -1 / obj.tau];
            B = zeros(4, 1);
            B(4, 1) = obj.K / obj.tau;
            C = [1, 0, 0, 0];
            D = zeros(1, 1);
            sys = c2d(ss(A, B, C, D), obj.dt);
            Q = eye(5);
            Q = diag([300 1 100 1 1e-3]);
            R = 0.01;
            N = zeros(5, 1);

            K = lqi(sys, Q, R, N);
            V_servo = - K * [xm(1) - p_ball_ref; 
                xm(2) - v_ball_ref; 
                xm(3); 
                xm(4);
                -obj.integrator_p];

            obj.last_p = p_ball;
            obj.last_theta = theta;
            obj.t_prev = t;
            obj.integrator_p = obj.integrator_p + (p_ball - p_ball_ref) * obj.dt;

            obj.xm_prev = xm;
            obj.Pm_prev = Pm;
            obj.u_prev = V_servo;
        
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
    
end
