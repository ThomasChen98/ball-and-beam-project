classdef studentControllerInterface_lqr_ekf < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        % ball position error & beam angle error in last timestep
        last_p = -0.19;
        last_theta = 0;

        dt = 0.01;

        %% For EKF
        u_prev = 0;
        xm_prev = [-0.19; 0.00; 0; 0];
        Pm_prev = diag([.1/100,.1/100,2*pi/180,2*pi/180]);
        H = [1 0 0 0;
            0 0 1 0];
        L = eye(4);
        M = eye(2);
        svv = diag([.1/100,.1/100,1*pi/180,1*pi/180]); % From noise settings
        sww = diag([.3/100,2*pi/180]);

    end

    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % This is the main function called every iteration. You have to implement
            % the controller in this function, bu you are not allowed to
            % change the signature of this function. 
            % Input arguments:
            %   t: current time
            %   p_ball: position of the ball provided by the ball position sensor (m)
            %
            %   theta: servo motor angle provided by the encoder of the motor (rad)
            % Output:
            %   V_servo: voltage to the servo input.  
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
            Pp = Ad*Pm_prev*Ad' + obj.L*obj.svv*obj.L';

            % Meas Update
            K_ekf = Pp*obj.H'*inv(obj.H*Pp*obj.H' + obj.M*obj.sww*obj.M');
            xm = xp + K_ekf*([p_ball;theta]-obj.H*xp);
            Pm = (eye(4) - K_ekf*obj.H)*Pp;
            %% LQR
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            v_ball = (p_ball - obj.last_p) / obj.dt;
            d_theta = (theta - obj.last_theta) / obj.dt;
            % state-space system
            rg = 0.0254; 
            L = 0.4255; 
            g = 9.81;
            K = 1.5; 
            tau = 0.025;
            c1 = 5 * g / 7 * rg / L;
            c2 = 5 / 7 * (rg / L) ^ 2;
            c3 = L / 2 - p_ball_ref;
            A = [0, 1, 0, 0;
                 c2 * d_theta^2 * cos(theta)^2, 0, c1 * cos(theta) + 2 * c2 * (c3 - p_ball) * d_theta^2 * cos(theta) * sin(theta), -2 * c2 * (c3 - p_ball) * d_theta * cos(theta)^2;
                 0, 0, 0, 1;
                 0, 0, 0, -1 / tau];
            B = zeros(4, 1);
            B(4, 1) = K / tau;
            C = [1, 0, 0, 0];
            D = zeros(1, 1);
            sys = c2d(ss(A, B, C, D), obj.dt);
            % set up LQR
            Q = diag([500,100,0.1,1]);
            R = 1;
            N = zeros(4, 1);
            % get control input
            K = lqr(sys, Q, R, N);
            V_servo = - K * (xm - [p_ball_ref v_ball_ref 0 0]'); 

            % Update class properties if necessary.
            obj.last_p = p_ball;
            obj.last_theta = theta;
            obj.t_prev = t;

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
