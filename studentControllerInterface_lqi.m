classdef studentControllerInterface_lqi < matlab.System
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
    end
    methods(Access = protected)
        function setupImpl(obj)
            disp("You can use this function for initializaition.");
        end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

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
            Q(1, 1) = 1;
            Q(2, 2) = 1;
            Q(5, 5) = 1e-2;
            R = 5;
            N = zeros(5, 1);

            K = lqi(sys, Q, R, N);
            V_servo = - K * [p_ball - p_ball_ref; 
                v_ball - v_ball_ref; 
                theta; 
                dtheta;
                -obj.integrator_p];

            obj.last_p = p_ball;
            obj.last_theta = theta;
            obj.t_prev = t;
            obj.integrator_p = obj.integrator_p + (p_ball - p_ball_ref) * obj.dt;
        end

%         function V_servo = stepImpl(obj, t, p_ball, theta)
%         % This is the main function called every iteration. You have to implement
%         % the controller in this function, bu you are not allowed to
%         % change the signature of this function. 
%         % Input arguments:
%         %   t: current time
%         %   p_ball: position of the ball provided by the ball position sensor (m)
%         %
%         %   theta: servo motor angle provided by the encoder of the motor (rad)
%         % Output:
%         %   V_servo: voltage to the servo input.        
%             %% Sample Controller: Simple Proportional Controller
%             t_prev = obj.t_prev;
%             % Extract reference trajectory at the current timestep.
%             [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
%             % Decide desired servo angle based on simple proportional feedback. 
%             k_p = 3;
%             theta_d = - k_p * (p_ball - p_ball_ref);
% 
%             % Make sure that the desired servo angle does not exceed the physical
%             % limit. This part of code is not necessary but highly recommended
%             % because it addresses the actual physical limit of the servo motor.
%             theta_saturation = 56 * pi / 180;    
%             theta_d = min(theta_d, theta_saturation);
%             theta_d = max(theta_d, -theta_saturation);
% 
%             % Simple position control to control servo angle to the desired
%             % position.
%             k_servo = 10;
%             V_servo = k_servo * (theta_d - theta);
%             
%             % Update class properties if necessary.
%             obj.t_prev = t;
%             obj.theta_d = theta_d;
% 
%             obj.last_theta = theta;
%             obj.last_p = p_ball;
%         end
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
