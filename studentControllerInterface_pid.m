classdef studentControllerInterface_pid < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and update while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        % ball position error & beam angle error in last timestep
        last_error_p = 0;
        last_error_theta = 0;
        % integral of ball position error & beam angle error
        int_error_p = 0;
        int_error_theta = 0;
        % time interval
        dt = 0.01;
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
            %% Sample Controller: Simple Proportional Controller
            t_prev = obj.t_prev;
            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            % Decide desired servo angle based on first PID
            error_p = p_ball - p_ball_ref; % position error
            d_error_p = (error_p - obj.last_error_p) / obj.dt; % derivative of position error
%             kp_p = 50;
%             ki_p = 0.05;
%             kd_p = 20;
            kp_p = 5;
            ki_p = 0.01;
            kd_p = 6;
            theta_d = - kp_p * error_p...
                      - ki_p * obj.int_error_p...
                      - kd_p * d_error_p;

            % Make sure that the desired servo angle does not exceed the physical
            % limit. This part of code is not necessary but highly recommended
            % because it addresses the actual physical limit of the servo motor.
            theta_saturation = 56 * pi / 180;
            theta_d = min(theta_d, theta_saturation);
            theta_d = max(theta_d, -theta_saturation);

            % Simple PID to control servo angle to the desired position.
            error_theta = theta_d - theta; % angle error
            d_error_theta = (error_theta - obj.last_error_theta) / obj.dt; % derivative of angle error
            kp_theta = 6; 
            ki_theta = 0.01;
            kd_theta = 0.5;
            V_servo = kp_theta * error_theta...
                    + ki_theta * obj.int_error_theta...
                    + kd_theta * d_error_theta;
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.theta_d = theta_d;
            obj.last_error_p = error_p;
            obj.last_error_theta = error_theta;
            obj.int_error_p = obj.int_error_p + error_p * obj.dt;
            obj.int_error_theta = obj.int_error_theta + error_theta * obj.dt;
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