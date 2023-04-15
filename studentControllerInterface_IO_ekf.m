classdef studentControllerInterface_IO_ekf < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
%         extra_dummy1 = 0;
%         extra_dummy2 = 0;
        last_p = -0.2;
        last_theta = 0;
        
        dt = 0.01;

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
            Pp = Ad*Pm_prev*Ad' + obj.L_ekf*obj.svv*obj.L_ekf';

            % Meas Update
            K_ekf = Pp*obj.H'*inv(obj.H*Pp*obj.H' + obj.M*obj.sww*obj.M');
            xm = xp + K_ekf*([p_ball;theta]-obj.H*xp);
            Pm = (eye(4) - K_ekf*obj.H)*Pp;
             %% IO
%             t_prev = obj.t_prev;
            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            
%             v_ball = (p_ball - obj.last_p) / obj.dt;
%             d_theta = (theta - obj.last_theta) /obj.dt;
%             
            % system params
            rg = 0.0254;
            L = 0.4255;
            g = 9.81;
            K = 1.5;
            tau = 0.025;
            
            % control tuning parameters
            k1 = 4.5;
            k2 = 7;
            k3 = 6;
            k4 = 2;
            k = [k1 k2 k3 k4];
            
            % new system
            eps1 = xm(1);
            eps2 = xm(2);
            eps3 = ((5*rg)/(7*L))*g*sin(xm(3));
            eps4 = ((5*rg)/(7*L))*xm(4)*g*cos(xm(3));
            
            % new system references
            eps1_ref = p_ball_ref;
            eps2_ref = v_ball_ref;
            eps3_ref = a_ball_ref;
            eps4_ref = ((5*rg)/(7*L))*xm(4)*g*cos(xm(3));
            
            b = -((5*rg)/(7*L))*xm(4)^2*sin(xm(3));
            a = ((5*rg)/(7*L))*g*cos(xm(3));
            
            v = -k1*(eps1-eps1_ref) - k2*(eps2-eps2_ref) - k3*(eps3-eps3_ref) - k4*(eps4-eps4_ref) + eps1_ref;
            u = (-b+v)/a;
            
            % Decide desired servo angle based on simple proportional feedback.
%             k_p = 3;
%             theta_d = - k_p * (p_ball - p_ball_ref);

            % Make sure that the desired servo angle does not exceed the physical
            % limit. This part of code is not necessary but highly recommended
            % because it addresses the actual physical limit of the servo motor.
%             theta_saturation = 56 * pi / 180;    
%             theta_d = min(theta_d, theta_saturation);
%             theta_d = max(theta_d, -theta_saturation);

            % Simple position control to control servo angle to the desired
            % position.
%             k_servo = 10;
%             V_servo = k_servo * (theta_d - theta);
            V_servo = u;
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.last_p = p_ball;
            obj.last_theta = theta;
%             obj.theta_d = theta_d;
            
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