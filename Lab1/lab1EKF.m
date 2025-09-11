classdef lab1EKF < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        x; 
        P; 
        Q; 
        R; 
    end
    properties (Access = private)
        % Robot and sample parameters. 
        L = 151.2; 
        T = 1/100; 
    end

    methods
        function obj = lab1EKF(x_0, Q, R)
            % Class constructor. 

            obj.x = x_0; % Initial state estimate
            obj.P = eye(3); % Initial covariance estimate
            obj.Q = Q; % Set process noise covariance
            obj.R = R; % Set measurement noise covariance
        end

        function retval = update(obj, u, z)
            % EKF update method. 

            % State prediction with control input
            obj.x = obj.x + [
                sum(u, 'all')/2*cos(obj.x(3)); ...
                sum(u, 'all')/2*sin(obj.x(3)); ...
                (-u(1)+u(2))/obj.L]*obj.T; 
            
            % Update covariance with process noise
            F = eye(3); 
            F(1:2, 3) = [
                -sum(u, 'all')/2*sin(obj.x(3)); ...
                sum(u, 'all')/2*cos(obj.x(3))]; 
            obj.P = F*obj.P*F.' + obj.Q; 
            
            % Measurement residual
            y = z - obj.x(1:2); 
            
            % Residual covariance
            H = [1 0 0; 0 1 0]; 
            S = H * obj.P * H.' + obj.R; 
            
            K = obj.P * H.' / S; % Kalman gain
            obj.x = obj.x + K * y; % Update state estimate
            obj.P = (eye(3) - K * H) * obj.P; % Update covariance estimate
            retval = obj.x; % Return the updated state estimate
        end
    end
end