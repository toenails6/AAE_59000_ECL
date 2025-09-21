function [wr] = wr_control_spd(wr, time)
   % Defalut values for straight velocity control. 
    wr.DIRL = 1;
    wr.DIRR = 1;
    % PWML = 0;
    % PWMR = 0;

    % PID gains. 
    K_P = 0.4;  
    K_I = 0.0;
    K_D = 2;

    % PID controller implementation. 
    % Proportion error. 
    % delta_pos = wr.pos - wr.pos_old; 
    % meas_spd = norm(delta_pos)/time.dt;
    meas_spd = norm(wr.pos - wr.pos_old)/time.dt; 
    error = wr.forward_spd - meas_spd;

    % Integral and integral clamping. 
    integral = integral + error * Ts;  
    integral = min(max(integral, 0), 150); 

    % Derivative. 
    derivative = (error - prev_error) / Ts;

    % Update/Record error history. 
    prev_error = error;

    % PID controller output. 
    u = K_P * error + K_I * integral + K_D * derivative;

    % Enforce limits and push control outputs to data struct. 
    % Equal control efforts on both sides for straight velocity control. 
    PWML = u; 
    PWMR = u; 
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end