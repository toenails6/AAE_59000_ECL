function [wr] = wr_control_spd(wr, time)
   % defalut values
    wr.DIRL = 1;
    wr.DIRR = 1;
    %PWML =0;
    %PWMR =0;

    % setting the PWM limits, keep the codes here
    P_gain = 1;  
    I_gain = 1;
    D_gain = 1;

    delta_pos = wr.pos - wr.pos_old; 
    disp = dot(delta_pos, wr.heading_vec); 
    meas_spd = disp / time.dt; 
    speed_error = wr.forward_spd - meas_spd;

    persistent int_err prev_err
    if isempty(int_err)
        int_err = 0;
    end
    if isempty(prev_err)
        prev_err = 0;
    end

    int_err = int_err + speed_error * time.dt;
    d_err = (speed_error - prev_err) / time.dt;
    pwm_cmd = P_gain * speed_error + I_gain * int_err + D_gain * d_err;
    prev_err = speed_error;

    PWML = pwm_cmd;
    PWMR = pwm_cmd;

    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end