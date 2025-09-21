function [wr] = wr_control_spd(wr, time)
   % defalut values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML =0;
    PWMR =0;

    % PID gains. 
    K_P = 0.4;  
    K_I = 0.0;
    K_D = 2;

    % Enforce limits and push control outputs to data struct. 
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end