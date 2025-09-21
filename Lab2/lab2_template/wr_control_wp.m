function [wr] = wr_control_wp(wr, time)
   % defalut values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML =0;
    PWMR =0;

    % setting the PWM limits, keep the codes here
    PWML = min(150, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(150, PWMR);
    wr.PWMR = uint8(max(0, PWMR));

end