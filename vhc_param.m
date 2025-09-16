%% white rover
wr.ID = 12;
% wr.mode = 0;
% Mode
wr.mode = 0;
% forward angle [deg]
wr.fwd_deg = 15;
% distance marin [mm]
wr.dist_mar = 100;
wr.curWP = 1;
% PID gain
wr.spd = 0;
wr.espd = 0;
wr.e_heading = 0;
wr.e_heading_cum = 0;
wr.e_heading_old = 0;
wr.espd_old = 0;
wr.espd_cum = 0;
wr.pos = [0 0];
wr.pos_old = [0 0];

% wr.forward_spd = 110;
wr.spd_avg = 0;
wr.filter_n = 10;
wr.v_buf = NaN(1, wr.filter_n);
wr.filter_init = 0;
wr.filter_count = 0;

% purely for testing a filter
wr.i2 = 0;
wr.test_filter_spd = zeros(1,5);
wr.PWML = 0;
wr.PWMR = 0;
