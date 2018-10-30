%% Timings
sample_freq = 1000;
update_freq = sample_freq;
sample_time = 1/sample_freq;
sm.t_end = 30;  
sm.output_sample_time = 1/120;

%% Noise
sm.noise_variance = (2e-7)^2;
sm.noise_scale = 0;