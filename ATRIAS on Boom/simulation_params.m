sample_freq = 1000;
update_freq = sample_freq;
sample_time = 1/sample_freq;

%% Timings
sm.t_end = 30;  
sm.output_sample_time = 1/120;

% Sensors
sm.lpf_damping = sqrt(2)/2; % Butterworth filter damping
sm.fcut_velocity = 100*(2*pi); % Cutoff frequency for velocities
sm.B1_lpf_velocity = -2*exp(-sm.lpf_damping*sm.fcut_velocity*sample_time)*cos(sm.fcut_velocity*sample_time*sqrt(1-sm.lpf_damping^2));
sm.B2_lpf_velocity = exp(-2*sm.lpf_damping*sm.fcut_velocity*sample_time);
sm.A_lpf_velocity = 1 + sm.B1_lpf_velocity + sm.B2_lpf_velocity;

%% Noise
sm.sensor_noise_variance = (2e-4)^2*ones(13,1);

%% SEA Control
kp_sea_torque = 10;
kd_sea_torque = 0.9*2*sqrt(kp_sea_torque/3500*3.75);

%-- Low Pass filter for torque derivative
fcut_dtau = 80*(2*pi); % Hz, Low pass filter cutoff frequency when calculating acceleration from velocity
lpf_damping = sqrt(2)/2; % butterworth damping ratio
B1_lpf_dtau = -2*exp(-lpf_damping*fcut_dtau*sample_time)*cos(fcut_dtau*sample_time*sqrt(1-lpf_damping^2));
B2_lpf_dtau = exp(-2*lpf_damping*fcut_dtau*sample_time);
A_lpf_dtau = 1 + B1_lpf_dtau + B2_lpf_dtau;

%% Extra
kp_motor = 5000;
kd_motor = 0.9*2*sqrt(3.75*kp_motor);
