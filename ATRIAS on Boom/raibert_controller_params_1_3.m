%% Robot model parameters for control
ctrl.g = sm.g;
ctrl.k_sea = repmat(sm.k_leg_spring,1,4);
ctrl.l_seg = 0.50;
ctrl.lateral_offset = 0.1831;
ctrl.m_total = 62.5911 + 0.5*4.4900;
ctrl.m_leg_motor = 20.3438;
ctrl.m_leg = 2.3438;
ctrl.m_torso = 21.9034;
ctrl.i_leg = ctrl.m_leg/2*ctrl.l_seg^2;
ctrl.i_torso = [1.5, 2.2, 1.5];
ctrl.i_leg_motor = [0.29, 0.27,  0.10];
ctrl.i_robot = 2.0;%5.8;
ctrl.i_motor = 3.75;
ctrl.i_lateral_about_pelvis = (ctrl.m_leg + ctrl.m_leg_motor)*ctrl.lateral_offset^2;
ctrl.com_torso = [0, 0, 0.50];
ctrl.com_leg_motor = [0, ctrl.lateral_offset-0.0149, 0.029];
ctrl.pelvis_to_imu = [0.11988, -0.16071+0.07366/2, 0.47675];
ctrl.leg_mtr_gear_ratio = 50;
ctrl.Force_Control = true;

%% Actuator limits, aftering gearing
ctrl.max_torque_lateral = 629.28; % [Nm]
ctrl.max_sagittal_unloaded_torque = 900; % [Nm]

%% Control Mechanical limits
ctrl.max_leg_length = cos(0.5/2);
ctrl.min_leg_length = 0.5;
ctrl.min_lateral_beta = 75*pi/180;
ctrl.max_lateral_beta = 99*pi/180;
ctrl.bar_angle_limit_margin = 5*pi/180;  % Margin between fourbar limits and commanded positions
ctrl.min_front_bar_angle = 1.61 + ctrl.bar_angle_limit_margin;
ctrl.max_front_bar_angle = 3.50 - ctrl.bar_angle_limit_margin;
ctrl.min_back_bar_angle = 2.78 + ctrl.bar_angle_limit_margin;
ctrl.max_back_bar_angle = 4.68 - ctrl.bar_angle_limit_margin;

%% Control initialization variables
ctrl.initial_leg_selection = 0; % 1 == right, 0 == left (always start with left leg on boom)
% double support startup
ctrl.standing_x_pos_swing = -1.1*cosd(75);
ctrl.standing_z_pos_swing = 1.1*sind(75);
ctrl.standing_x_pos_stance = ctrl.standing_x_pos_swing;
ctrl.standing_z_pos_stance = ctrl.standing_z_pos_swing;
% single support startup
ctrl.initial_x_pos_swing = 0;
ctrl.initial_z_pos_swing = 1.1;
ctrl.initial_x_pos_stance = ctrl.initial_x_pos_swing;
ctrl.initial_z_pos_stance = ctrl.initial_z_pos_swing;

ctrl.stepping_time = 1;

%% Contact detection parameters
ctrl.twilight_time = 0.000;
ctrl.weight_factor = 1.2;
ctrl.touchdown_threshold_lo = 120; % [N]
% ctrl.touchdown_threshold_lo = 70; % [N]
ctrl.touchdown_threshold_hi = 450; % [N]
% ctrl.touchdown_threshold_hi = (ctrl.weight_factor + 0.5)*ctrl.m_total*ctrl.g; % [N]
ctrl.touchdown_force_rate_hi = 5*10^4;
ctrl.touchdown_dist_hi = 0.03;

%% Actuator control parameters
ctrl.lateral_motor_efficiency = 0.65;
ctrl.sagittal_motor_efficiency = 0.60;
ctrl.kp_sea = 20;

%% Swing - Leg control
ctrl.kp_velocity_swing = 10;
ctrl.kp_torque_swing = 5000;
ctrl.kd_torque_swing = 1.0*2*sqrt(ctrl.kp_torque_swing*ctrl.i_motor);
ctrl.kp_lateral_swing = 1000;
ctrl.kd_lateral_swing = 0.8*2*sqrt(ctrl.kp_lateral_swing*ctrl.i_lateral_about_pelvis);
ctrl.z_retract_flight = 0.10;  % [m], max distance to retract foot from ground (going to flight)
ctrl.max_z_retract = 0.20; % [m]
ctrl.min_z_retract = 0.075; % [m]
ctrl.retract_extend_speed = 1.25; % [m/s]
ctrl.reactive_swing_time = 0.500;
ctrl.z_swing_target = 1.1;
ctrl.y_swing_target_right = 0.15;  % [m], lateral distance to y foot target in swing
ctrl.y_swing_target_left = -0.15;  % [m], lateral distance to y foot target in swing
ctrl.min_swing_time = 0.200;   % [s], min time for planning a single support swing foot trajectory
ctrl.avg_horz_swing_speed = 4; % [m/s], average horizontal swing foot speed for planning time
ctrl.max_vert_swing_speed = 1; % [m/s], max vertical swing foot speed for planning time
ctrl.k_placement = 0.1;

%% Stance - Centroidal Model Feedback Control
% (See simulink block for details)
% state = [x, dx/dt, z, dz/dt, theta, theta/dt]
% control = [Fx, Fz1, Fz2] or [F_magnitude, F_angle]    

ctrl.Q_lqr_single = diag([1, 10, 100, 100, 100, 10]);
ctrl.H_lqr_single = diag([1, 10, 100, 100, 100, 10]);
ctrl.R_lqr_single = diag([2e-5 3e2]);

ctrl.Q_lqr_double = diag([1, 10, 100, 100, 100, 10]);
ctrl.H_lqr_double = diag([1, 10, 100, 100, 100, 10]); 
ctrl.R_lqr_double = diag([10, 1, 1]*10e-5);

ctrl.dt_lqr = 0.010; % discrete time step for LQR
ctrl.zero_crossing_dt = 0.001;
ctrl.max_lqr_iterations = 100;
ctrl.max_trajectory_length = ctrl.max_lqr_iterations + 1;
ctrl.max_lqr_horizon = ctrl.dt_lqr*ctrl.max_lqr_iterations; % maximum horizon time (in seconds) to consider during LQR
ctrl.desired_torso_pitch = 0*pi/180;

% leg hyperextension prevention
ctrl.stop_stance_leg_extension = 0.92; % leg length to start reducing ground forces
ctrl.max_stance_leg_extension = 0.96;  % leg length where force attenuation = exp(-rate*x)
ctrl.hyperextension_attenuation_rate = -log(0.5)/0.1; % rate in attenuation = exp(-rate*x)

%% Low-pass filters
ctrl.lpf_damping = sqrt(2)/2;
ctrl.f_imu = 250*(2*pi);            % cutoff for orientation velocities
ctrl.f_accelerometer = 80*(2*pi);   % cutoff for translational accelerations
ctrl.f_lateral_accel = 80*(2*pi);   % cutoff for lateral angular accelerations
ctrl.f_dtau = 200*(2*pi);           % cutoff for derivative of desired joint torques
ctrl.B1_imu = -2*exp(-ctrl.lpf_damping*ctrl.f_imu*sample_time)*...
                 cos(ctrl.f_imu*sample_time*sqrt(1-ctrl.lpf_damping^2));
ctrl.B2_imu = exp(-2*ctrl.lpf_damping*ctrl.f_imu*sample_time);
ctrl.A_imu = 1 + ctrl.B1_imu + ctrl.B2_imu;
ctrl.B1_lpf_accelerometer = -2*exp(-ctrl.lpf_damping*ctrl.f_accelerometer*sample_time)*...
                               cos(ctrl.f_accelerometer*sample_time*sqrt(1-ctrl.lpf_damping^2));
ctrl.B2_lpf_accelerometer = exp(-2*ctrl.lpf_damping*ctrl.f_accelerometer*sample_time);
ctrl.A_lpf_accelerometer = 1 + ctrl.B1_lpf_accelerometer + ctrl.B2_lpf_accelerometer;
ctrl.B1_lpf_lateral_accel = -2*exp(-ctrl.lpf_damping*ctrl.f_lateral_accel*sample_time)*...
                               cos(ctrl.f_lateral_accel*sample_time*sqrt(1-ctrl.lpf_damping^2));
ctrl.B2_lpf_lateral_accel = exp(-2*ctrl.lpf_damping*ctrl.f_lateral_accel*sample_time);
ctrl.A_lpf_lateral_accel = 1 + ctrl.B1_lpf_lateral_accel + ctrl.B2_lpf_lateral_accel;
ctrl.B1_lpf_dtau = -2*exp(-ctrl.lpf_damping*ctrl.f_dtau*sample_time)*...
                               cos(ctrl.f_dtau*sample_time*sqrt(1-ctrl.lpf_damping^2));
ctrl.B2_lpf_dtau = exp(-2*ctrl.lpf_damping*ctrl.f_dtau*sample_time);
ctrl.A_lpf_dtau = 1 + ctrl.B1_lpf_dtau + ctrl.B2_lpf_dtau;

%% CoM Kalman filter parameters
% Coordinate system is z-up and in the yawed frame.
% x_state = [x, dx/dt, ddx/ddt]
% x_measurement = [x_right_or_left, ddx/ddt]
% z_state = [z, dz/dt, ddz/ddt]
% z_measurement = [z_right, z_left, ddz/ddt]
ctrl.A_kalman_2nd_order = [1 sample_time; 0 1];
ctrl.B_kalman_2nd_order = [0; 1/ctrl.m_total];
ctrl.A_kalman_3rd_order = [1 sample_time 0; ...
                          0 1 sample_time; ...
                          0 0 1];
ctrl.B_kalman_3rd_order = [0; 0; 1/ctrl.m_total];
ctrl.C_kalman_vertical = [1 0 0; 1 0 0; 0 0 1];
ctrl.G_kalman_vertical = ctrl.B_kalman_3rd_order;
ctrl.C_kalman_transverse = [1 0 0; 0 0 1];
ctrl.G_kalman_transverse = ctrl.B_kalman_3rd_order;
ctrl.C_kalman_no_position = [1 0; 1 0; 0 1];
ctrl.G_kalman_no_position = ctrl.B_kalman_2nd_order;
% Covariances
ctrl.Q_kalman_GRFz_difference = 2*(50)^2/sample_time; % covariance of dF/dt = 2*cov(F)/dt
ctrl.Q_kalman_GRFx_difference = 2*(50)^2/sample_time;
ctrl.Q_kalman_GRFy_difference = 2*(50)^2/sample_time;
ctrl.R_kalman_accelerometer = 1.00^2;  % m/s^2
ctrl.R_kalman_foot_stance = 0.01^2 + 0.005^2;  % m, geometry error + ground error
ctrl.R_kalman_foot_slip = 0.01^2 + 1^2;
ctrl.R_kalman_dfoot_stance = 0.25^2; % m/s
ctrl.R_kalman_dfoot_slip = 5^2; % m/s
ctrl.kalman_stance_slip_threshold = 0.1;
% Initial estimates
ctrl.P0_kalman_transverse = diag([0.015^2, 0.05^2, 0.125^2]); % [m, m/s, m/s^2], initial state error covariance
ctrl.P0_kalman_no_position = diag([0.05^2, 0.125^2]); % [m/s, m/s^2], initial state error covariance
ctrl.P0_kalman_vertical = diag([0.015^2, 0.05^2, 0.125^2]); % [m, m/s, m/s^2], initial state error covariance

%% Raibert Control

ctrl.F_thrust = ctrl.m_total*ctrl.g*ctrl.weight_factor;
ctrl.com_x_vel_desired = 0.8;
ctrl.kp_theta = 150;
ctrl.kd_theta = 50;
ctrl.z_com_desired = ctrl.z_swing_target - 0.15;
ctrl.torso_pitch_desired = 0;
ctrl.kp_l = 3000;
ctrl.kd_l = 100;
ctrl.kp_x = 0;
ctrl.kd_x = 0;