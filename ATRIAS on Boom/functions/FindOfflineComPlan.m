%% Lookup motion plan for the CoM at a single support Poincare section
% state = [x, dx/dt, z, dz/dt]
% to_double_support = true if leg will touchdown, false otherwise
function [ state_plan, accel_plan, time_plan, angle_of_attack, touchdown_time, to_double_support, takeoff_leg, neutral_point, k_placement] = ...
          FindOfflineComPlan( initial_state, initial_section, prev_phase, delayed_state, policy)
  % Initialize some outputs
  neutral_point = 0;
  k_placement = 0;
  takeoff_leg = 1; % Takeoff primary leg (if taking off)
  % Nothing to be done for flight apex Poincare section
  if initial_section == 1
    state_plan = zeros(1,4);
    accel_plan = zeros(1,2);
    time_plan = zeros(1,1);
    angle_of_attack = FindTouchdownAngle(initial_state, initial_section, policy);
    touchdown_time = 0;
    to_double_support = false;
    return;
  end
  persistent p_neutral_point
  if isempty(p_neutral_point)
    p_neutral_point = 0;
  end
  % Look up closest valid trajectory
%   persistent prev_final_state
%   if isempty(prev_final_state) || prev_phase == 0
     closest_bin_idx = FindClosestBinIdx(initial_state, initial_section, policy);
%     prev_final_state = initial_state;
%   else
%     if any(abs(initial_state - prev_final_state) > [0.05 0.2 0.05 0.2])
%       prev_final_state = initial_state;
%     end
%     closest_bin_idx = FindClosestBinIdx(prev_final_state, initial_section, policy);
%   end
  % Find state and accel plans
  if initial_section == 2 % SSI
    %closest_bin_idx = 20756;
    % Use valid point mapping to override invalid points
    closest_bin_idx = policy.com_traj.ssi.valid_map(closest_bin_idx);
    % Use directory to lookup where trajectory is stored
    location = policy.com_traj.ssi.directory(closest_bin_idx,:);
    % Find timings
    time_plan = double(policy.com_traj.ssi.timings(location(1):location(2))');
    % Concatenate state and accel plans
    state_plan = double([ policy.com_traj.ssi.x_pos(location(1):location(2))', ...
                           policy.com_traj.ssi.x_vel(location(1):location(2))', ...
                           policy.com_traj.ssi.z_pos(location(1):location(2))', ...
                           policy.com_traj.ssi.z_vel(location(1):location(2))' ]);
    accel_plan = double([ policy.com_traj.ssi.x_acc(location(1):location(2))', ...
                          policy.com_traj.ssi.z_acc(location(1):location(2))' ]);
    to_double_support = policy.com_traj.ssi.to_double_support(closest_bin_idx);
    % Lookup touchdown time and angle if planning a double support phase
    if to_double_support
      closest_final_bin_idx = FindClosestBinIdx(state_plan(end,:), 3, policy);
      closest_final_bin_idx = policy.com_traj.ssa.valid_map(closest_final_bin_idx);
      next_double_support_time = policy.com_traj.ssa.touchdown_times(closest_final_bin_idx);
      touchdown_time = time_plan(end) + next_double_support_time;
      angle_of_attack = policy.angle.ssa(closest_final_bin_idx);
      % Use directory to lookup where trajectory is stored
      location = policy.com_traj.ssa.directory(closest_final_bin_idx,:);
      % Find next section timings and x velocity plan
      next_time_plan = double(policy.com_traj.ssa.timings(location(1):location(2))');
      next_x_vel_plan = double(policy.com_traj.ssa.x_vel(location(1):location(2))');
      % Compute linear feedback for leg placement
      com_x_vel_expected = interp1(next_time_plan, next_x_vel_plan, next_double_support_time, 'linear');
      x_target = -policy.model.l0 * cosd(angle_of_attack);
      p_neutral_point = - 0.5 * mean([state_plan(:,2); next_x_vel_plan]) * (time_plan(end) + next_time_plan(end));
      k_placement = ( x_target - p_neutral_point ) / ( policy.desired_x_velocity - com_x_vel_expected );
    else % Entering flight
      touchdown_time = time_plan(end);
      angle_of_attack = FindTouchdownAngle(state_plan(end,:), 1, policy);
      com_x_vel_expected = state_plan(end,2);
      x_target = -policy.model.l0 * cosd(angle_of_attack);
      p_neutral_point = - 0.5 * mean(state_plan(:,2)) * time_plan(end);
      k_placement = ( x_target - p_neutral_point ) / ( policy.desired_x_velocity - com_x_vel_expected );
    end
  else % initial_section == 3, SSA
    %closest_bin_idx = 2624;
    % Use valid point mapping to override invalid points
    closest_bin_idx = policy.com_traj.ssa.valid_map(closest_bin_idx);
    % Use directory to lookup where trajectory is stored
    location = policy.com_traj.ssa.directory(closest_bin_idx,:);
    % Find timings
    time_plan = double(policy.com_traj.ssa.timings(location(1):location(2))');
    % Concatenate state and accel plans
    state_plan = double([ policy.com_traj.ssa.x_pos(location(1):location(2))', ...
                           policy.com_traj.ssa.x_vel(location(1):location(2))', ...
                           policy.com_traj.ssa.z_pos(location(1):location(2))', ...
                           policy.com_traj.ssa.z_vel(location(1):location(2))' ]);
    accel_plan = double([ policy.com_traj.ssa.x_acc(location(1):location(2))', ...
                          policy.com_traj.ssa.z_acc(location(1):location(2))' ]);
    to_double_support = true;
    % Lookup touchdown time and angle if planning a double support phase
    touchdown_time = policy.com_traj.ssa.touchdown_times(closest_bin_idx);
    angle_of_attack = policy.angle.ssa(closest_bin_idx);
    % Compute linear feedback for leg placement
    com_x_vel_expected = interp1(time_plan, state_plan(:,2), touchdown_time, 'linear');
    x_target = -policy.model.l0 * cosd(angle_of_attack);
    k_placement = ( x_target - p_neutral_point ) / ( policy.desired_x_velocity - com_x_vel_expected );
  end
  % Store final state of trajectory
  %prev_final_state = state_plan(end,:);
  % Output persistent variables
  neutral_point = p_neutral_point;
end

