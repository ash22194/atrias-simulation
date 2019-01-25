%% Lookup motion plan for the CoM at the start of a single support Poincare section
% state = [x, dx/dt, z, dz/dt]
% touchdown = true if leg will touchdown, false otherwise
function [ state_plan, accel_plan, time_plan, angle_of_attack, touchdown_time, touchdown, takeoff_leg ] = ...
          FindOnlineComPlan( initial_state, initial_section, prev_phase, delayed_state, time_step, max_iterations, ctrl, policy)
  % Nothing to be done for flight apex Poincare section
  if initial_section == 1
    state_plan = zeros(1,4);
    accel_plan = zeros(1,2);
    time_plan = zeros(1,1);
    angle_of_attack = FindTouchdownAngle(initial_state, initial_section, policy);
    touchdown_time = 0;
    touchdown = false;
    takeoff_leg = 1;
    return;
  end
  % preallocate outputs
  time_plan = zeros(max_iterations, 1);
  state_plan = zeros(max_iterations, 4);
  accel_plan = zeros(max_iterations, 2);
  angle_of_attack = 0;
  touchdown_time = 0;
  touchdown = false;
  takeoff_leg = 1;
  % Simulate from a single support Poincare section (until takeoff or next SSI)
  iteration = 1;
  time = 0;
  section = initial_section;
  state = initial_state;
  rest_length = policy.model.l0;
  % Simulate single support init (SSI,2) until single support apex (SSA,3) or takeoff (Flight,1)
  if section == 2
    if prev_phase == 0
      [stiffness, damping] = FindStiffnessDamping(delayed_state, section, policy); 
    else
      [stiffness, damping] = FindStiffnessDamping(state, section, policy); 
    end
    integrand_ssi = @(s) SmmDynamics( s, 1, 0, rest_length, stiffness, damping, policy.model);
    % populate initial point
    if iteration == 1
      state_plan(iteration,:) = state;
      time_plan(iteration) = time;
      slope = integrand_ssi(state);
      accel_plan(iteration,:) = slope([2, 4]);
    end
    done = false;
    while(~done)
      % Save pre-step variables
      prev_z_vel = state(4);
      % integrate 1 time step
      iteration = iteration + 1;
      delta_t = time_step;
      next_state = RungeKutta2( integrand_ssi, state, delta_t );
      slope = integrand_ssi(next_state);
      % check for z apex event
      if (prev_z_vel > -0.05 && next_state(4) <= 0)
        timings = (ctrl.zero_crossing_dt:ctrl.zero_crossing_dt:time_step-ctrl.zero_crossing_dt)';
        next_states = [RungeKutta2Vectorized( integrand_ssi, repmat(state,numel(timings),1), timings); next_state];
        % pick closest successor state
        [~,best_idx] = min(abs(next_states(:,4)),[],1);
        delta_t = best_idx*ctrl.zero_crossing_dt;
        next_state = next_states(best_idx,:);
        slope = integrand_ssi(next_state);
        % transition to section #3 (SSA)
        touchdown = true;
        section = 3*ones(1,'like',section);
        done = true;
      end
      % check for leg takeoff event
      leg_1_vel_sign = sign(next_state(1)*next_state(2) + next_state(3)*next_state(4));
      leg_1_pos = sqrt(next_state(1)^2 + next_state(3)^2);
      % leg_1_vel = (next_state(1)*next_state(2) + next_state(3)*next_state(4)) / leg_1_pos;
      if (leg_1_vel_sign > 0 && next_state(4) > 0.1  && leg_1_pos > policy.model.l0) % leg 1 takeoff 
        timings = (ctrl.zero_crossing_dt:ctrl.zero_crossing_dt:time_step-ctrl.zero_crossing_dt)';
        next_states = [RungeKutta2Vectorized( integrand_ssi, repmat(state,numel(timings),1), timings); next_state];
        % pick closest successor state
        leg_lengths = sqrt(next_states(:,1).^2 + next_states(:,3).^2);
        [~,best_idx] = min(abs(leg_lengths - policy.model.l0),[],1);
        delta_t = best_idx*ctrl.zero_crossing_dt;
        next_state = next_states(best_idx,:);
        slope = integrand_ssi(next_state);
        % transition to section #1 (Flight)
        touchdown = false;
        section = 1*ones(1,'like',section);
        done = true;
      end
      % update time and state
      time = time + delta_t;
      state = next_state;
      % add to output
      state_plan(iteration,:) = state;
      time_plan(iteration) = time;
      accel_plan(iteration,:) = slope([2, 4]);
      % check counter
      if iteration >= max_iterations
        break;
      end
    end
  end
  % Simulate single support apex (SSA,3) until single support init (SSI,2)
  if section == 3 && iteration < max_iterations
    % look up control variables
    angle_of_attack = FindTouchdownAngle(state, section, policy);
    [stiffness, damping] = FindStiffnessDamping(state, section, policy);

    % apex to double support (single support)
    integrand_ssa = @(s) SmmDynamics( s, 1, 0, rest_length, stiffness, damping, policy.model);
    foot2ground_dist = state(3) - sind(angle_of_attack)*policy.model.l0;
    done = false;
    while(~done)
      % update iteration number and time step
      iteration = iteration + 1;
      delta_t = time_step;
      % Save pre-step variables
      prev_foot2ground_dist = foot2ground_dist;
      % integrate 1 time step
      next_state = RungeKutta2( integrand_ssa, state, delta_t );
      slope = integrand_ssa(next_state);
      % check for foot touchdown
      foot2ground_dist = next_state(3) - sind(angle_of_attack)*policy.model.l0;
      if (prev_foot2ground_dist > -0.01 && foot2ground_dist <= 0)        
        timings = (ctrl.zero_crossing_dt:ctrl.zero_crossing_dt:time_step-ctrl.zero_crossing_dt)';
        next_states = [RungeKutta2Vectorized( integrand_ssa, repmat(state,numel(timings),1), timings); next_state];
        % pick closest successor state
        [~,best_idx] = min(abs(next_states(:,3) - sind(angle_of_attack)*policy.model.l0),[],1);
        delta_t = best_idx*ctrl.zero_crossing_dt;
        next_state = next_states(best_idx,:);
        slope = integrand_ssa(next_state);
        touchdown = true;
        touchdown_time = time + delta_t;
        done = true;
      end
      % update time and state
      time = time + delta_t;
      state = next_state;
      % add to output
      state_plan(iteration,:) = state;
      time_plan(iteration) = time;
      accel_plan(iteration,:) = slope([2, 4]);
      % check counter
      if iteration >= max_iterations
        break;
      end
    end
    
    % double support to takeoff
    d_feet = -policy.model.l0*cosd(angle_of_attack) - state(1);
    integrand_ds = @(s) SmmDynamics( s, 2, d_feet, rest_length, stiffness, damping, policy.model);
    done = false || iteration >= max_iterations;
    while(~done)
      % update iteration number and time step
      iteration = iteration + 1;
      delta_t = time_step;
      % integrate 1 time step
      next_state = RungeKutta2( integrand_ds, state, delta_t );
      slope = integrand_ds(next_state);
      % check for leg takeoff event
      leg_1_vel_sign = sign(next_state(1)*next_state(2) + next_state(3)*next_state(4));
      leg_1_pos = sqrt(next_state(1)^2 + next_state(3)^2);
      if (leg_1_vel_sign > 0 && leg_1_pos > policy.model.l0) % leg 1 takeoff
        timings = (ctrl.zero_crossing_dt:ctrl.zero_crossing_dt:time_step-ctrl.zero_crossing_dt)';
        next_states = [RungeKutta2Vectorized( integrand_ds, repmat(state,numel(timings),1), timings); next_state];
        % pick closest successor state
        leg_lengths = sqrt(next_states(:,1).^2 + next_states(:,3).^2);
        [~,best_idx] = min(abs(leg_lengths - policy.model.l0),[],1);
        delta_t = best_idx*ctrl.zero_crossing_dt;
        next_state = next_states(best_idx,:);
        slope = integrand_ds(next_state);
        % transition to section #2 (SSI)
        section = 2*ones(1,'like',section);
        done = true;
        takeoff_leg = 1;
      end
      leg_2_vel_sign = sign((next_state(1)+d_feet)*next_state(2) + next_state(3)*next_state(4));
      leg_2_pos = sqrt((next_state(1)+d_feet)^2 + next_state(3)^2);
      if (leg_2_vel_sign > 0 && leg_2_pos > policy.model.l0) % leg 2 takeoff
        timings = (ctrl.zero_crossing_dt:ctrl.zero_crossing_dt:time_step-ctrl.zero_crossing_dt)';
        next_states = [RungeKutta2Vectorized( integrand_ds, repmat(state,numel(timings),1), timings); next_state];
        % pick closest successor state
        leg_lengths = sqrt((next_states(:,1)+d_feet).^2 + next_states(:,3).^2);
        [~,best_idx] = min(abs(leg_lengths - policy.model.l0),[],1);
        delta_t = best_idx*ctrl.zero_crossing_dt;
        next_state = next_states(best_idx,:);
        slope = integrand_ds(next_state);
        % transition to section #2 (SSI)
        section = 2*ones(1,'like',section);
        done = true;
        takeoff_leg = 2;
      end
      % update time and state
      time = time + delta_t;
      state = next_state;
      % add to output
      state_plan(iteration,:) = state;
      time_plan(iteration) = time;
      accel_plan(iteration,:) = slope([2, 4]);
      % check counter
      if iteration >= max_iterations
        break;
      end
    end
  end
  % Lookup next angle of attack if entering flight
  if section == 1
    angle_of_attack = FindTouchdownAngle(state, section, policy);
  end
  % Prevent touchdown at t=0 (Double support did not occur!)
  if touchdown_time == 0
    touchdown_time = time;
  end
  % Trim down plan outputs (variable size)
  time_plan = time_plan(1:iteration);
  state_plan = state_plan(1:iteration,:);
  accel_plan = accel_plan(1:iteration,:);
end

