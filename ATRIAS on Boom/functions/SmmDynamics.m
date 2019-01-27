%% State derivative for spring mass model dynamics
% state = [x, dx/dt, z, dz/dt]
function [ state_derivative ] = SmmDynamics( state, phase, d_feet, rest_length, stiffness, damping, model)

  % Unpack state vector
  x1_pos = state(:,1);
  x_vel = state(:,2);
  z_pos = state(:,3);
  z_vel = state(:,4);
  n_states = size(state,1);

  % Phase dependent dynamics
  switch(phase)
    case 0 % Flight
      % com dynamics
      state_derivative = [x_vel, zeros(n_states,1), z_vel, repmat(-model.g,n_states,1)];
    case 1 % Single Support
      % leg 1 dynamics
      l1_pos = max(sqrt(z_pos.^2 + x1_pos.^2),eps);
      l1_vel = (z_pos.*z_vel + x1_pos.*x_vel) ./ l1_pos;
      l1_accel = max(model.g/model.l0 * stiffness.*max(rest_length - l1_pos,0) + ...
                  sqrt(model.g/model.l0)/model.l0 * damping*l1_vel.*max(rest_length - l1_pos,0), 0);
      % com dynamics
      x_accel = l1_accel.*x1_pos./l1_pos;
      z_accel = l1_accel.*z_pos./l1_pos - model.g;
      state_derivative = [x_vel, x_accel, z_vel, z_accel];
    case 2 % Double Support
      % leg 1 dynamics
      l1_pos = max(sqrt(z_pos.^2 + x1_pos.^2),eps);
      l1_vel = (z_pos.*z_vel + x1_pos.*x_vel) ./ l1_pos;
      l1_accel = max(model.g/model.l0 * stiffness.*max(rest_length - l1_pos,0) + ...
                  sqrt(model.g/model.l0)/model.l0 * damping*l1_vel.*max(rest_length - l1_pos,0), 0);
      % leg 2 dynamics
      x2_pos = x1_pos + d_feet;
      l2_pos = max(sqrt(z_pos.^2 + x2_pos.^2),eps);
      l2_vel = (z_pos.*z_vel + x2_pos.*x_vel) ./ l2_pos;
      l2_accel = max(model.g/model.l0 * stiffness.*max(rest_length - l2_pos,0) + ...
                  sqrt(model.g/model.l0)/model.l0 * damping*l2_vel.*max(rest_length - l2_pos,0), 0);
      % com dynamics
      x_accel = l1_accel.*x1_pos./l1_pos + l2_accel.*x2_pos./l2_pos;
      z_accel = l1_accel.*z_pos./l1_pos + l2_accel.*z_pos./l2_pos - model.g;
      state_derivative = [x_vel, x_accel, z_vel, z_accel];
    otherwise
      state_derivative = zeros(n_states, 4);
  end

end