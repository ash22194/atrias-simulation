%% Convert state [x,dx/dt,z,dz/dt],d_feet to policy bin space
function [bin] = StateToBin(state, section, model)

  % Unpack state vector (foot to CoM)
  x1_pos = state(1);
  x_vel = state(2);
  z_pos = state(3);
  z_vel = state(4);

  % Phase dependent conversion
  switch(section)
    case 1 % Flight Apex
      bin = [z_pos, x_vel];
    case 2 % Single support Initial
      leg_angle = atan2d( z_pos, -x1_pos);
      leg_compress = max(model.l0 - sqrt(x1_pos.^2 + z_pos.^2), 0);
      bin = [leg_angle, x_vel, z_vel, leg_compress];
    case 3 % Single support Apex
      leg_angle = atan2d( z_pos, -x1_pos);
      leg_compress = max(model.l0 - sqrt(x1_pos.^2 + z_pos.^2), 0);
      bin = [ leg_angle, x_vel, leg_compress];
    otherwise
      bin = [1, 1]; % dummy values for Simulink compilation
  end

end