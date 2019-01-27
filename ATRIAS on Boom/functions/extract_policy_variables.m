function [ structure ] = extract_policy_variables(filename)

  policy = load(filename);
  structure.model.m = double(policy.model.m);
  structure.model.l0 = double(policy.model.l0);
  structure.model.g = double(policy.model.g);
  structure.min_angle = double(policy.angle_bounds(1));
  structure.max_angle = double(policy.angle_bounds(2));
  structure.desired_x_velocity = policy.target_states{policy.target_idx}(2);
  structure.bins.fa = policy.bins{1};
  structure.bins.ssi = policy.bins{2};
  structure.bins.ssa = policy.bins{3};
  structure.angle.fa = policy.control{1,1};
  structure.angle.ssi = zeros(policy.bins{2}.n_divisions);
  structure.angle.ssa = policy.control{3,1};
  
  structure.com_traj = policy.com_traj;
  
%   structure.stiffness.fa = zeros(policy.bins{1}.n_divisions);
%   structure.stiffness.ssi = policy.control{2,2};
%   structure.stiffness.ssa = policy.control{3,2};
%   structure.damping.fa = zeros(policy.bins{1}.n_divisions);
%   structure.damping.ssi = policy.control{2,3};
%   structure.damping.ssa = policy.control{3,3};
  
  
end