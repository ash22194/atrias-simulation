%% Returns the linear index of the closest bin for a given state
function [ closest_idx ] = FindClosestBinIdx( state, section, policy )

  % Not used in flight
  if section == 1 % FA
    closest_idx = 0;
    return;
  end
  % determine current bin coordinate
  current_bin = StateToBin(state, section, policy.model);
  % scale to an index and round
  if section == 2 % SSI
    scaled_bin = (current_bin - policy.bins.ssi.bounds(1,:)) ./ policy.bins.ssi.spacing + 1;
    scaled_bin = round(scaled_bin);
    scaled_bin = max(scaled_bin, double(1));
    scaled_bin = min(scaled_bin, double(policy.bins.ssi.n_divisions));
    closest_idx = sub2ind(policy.bins.ssi.n_divisions, scaled_bin(1), scaled_bin(2), scaled_bin(3), scaled_bin(4));
  else % section == 3, SSA
    scaled_bin = (current_bin - policy.bins.ssa.bounds(1,:)) ./ policy.bins.ssa.spacing + 1;
    scaled_bin = round(scaled_bin);
    scaled_bin = max(scaled_bin, double(1));
    scaled_bin = min(scaled_bin, double(policy.bins.ssa.n_divisions));
    closest_idx = sub2ind(policy.bins.ssa.n_divisions, scaled_bin(1), scaled_bin(2), scaled_bin(3));
  end
  
end