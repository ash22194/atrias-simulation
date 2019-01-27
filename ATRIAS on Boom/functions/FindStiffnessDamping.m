%% Lookup the stiffness & damping for a specified state and Poincare section
function [stiffness,damping] = FindStiffnessDamping(state, section, policy)

  % No stiffnesses or dampings are stored for Poincare section #1 (Flight Apex)
  if section == 1
    stiffness = 0;
    damping = 0;
    return;
  end

  % Lookup stiffness/damping using interpolation
  bin_index = FindBinIndex(state, section, policy.bins, policy.model);
%   if section == 2 && from_flight
%     bin_index(4) = 1;
%   end
  stiffness = Interpolate(bin_index, section, policy.stiffness);
  damping = Interpolate(bin_index, section, policy.damping);
  
end