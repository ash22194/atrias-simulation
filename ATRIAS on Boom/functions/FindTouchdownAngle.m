%% Lookup the angle of attack for a specified state and Poincare section
function angle = FindTouchdownAngle(state, section, policy)

  % No angles are stored for Poincare section #2 (Initial Single Support)
  if section == 2
    angle = 0;
  else % Lookup angle using interpolation
    bin_index = FindBinIndex(state, section, policy.bins, policy.model);
    angle = Interpolate(bin_index, section, policy.angle);
    angle = max(min(angle, policy.max_angle), policy.min_angle );
  end
  
end