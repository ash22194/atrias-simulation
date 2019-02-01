function bin_index = FindBinIndex(state, section, bins, model)

  % Pick bin parameters based on section number (Simulink doesn't allow cell arrays)
  switch(section)
    case 1 % Flight apex
      bin_params = bins.fa;
      % (in case state is not at apex) compute flight apex state (dz/dt = 0)
      state(3) = state(3) + state(4)^2 / (2*model.g);
      state(4) = 0;
    case 2 % Single support init
      bin_params = bins.ssi;
    case 3 % Single support apex
      bin_params = bins.ssa;
    otherwise
      bin_index = [1 1]; % dummy values for Simulink compilation
      return;
  end

  % convert state into scaled bin coordinates
  bin = StateToBin(state, section, model);
  bin_index = (bin - bin_params.bounds(1,:)) ./ bin_params.spacing + 1;
  bin_index = max(bin_index, 1); % snap to lower bound
  bin_index = min(bin_index, double(bin_params.n_divisions)); % snap to upper bounds
  
end