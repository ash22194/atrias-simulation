function value = Interpolate(bin_idx, section, grid)
  
  % interpolate inside a section grid
  switch(section)
    case 1 % Flight apex section
      value = double(interpn(grid.fa,bin_idx(1),bin_idx(2),'nearest'));
    case 2 % Single support init section
      value = double(interpn(grid.ssi,bin_idx(1),bin_idx(2),bin_idx(3),bin_idx(4),'nearest'));
    case 3 % Single support apex section
      value = double(interpn(grid.ssa,bin_idx(1),bin_idx(2),bin_idx(3),'nearest'));
    otherwise
      value = 0;
  end
    
end