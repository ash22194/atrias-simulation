function [ground_height_map, grid_space] = create_rough_terrain(random_seed, max_terrain_step, terrain_width_in_meters, terrain_length_in_meters)

    % Terrain parameters
    mesh_length = round((terrain_length_in_meters/5)*257);
    if mod(mesh_length,2) == 0, mesh_length = mesh_length + 1; end
    mesh_width = round((mesh_length / terrain_length_in_meters) * terrain_width_in_meters);
    if mod(mesh_width,2) == 0, mesh_width = mesh_width + 1; end
    initial_terrain_height = 0;
    initial_max_terrain_step = max_terrain_step;
    max_terrain_step_step = 0.01;

    % Generate terrain
    rng(random_seed);
    n = 7;             % Number of iterations
    n0 = 10;            % Number of initial points
    m  = 3;            % How many points grow from each old point
    nf = n0 * (m+1)^n; % Total number of points

    % Create initial x, y, and height coordinates and roughness map.
    x = [randn(n0, 1);           zeros(nf-n0, 1)];
    y = [randn(n0, 1);           zeros(nf-n0, 1)];
    h = [initial_max_terrain_step * randn(n0, 1) + initial_terrain_height; zeros(nf-n0, 1)];
    r = [max_terrain_step_step * randn(n0, 1) + initial_max_terrain_step; zeros(nf-n0, 1)];

    % Create new points from old points n times.
    for k = 1:n

        % Calculate the new variance for the x, y random draws and for the
        % h, r random draws.
        dxy = (0.75^k);
        dh  = 0.5^k;

        % Number of new points to generate
        n_new = m * n0;

        % Parents for new points
        parents = reshape(repmat(1:n0, m, 1), [n_new, 1]);

        % Calculate indices for new and existing points.
        new = (n0+1):(n0+n_new);
        old = 1:n0;

        % Generate new x/y values.
        theta  = 2*pi * rand(n_new, 1);
        radius = dxy * (rand(n_new, 1) + 1);
        x(new) = x(parents) + radius .* cos(theta);
        y(new) = y(parents) + radius .* sin(theta);

        % Interpolate to find nominal new r and h values and add noise to
        % roughness and height maps.
        r(new) =   interpolate(x(old), y(old), r(old), x(new), y(new)) ...
                 + (dh * max_terrain_step_step) .* randn(n_new, 1);
        h(new) =   interpolate(x(old), y(old), h(old), x(new), y(new)) ...
                 + (dh/dxy) * radius .* r(new) .* randn(n_new, 1);
             
        % Add up the new points.
        n0 = n_new + n0;

    end

    % Normalize the distribution of the points about the median.
    x = (x - median(x))/std(x);
    y = (y - median(y))/std(y);

    % Remap points to terrain size
    range = max(x) - min(x);
    x = (x - min(x)) / range;
    x = x*2*terrain_length_in_meters - terrain_length_in_meters;
    range = max(y) - min(y);
    y = (y - min(y)) / range;
    y = y*2*terrain_length_in_meters - terrain_length_in_meters;
    
    
    % Create mesh
    grid_length_vector = linspace(-terrain_length_in_meters, terrain_length_in_meters, mesh_length);
    grid_width_vector = linspace(-terrain_width_in_meters, terrain_width_in_meters, mesh_width);
    [xm, ym] = meshgrid(grid_length_vector, grid_width_vector);
    grid_space = [grid_length_vector(2) - grid_length_vector(1); grid_width_vector(2) - grid_width_vector(1)];
    ground_height_map = interpolate(x, y, h, xm, ym);
    
    
    % Add hills
    hill_height = 1;
    hill_width = 3;
    ground_height_map = ground_height_map + hill_height*exp(-(xm.^2 + 0*ym.^2)/(2*hill_width^2));
    
%     number_of_hills = randi(5);
%     for i=1:number_of_hills
%         x_hill_center = min(x) + (max(x)-min(x))*rand;
%         y_hill_center = min(y) + (max(y)-min(y))*rand;
%         hill_height = 1 + 1*rand;
%         hill_width = 1 + 3*rand;
%         ground_height_map = ground_height_map + hill_height*exp(-((xm-x_hill_center).^2 + (ym-y_hill_center).^2)/(2*hill_width^2));
%     end
%     
    
    ground_height_map = 0*ground_height_map; % flat ground
    
    % Write to file
    stlwrite('terrain.stl', xm, ym, ground_height_map);

end


% Perform our particular type of interpolation.
function vn = interpolate(x0, y0, v0, xn, yn)

    % Make an interpolator for height or roughness. We'll add safe far-away
    % points so we'll always be interpolating and not extrapolating.
    int = TriScatteredInterp([100*[-1 -1 1 1]'; x0], ...
                             [100*[-1 1 -1 1]'; y0], ...
                             [zeros(4, 1);      v0], 'linear'); 

	% Perform the actual interpolation.
	vn = int(xn, yn);

end

