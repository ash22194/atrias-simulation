%% Perform second order Runge-Kutta integration
function x_next = RungeKutta2( integrand, x_initial, delta_t )

% Heun's Method
%   slope_left = integrand(x_initial);
%   slope_right = integrand(x_initial + delta_t*slope_left);
%   x_next = x_initial + 0.5*delta_t*(slope_left + slope_right);
  
  % Ralston's method
  k1 = integrand(x_initial);
  k2 = integrand(x_initial + (2/3)*delta_t.*k1);
  x_next = x_initial + delta_t.*(0.25.*k1 + 0.75.*k2);
  
end