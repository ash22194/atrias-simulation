%% Perform fourth order Runge-Kutta integration
function [x_next, k1] = RungeKutta4( integrand, x_initial, delta_t )
 
  k1 = integrand(x_initial);
  k2 = integrand(x_initial + 0.5.*delta_t.*k1);
  k3 = integrand(x_initial + 0.5.*delta_t.*k2);
  k4 = integrand(x_initial + delta_t.*k3);
  x_next = x_initial + (delta_t/6).*(k1+2*k2+2*k3+k4);

end