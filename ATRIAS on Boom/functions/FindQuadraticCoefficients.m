%% Solve for coefficients of quadratic 
% x = a*t^2 + b*t + c
% dx/dt = 2*a*t + b
function [ a, b, c ] = FindQuadraticCoefficients( x0_pos, xf_pos, xf_vel, t_final )

  % boundary conditions:
  % x0 = c  
  % xf = a*t_final^2 + b*t_final + c
  % dxf/dt = 2*a*t_final + b 
  c = x0_pos;
  a = (c - xf_pos + xf_vel*t_final) / t_final^2;
  b = -(2*c - 2*xf_pos + xf_vel*t_final) / t_final;
  
end

