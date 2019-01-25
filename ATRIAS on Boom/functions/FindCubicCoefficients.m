%% Solve for coefficients of cubic
% x = a*t^3 + b*t^2 + c*t + d
% dx/dt = 3*a*t^2 + 2*b*t + c
function [ a, b, c, d ] = FindCubicCoefficients( x0_pos, x0_vel, xf_pos, xf_vel, t_final )

  % boundary conditions:
  % x0 = d
  % dx0/dt = c
  % xf = a*t_final^3 + b*t_final^2 + c*t_final + d
  % dxf/dt = 3*a*t_final^2 + 2*b*t_final + c 
  d = x0_pos;
  c = x0_vel;
  a =     (2*d - 2*xf_pos + c*t_final + t_final*xf_vel)/t_final^3;
  b =  -(3*d - 3*xf_pos + 2*c*t_final + t_final*xf_vel)/t_final^2;
  
end

