function [dx, y] = parrot_bebop_veltilt_m(t, x, tilt_rad, Cx, Cy, g, varargin)
% x             [vx;vy]
% y             [vx;vy]
% tilt_rad      [pitch_rad roll_rad]
% d_vx = -Cx * vx + g * tan(pitch_rad)
% d_vy = -Cy * vy - g * tan(roll_rad)

% Output equation.
y = x;

% State equations.
dx = -diag([Cx Cy]) * x + diag([g -g]) * tan(tilt_rad)';

end