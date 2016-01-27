function [A, B, C, D] = parrot_bebop_veltilt_lin_m(Cx, Cy, g, Ts)
% x             [vx;vy]
% y             [vx;vy]
% tilt_rad      [pitch_rad roll_rad]
% d_vx = -Cx * vx + g * tan(pitch_rad)
% d_vy = -Cy * vy - g * tan(roll_rad)

A = -diag([Cx Cy]);
B = diag([g -g]);
C = eye(2);
D = zeros(2,2);

end