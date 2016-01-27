function a_n = normal_angle(a)
  a_np = mod(mod(a, 2*pi) + 2*pi, 2*pi);
  if (a_np > pi)
    a_np = a_np - 2*pi;
  end
  a_n = a_np;
end

