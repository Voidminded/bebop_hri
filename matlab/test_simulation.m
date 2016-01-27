pitch = 0:0.001:0.1;
vx = 0.0;

out=zeros(size(pitch));
time_vec = 0:0.01:1;
out(1) = 0.0;

i = 2;

% discretize
Ts = 0.01;
Cx = -0.576335778073963;
Ad = exp(-Cx * Ts);
Bd = (1.0 / -Cx) * (Ad - 1) * 9.81;
for t=time_vec(2:end)
    p = pitch(i)
%     dvx = 0.5763 * vx + 9.81 * p;
%     vx = vx + (dvx * 0.01);
%     out(i) = vx;
    out(i) = Ad * out(i-1) + Bd * p;
    i = i + 1;
end

plot(time_vec, pitch, time_vec, out);