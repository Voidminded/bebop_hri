function visualize_speed_test2(csvfilebase, sgolay_order, sgolay_win_size)
% csvfilebase = '2016-02-23-19-01-43';
% sgolay_order = 9;
% sgolay_win_size = 505;
vicon_data_file = strcat(csvfilebase, '_vicon.csv');
att_data_file = strcat(csvfilebase, '_att.csv');
speed_data_file = strcat(csvfilebase, '_speed.csv');
cmdvel_data_file = strcat(csvfilebase, '_vservo.csv');

fprintf('Processing %s with SGolay Filter of Order %d and Window Size of %d\n', vicon_data_file, sgolay_order, sgolay_win_size);

[b,g] = sgolay(sgolay_order, sgolay_win_size);   % Calculate S-G coefficients
HalfWin  = ((sgolay_win_size+1)/2) -1;

vicon_data = importdata(vicon_data_file, ',', 1);
r = size(vicon_data.textdata);
vic_t = (str2double(vicon_data.textdata(2:r,3)) * 1.0e-9);

fprintf('Importing input velocity information from %s\n', cmdvel_data_file);

input_data = importdata(cmdvel_data_file, ',', 1);
vservo = input_data.data(:, 2:4);
vservo_time = input_data.data(2:size( input_data.data), 1) * 1e-9;

fprintf('Importing speed information from %s\n', speed_data_file);

speed_data = importdata(speed_data_file, ',', 1);
r = size(speed_data.textdata);
speed_t = str2double(speed_data.textdata(2:r,3)) * 1e-9;


min_time = min([vic_t;vservo_time;speed_t])
vic_t = vic_t - min_time;
vservo_time = vservo_time - min_time;
speed_t = speed_t - min_time;


fprintf('Number of records: %d\n', r);

vic_pose_w = vicon_data.data(:,1:3);
% ros is xyzw, matlab is wxyz
vic_ypr_w = quat2eul([vicon_data.data(:,7) vicon_data.data(:,4:6)]);
d_vic_t = diff(vic_t);
d_vic_t_avg = mean(d_vic_t);
d_vic_t_std = std(d_vic_t);
fprintf('dt Mean: %.6f Std: %.6f\n', d_vic_t_avg, d_vic_t_std);

% map to bebop
vic_pose_b = zeros(size(vic_pose_w));
for i = 1:size(vic_pose_w)
    phi = vic_ypr_w(i, 1);
    rot_mat = [cos(phi), sin(phi); -sin(phi), cos(phi)];
    vic_pose_b(i, 1:2) = (rot_mat * vic_pose_w(i, 1:2).').';
end

vic_pose_smoothed_b = zeros(size(vic_pose_b));
vic_vel_smoothed_b = zeros(size(vic_pose_b));
vic_acc_smoothed_b = zeros(size(vic_pose_b));

fprintf('Smoothing data with SGolay ...\n');

for n = (sgolay_win_size+1)/2:size(vic_t)-(sgolay_win_size+1)/2,
  % Zeroth derivative (smoothing only)
  vic_pose_smoothed_b(n, 1) = dot(g(:,1),vic_pose_b(n - HalfWin:n + HalfWin, 1));
  vic_pose_smoothed_b(n, 2) = dot(g(:,1),vic_pose_b(n - HalfWin:n + HalfWin, 2));
  vic_pose_smoothed_b(n, 3) = dot(g(:,1),vic_pose_b(n - HalfWin:n + HalfWin, 3));

  % 1st differential
  vic_vel_smoothed_b(n, 1) = dot(g(:,2),vic_pose_b(n - HalfWin:n + HalfWin, 1));
  vic_vel_smoothed_b(n, 2) = dot(g(:,2),vic_pose_b(n - HalfWin:n + HalfWin, 2));
  vic_vel_smoothed_b(n, 3) = dot(g(:,2),vic_pose_b(n - HalfWin:n + HalfWin, 3));

  % 2nd differential
  vic_acc_smoothed_b(n, 1) = 2*dot(g(:,3)',vic_pose_b(n - HalfWin:n + HalfWin, 1))';
  vic_acc_smoothed_b(n, 2) = 2*dot(g(:,3)',vic_pose_b(n - HalfWin:n + HalfWin, 2))';
  vic_acc_smoothed_b(n, 3) = 2*dot(g(:,3)',vic_pose_b(n - HalfWin:n + HalfWin, 3))';
end

vic_vel_smoothed_b(2:end,1) = vic_vel_smoothed_b(2:end,1) ./ d_vic_t_avg;
vic_vel_smoothed_b(2:end,2) = vic_vel_smoothed_b(2:end,2) ./ d_vic_t_avg;
vic_vel_smoothed_b(2:end,3) = vic_vel_smoothed_b(2:end,3) ./ d_vic_t_avg;

vic_acc_smoothed_b(2:end,1) = vic_acc_smoothed_b(2:end,1) ./ (d_vic_t_avg^2);
vic_acc_smoothed_b(2:end,2) = vic_acc_smoothed_b(2:end,2) ./ (d_vic_t_avg^2);
vic_acc_smoothed_b(2:end,3) = vic_acc_smoothed_b(2:end,3) ./ (d_vic_t_avg^2);

fprintf('Importing attitude information from %s\n', att_data_file);

att_data = importdata(att_data_file, ',', 1);
r = size(att_data.textdata);
att_t = str2double(att_data.textdata(2:r,3)) * 1e-9;
att_t = att_t - att_t(1,1);
beb_rpy_b = att_data.data(:,1:3);
beb_rpy_b(:,2) = -beb_rpy_b(:,2);
beb_rpy_b(:,3) = -beb_rpy_b(:,3);

% input is in ESD coords (we convert it to ENU)
beb_vel_enu= speed_data.data(:,1:3);
beb_vel_enu(:,2) = -beb_vel_enu(:,2);
beb_vel_enu(:,3) = -beb_vel_enu(:,3);

fprintf('Transforming Speed to Body frame ...\n');

% sync v_enu and rpy timeseries
beb_vel_enu_ts = timeseries(beb_vel_enu, speed_t);
beb_rpy_b_ts = timeseries(beb_rpy_b, att_t);
[beb_vel_enu_ts, beb_rpy_b_ts] = synchronize(beb_vel_enu_ts, beb_rpy_b_ts,  'Union');

beb_vel_b = zeros(size(beb_vel_enu_ts.Data));

for i = 1:size(beb_rpy_b_ts.Data)
    % Do not change the sign again, it has already been applied
    phi = beb_rpy_b_ts.Data(i, 3);
    rot_mat = [cos(phi), sin(phi); -sin(phi), cos(phi)];
    beb_vel_b(i, 1:2) = (rot_mat * beb_vel_enu_ts.Data(i, 1:2).').';
end

vservo_ts = timeseries(vservo(2:end,:), vservo_time);

[beb_vel_enu_ts, vservo_ts] = synchronize(beb_vel_enu_ts, vservo_ts,  'Union');

figure;
subplot(1,2,1), plot(vic_t, vic_vel_smoothed_b(:,1), beb_rpy_b_ts.Time, beb_vel_b(:,1), vservo_ts.Time, vservo_ts.Data(:,1)), title('Vx');
subplot(1,2,2), plot(vic_t, vic_vel_smoothed_b(:,2), beb_rpy_b_ts.Time, beb_vel_b(:,2), vservo_ts.Time, vservo_ts.Data(:,2)), title('Vy');

%
% % Predictor Code
% 
% %tttt_vec = 0:bebop_sys_lin_refined_plant_vx.Ts:0.262;
% tttt_vec = 0:0.001:0.262;
% pred_velx = timeseries();
% for i = 1:size(beb_rpy_b_ts.Data)
%     current_pitch = beb_rpy_b_ts.Data(i, 2);    
%     current_v = [beb_vel_b(i,1), beb_vel_b(i,2)];
% %     vvvv = lsim(bebop_sys_lin_refined_plant_vx, ones(size(tttt_vec)) .* current_pitch, tttt_vec, current_v);
% %     pred_velx = pred_velx.addsample('Data', vvvv(end), 'Time', beb_rpy_b_ts.Time(i));
% 
%    vvvv = current_v(1,1);
%    for t=tttt_vec
%        vvvv = vvvv + (0.001 * ( (0.576335778073963 * vvvv) + (9.81 * tan(current_pitch))));
%    end
%    pred_velx = pred_velx.addsample('Data', vvvv, 'Time', beb_rpy_b_ts.Time(i));
% end
% 
% figure;
% plot(vic_t, vic_vel_smoothed_b(:,1), beb_rpy_b_ts.Time, beb_vel_b(:,1), pred_velx.Time, pred_velx.Data);
% legend('vicon-vx', 'bebop-vx', 'bebop-simulated-vx');
% 
% % end predictor
% 
% speed_t_sync = beb_vel_enu_ts.Time;
% 
% % Error Calculation
% fprintf('Calculating Errors ...\n');
% 
% vic_ypr_w_ts = timeseries(vic_ypr_w, vic_t);
% beb_rpy_b_ts_2 = timeseries(beb_rpy_b, att_t);
% 
% [vic_ypr_w_ts, beb_rpy_b_ts_2] = synchronize(vic_ypr_w_ts, beb_rpy_b_ts_2, 'Union');
% err_roll = angleDiff(vic_ypr_w_ts.Data(:,3), beb_rpy_b_ts_2.Data(:,1));
% err_pitch = angleDiff(vic_ypr_w_ts.Data(:,2), beb_rpy_b_ts_2.Data(:,2));
% 
% figure; plot( vic_ypr_w_ts.Time, vic_ypr_w_ts.Data(:,2), beb_rpy_b_ts_2.Time, beb_rpy_b_ts_2.Data(:,2)), legend('vic','bep'), title('pitch');
% figure; plot( vic_ypr_w_ts.Time, vic_ypr_w_ts.Data(:,3), beb_rpy_b_ts_2.Time, beb_rpy_b_ts_2.Data(:,1)), legend('vic','bep'), title('roll');
% figure; plot( vic_ypr_w_ts.Time, vic_ypr_w_ts.Data(:,1), beb_rpy_b_ts_2.Time, beb_rpy_b_ts_2.Data(:,3)), legend('vic','bep'), title('yaw');
% 
% err_roll_rmse = mean(err_roll .^ 2)^0.5;
% err_pitch_rmse = mean(err_pitch .^ 2)^0.5;
% 
% fprintf('RMSE roll: %0.4f pitch: %0.4f\n', err_roll_rmse, err_pitch_rmse);

 % % input is in ESD coords (we convert it to ENU)
% beb_vel_enu= speed_data.data(:,1:3);
% beb_vel_enu(:,2) = -beb_vel_enu(:,2);
% beb_vel_enu(:,3) = -beb_vel_enu(:,3);
% 
% fprintf('Transforming Speed to Body frame ...\n');
% 
% % sync v_enu and rpy timeseries
% beb_vel_enu_ts = timeseries(beb_vel_enu, speed_t);
% beb_rpy_b_ts = timeseries(beb_rpy_b, att_t);
% [beb_vel_enu_ts, beb_rpy_b_ts] = synchronize(beb_vel_enu_ts, beb_rpy_b_ts,  'Union');
% 
% beb_vel_b = zeros(size(beb_vel_enu_ts.Data));
% 
% for i = 1:size(beb_rpy_b_ts.Data)
%     % Do not change the sign again, it has already been applied
%     phi = beb_rpy_b_ts.Data(i, 3);
%     rot_mat = [cos(phi), sin(phi); -sin(phi), cos(phi)];
%     beb_vel_b(i, 1:2) = (rot_mat * beb_vel_enu_ts.Data(i, 1:2).').';
% end
% 
% % Predictor Code
% 
% %tttt_vec = 0:bebop_sys_lin_refined_plant_vx.Ts:0.262;
% tttt_vec = 0:0.001:0.262;
% pred_velx = timeseries();
% for i = 1:size(beb_rpy_b_ts.Data)
%     current_pitch = beb_rpy_b_ts.Data(i, 2);    
%     current_v = [beb_vel_b(i,1), beb_vel_b(i,2)];
% %     vvvv = lsim(bebop_sys_lin_refined_plant_vx, ones(size(tttt_vec)) .* current_pitch, tttt_vec, current_v);
% %     pred_velx = pred_velx.addsample('Data', vvvv(end), 'Time', beb_rpy_b_ts.Time(i));
% 
%    vvvv = current_v(1,1);
%    for t=tttt_vec
%        vvvv = vvvv + (0.001 * ( (0.576335778073963 * vvvv) + (9.81 * tan(current_pitch))));
%    end
%    pred_velx = pred_velx.addsample('Data', vvvv, 'Time', beb_rpy_b_ts.Time(i));
% end
% 
% figure;
% plot(vic_t, vic_vel_smoothed_b(:,1), beb_rpy_b_ts.Time, beb_vel_b(:,1), pred_velx.Time, pred_velx.Data);
% legend('vicon-vx', 'bebop-vx', 'bebop-simulated-vx');
% 
% % end predictor
% 
% speed_t_sync = beb_vel_enu_ts.Time;
% 
% % Error Calculation
% fprintf('Calculating Errors ...\n');
% 
% vic_ypr_w_ts = timeseries(vic_ypr_w, vic_t);
% beb_rpy_b_ts_2 = timeseries(beb_rpy_b, att_t);
% 
% [vic_ypr_w_ts, beb_rpy_b_ts_2] = synchronize(vic_ypr_w_ts, beb_rpy_b_ts_2, 'Union');
% err_roll = angleDiff(vic_ypr_w_ts.Data(:,3), beb_rpy_b_ts_2.Data(:,1));
% err_pitch = angleDiff(vic_ypr_w_ts.Data(:,2), beb_rpy_b_ts_2.Data(:,2));
% 
% figure; plot( vic_ypr_w_ts.Time, vic_ypr_w_ts.Data(:,2), beb_rpy_b_ts_2.Time, beb_rpy_b_ts_2.Data(:,2)), legend('vic','bep'), title('pitch');
% figure; plot( vic_ypr_w_ts.Time, vic_ypr_w_ts.Data(:,3), beb_rpy_b_ts_2.Time, beb_rpy_b_ts_2.Data(:,1)), legend('vic','bep'), title('roll');
% figure; plot( vic_ypr_w_ts.Time, vic_ypr_w_ts.Data(:,1), beb_rpy_b_ts_2.Time, beb_rpy_b_ts_2.Data(:,3)), legend('vic','bep'), title('yaw');
% 
% err_roll_rmse = mean(err_roll .^ 2)^0.5;
% err_pitch_rmse = mean(err_pitch .^ 2)^0.5;
% 
% fprintf('RMSE roll: %0.4f pitch: %0.4f\n', err_roll_rmse, err_pitch_rmse);
% 
% fprintf('Importing cmd_vel information from %s\n', cmdvel_data_file);
% 
% cmdvel_data = importdata(cmdvel_data_file, ',', 1);
% r = size(cmdvel_data.textdata);
% cmdvel_t = str2double(cmdvel_data.textdata(2:r,3));
% cvel_p_r_vz_vyaw_b = cmdvel_data.data(:,[1:3 6]);

% let's calculate time shift
% I ran this code once to just gather the data, this needs to be
% moved to its own function
% Test on roll values
% 30 0.245754 0.000275
% 31 0.253946 0.000260
% 32 0.262137 0.000257
% 33 0.270329 0.000265
% 34 0.278521 0.000285
% test on pitch
% 28 0.229370 0.000684
% 29 0.237562 0.000648
% 30 0.245754 0.000623
% 31 0.253946 0.000609
% 32 0.262137 0.000605
% 33 0.270329 0.000613
% 34 0.278521 0.000631
% 35 0.286713 0.000659
% 36 0.294905 0.000699
% consensus is 32 delays on vicon data (~ 0.262137ms) is the approx delay
% dt = mean(diff(beb_rpy_b_ts.Time));
% max_delay_test = 0.4;
% current_delay = dt;
% current_delay_offset = 1;
% while current_delay < max_delay_test
%     df = dfilt.delay(current_delay_offset);
%     delayed = filter(df, vic_ypr_w_ts.Data(:,2));
%     err = mean(( beb_rpy_b_ts.Data(:,2) - delayed) .^ 2);
%     s = sprintf('%d %f %f', current_delay_offset, current_delay, err);
%     disp(s);
%     current_delay = current_delay + dt;
%     current_delay_offset = current_delay_offset + 1;
% end