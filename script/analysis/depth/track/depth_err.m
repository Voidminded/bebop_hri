function [n, ed, ez] = depth_err(csvfilebase)
% csvfilebase = '2016-02-23-23-08-00';
vicon_data_file = strcat(csvfilebase, '_bebop.csv');
vservo_data_file = strcat(csvfilebase, '_vservo.csv');
target_data_file = strcat(csvfilebase, '_target.csv');

vicon_data = importdata(vicon_data_file, ',', 1);
viconX = vicon_data.data(:,1);
viconY = vicon_data.data(:,2);
viconZ = vicon_data.data(:,3);
r = size(vicon_data.textdata);
vic_t = (str2double(vicon_data.textdata(2:r,3)) * 1.0e-9);

vservo_data = importdata(vservo_data_file, ',', 1);
vservo = vservo_data.data(:, 6);
vservo_time = vservo_data.data(2:size( vservo_data.data), 1) * 1e-9;

target_data = importdata(target_data_file, ',', 1);
targetX = target_data.data(:,1);
targetY = target_data.data(:,2);
r = size(target_data.textdata);
target_t = (str2double(target_data.textdata(2:r,3)) * 1.0e-9);

viconX = viconX(vic_t > min(vservo_time));
viconY = viconY(vic_t > min(vservo_time));
viconZ = viconZ(vic_t > min(vservo_time));
vic_t = vic_t(vic_t > min(vservo_time));
viconX = viconX(vic_t < max(vservo_time));
viconY = viconY(vic_t < max(vservo_time));
viconZ = viconZ(vic_t < max(vservo_time));
vic_t = vic_t(vic_t < max(vservo_time));

targetX = targetX(target_t > min(vservo_time));
targetY = targetY(target_t > min(vservo_time));
target_t = target_t(target_t > min(vservo_time));
targetX = targetX(target_t < max(vservo_time));
targetY = targetY(target_t < max(vservo_time));
target_t = target_t(target_t < max(vservo_time));

vic_t = vic_t - vservo_time(1,1);
target_t = target_t - vservo_time(1,1);
vservo_time = vservo_time - vservo_time(1,1);

vicon_X_ts = timeseries(viconX, vic_t);
vicon_Y_ts = timeseries(viconY, vic_t);
target_X_ts = timeseries(targetX, target_t);
target_Y_ts = timeseries(targetY, target_t);

[vicon_X_ts, target_X_ts] = synchronize(vicon_X_ts, target_X_ts,  'Union');
[vicon_Y_ts, target_Y_ts] = synchronize(vicon_Y_ts, target_Y_ts,  'Union');

ground_truth(:,1) = vicon_X_ts.Data - target_X_ts.Data;
ground_truth(:,2) = vicon_Y_ts.Data - target_Y_ts.Data;

vservo_ts = timeseries(vservo(2:end,:)+0.09, vservo_time);

vic = sqrt(ground_truth(:,1).^2 + ground_truth(:,2).^2);
vic_ts = timeseries(vic, vicon_X_ts.Time);

[vic_ts, vservo_ts] = synchronize(vic_ts, vservo_ts,  'Union');

error = (vservo_ts.Data-vic_ts.Data);

% figure, plot(vic_ts.Time, vic_ts.Data, 'b', vservo_ts.Time, vservo_ts.Data, 'g', vic_ts.Time, error, 'r'), legend('Ground Truth', 'Estimation', 'Error');

ed = sqrt( sum( error.^2)/length(error));
ez = viconZ(end);
n = [];
for i = 1:50:length(vic_ts.Data)-1
    n(length(n)+1,:) = [vic_ts.Data(i), abs(vic_ts.Data(i) - vservo_ts.Data(i))];
end