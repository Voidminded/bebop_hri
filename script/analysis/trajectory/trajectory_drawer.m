function start = trajectory_drawer(csvfilebase)
%     csvfilebase = './track/2016-02-23-23-04-04';
    bebop_data_file = strcat(csvfilebase, '_bebop_trimmed.csv');
    target_data_file = strcat(csvfilebase, '_target_trimmed.csv');
    bebop_data_notrim_file = strcat(csvfilebase, '_bebop.csv');
    
    bebop_data = importdata(bebop_data_file, ',', 1);
    target_data = importdata(target_data_file, ',', 1);
    
    bebop_data_notrim = importdata(bebop_data_notrim_file, ',', 1);
%     plot( bebop_data.data(:,2), bebop_data.data(:,3)), xlabel('X'), ylabel('Y')
%     hold on, plot( target_data.data(:,2), target_data.data(:,3), 'ro')
    plot3( bebop_data.data(:,2)-target_data.data(end,2), bebop_data.data(:,3)-target_data.data(end,3), bebop_data.data(:,4)-target_data.data(end,4), 'LineWidth',2)
    hold on, plot3( target_data.data(end,2)-target_data.data(end,2), target_data.data(end,3)-target_data.data(end,3), target_data.data(end,4)-target_data.data(end,4), 'xr', 'MarkerSize', 12,'linestyle', 'none')
%     plot3( bebop_data_notrim.data(1,1)-target_data.data(end,2), bebop_data_notrim.data(1,2)-target_data.data(end,3), bebop_data_notrim.data(1,3)-target_data.data(end,4), '*b', 'MarkerSize', 12,'linestyle', 'none')
    start = [bebop_data_notrim.data(1,1)-target_data.data(end,2), bebop_data_notrim.data(1,2)-target_data.data(end,3), bebop_data_notrim.data(1,3)-target_data.data(end,4)];
%end