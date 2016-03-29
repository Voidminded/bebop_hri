function [z , d] = target_drawer(csvfilebase, color)
%     csvfilebase = './track/2016-02-23-23-04-04';
%     color = 1;
    bebop_data_file = strcat(csvfilebase, '_bebop_trimmed.csv');
    target_data_file = strcat(csvfilebase, '_target_trimmed.csv');
    
    bebop_data = importdata(bebop_data_file, ',', 1);
    target_data = importdata(target_data_file, ',', 1);
    
    if color == 1 
        plot( bebop_data.data(end,2)-target_data.data(end,2), bebop_data.data(end,3)-target_data.data(end,3), '.b', 'MarkerSize', 24, 'linestyle', 'none')
    else
        plot( bebop_data.data(end,2)-target_data.data(end,2), bebop_data.data(end,3)-target_data.data(end,3), '.g', 'MarkerSize', 24, 'linestyle', 'none')
    end
    plot( target_data.data(end,2)-target_data.data(end,2), target_data.data(end,3)-target_data.data(end,3), 'xr', 'MarkerSize', 12,'linestyle', 'none'), legend('Final Position', 'Target'), legend boxoff 
    z = bebop_data.data(end,4) - target_data.data(end,4)
    d = sqrt((bebop_data.data(end,2)-target_data.data(end,2))^2 + (bebop_data.data(end,3)-target_data.data(end,3))^2 + (bebop_data.data(end,4)-target_data.data(end,4))^2 )-2.5
%end