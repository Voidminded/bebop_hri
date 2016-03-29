close all;
clear all;
%%
files = dir('./csvs');
[~, sorted] = sort([files.datenum]);
files = files(sorted);
numfiles = numel(files);
In = cell(numfiles,1);
for i = 1:4:numfiles
    if (length( files(i).name) > 3)
        visualize_speed_test2( files(i).name(1:19), 9, 505);
    end
end