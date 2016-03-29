close all;
clear all;
%%
track_files = importdata('./track/files.txt');
detect_files = importdata('./detect/files.txt');
subplot(1,2,2)% , hold on;
track_starts=[];
for i=1:length(track_files)
    fprintf('%s\n', char(track_files(i)));
    track_starts(i,:) = trajectory_drawer( strcat('./track/',  char(track_files(i))));
end
t_s = [];
t_s(:,1) = mean(track_starts(1:4,:));
t_s(:,2) = mean(track_starts(5:8,:));
t_s(:,3) = mean(track_starts(9:12,:));
t_s(:,4) = mean(track_starts(13:16,:));
t_s(:,5) = mean(track_starts(17:20,:));
plot3( t_s(1,:), t_s(2,:), t_s(3,:), '*k', 'MarkerSize', 12,'linestyle', 'none')
view(0,90)
[X, Y, Z] = sphere(300);
X = X * 2.5;
Y = Y * 2.5;
Z = Z * 2.5;
X = X(140:160,180:270);
Y = Y(140:160,180:270);
Z = Z(140:160,180:270);
surf(X, Y, Z,'edgecolor', 'none');
colormap([0  0  0; 0  0  0]);
grid on, axis equal, ylim([-1 9]), xlim([-3 3]);
alpha( 0.3);
hXLabel = xlabel('X (m)');
hYLabel = ylabel('Y (m)');
set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([ hXLabel, hYLabel], ...
    'FontName'   , 'AvantGarde');
set([ gca]             , ...
    'FontSize'   , 8           );
set([hXLabel, hYLabel]  , ...
    'FontSize'   , 10          );
set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
   'LineWidth'   , 1         );
set(gcf, 'PaperPositionMode', 'auto');

subplot(1,2,1) %, hold on;
detect_starts = [];
for i=1:length(detect_files)
    fprintf('%s\n', char(detect_files(i)));
    detect_starts(i,:) = trajectory_drawer( strcat('./detect/',  char(detect_files(i))));
    waitforbuttonpress
end
d_s = [];
d_s(:,1) = mean(detect_starts(1:4,:));
d_s(:,2) = mean(detect_starts(5:8,:));
d_s(:,3) = mean(detect_starts(9:12,:));
d_s(:,4) = mean(detect_starts(13:17,:));
d_s(:,5) = mean(detect_starts(18:21,:));
plot3( d_s(1,:), d_s(2,:), d_s(3,:), '*k', 'MarkerSize', 12,'linestyle', 'none')
grid on, axis equal, ylim([-1 9]), xlim([-3 3]);
view(0,90)
surf(X, Y, Z,'edgecolor', 'none');
colormap([0  0  0; 0  0  0]);
alpha( 0.3);
hXLabel = xlabel('X (m)');
hYLabel = ylabel('Y (m)');
set( gca                       , ...
    'FontName'   , 'Helvetica' );
set([hXLabel, hYLabel], ...
    'FontName'   , 'AvantGarde');
set([ gca]             , ...
    'FontSize'   , 8           );
set([hXLabel, hYLabel]  , ...
    'FontSize'   , 10          );
set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
   'LineWidth'   , 1         );
set(gcf, 'PaperPositionMode', 'auto');