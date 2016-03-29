
close all;
clear all;
%%
track_files = importdata('./track/files.txt');
detect_files = importdata('./detect/files.txt');
subplot(1,2,2) , hold on;
track_z_err = [];
track_d_err = [];
for i=1:length(track_files)
    fprintf('%s\n', char(track_files(i)));
    [track_z_err(i), track_d_err(i)] = target_drawer( strcat('./track/',  char(track_files(i))), 1);
end
view(0,90)
[X, Y, Z] = sphere(300);
X = X * 2.5;
Y = Y * 2.5;
Z = Z * 2.5;
X = X(140:160,170:282);
Y = Y(140:160,170:282);
Z = Z(140:160,170:282);
surf(X, Y, Z,'edgecolor', 'none');
colormap([0  0  0; 0  0  0]);
grid on, axis equal, ylim([-0.5 4]), xlim([-4 4]);
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
detect_z_err = [];
detect_d_err = [];
subplot(1,2,1) , hold on;
for i=1:length(detect_files)
    fprintf('%s\n', char(detect_files(i)));
    [detect_z_err(i), detect_d_err(i)] = target_drawer( strcat('./detect/',  char(detect_files(i))),2);
end
view(0,90)
surf(X, Y, Z,'edgecolor', 'none');
colormap([0  0  0; 0  0  0]);
grid on, axis equal, ylim([-0.5 4]), xlim([-4 4]);
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
set(gca, 'LooseInset', get(gca, 'TightInset'));

fprintf('Track distance RMS: %g\n', sqrt( sum( track_d_err.^2)/length(track_d_err)));
fprintf('Track height RMS: %g\n', sqrt( sum( track_z_err.^2)/length(track_z_err)));
fprintf('Detect distance RMS: %g\n', sqrt( sum( detect_d_err.^2)/length(detect_d_err)));
fprintf('Detect height RMS: %g\n', sqrt( sum( detect_z_err.^2)/length(detect_z_err)));

% set(gcf,'NextPlot','add');
% axes;
% h = title('Final Position');
% set(gca,'Visible','off');
% set(h,'Visible','on');
% set(gcf, 'PaperSize', [6.25 7.5]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 6.25 7.5]);
% 
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [6.25 7.5]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 6.25 7.5]);
% 
% set(gcf, 'renderer', 'painters');
% print(gcf, '-dpdf', 'my-figure.pdf');