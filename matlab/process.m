% max roll and pitch angles are 20 degrees for 2016-01-18 dataset
% first final set of data N=5, F=303

do_reimport = true;
do_initial_graphs = false;

if do_reimport
    clear vic_t vic_pose_w vic_pose_b vic_ypr_w d_vic_t vic_pose_smoothed_b vic_vel_smoothed_b ...
    vic_acc_smoothed_b att_t beb_rpy_b speed_t beb_vel_enu speed_t_sync beb_vel_b cmdvel_t cvel_p_r_vz_vyaw_b beb_vel_enu_ts beb_rpy_b_ts;

    [vic_t, vic_pose_w, vic_pose_b, vic_ypr_w, d_vic_t, vic_pose_smoothed_b, vic_vel_smoothed_b, ...
    vic_acc_smoothed_b, att_t, beb_rpy_b, speed_t, beb_vel_enu, speed_t_sync, beb_vel_b, cmdvel_t, cvel_p_r_vz_vyaw_b, beb_vel_enu_ts, beb_rpy_b_ts] = ...
    import_and_filter_data('2016-01-18-11-56-51', 9, 505);
end

if do_initial_graphs
    plot(vic_t, vic_vel_smoothed_b(:,1), speed_t_sync, beb_vel_b(:, 1));
    legend('Vicon X', 'Bebop X');
    figure;
    plot(vic_t, vic_vel_smoothed_b(:,2), speed_t_sync, beb_vel_b(:, 2));
    legend('Vicon Y', 'Bebop Y');
    create_acc_fig(vic_ypr_w(:,3), vic_acc_smoothed_b(:,2), 'Roll Angle (Rad)' , 'Lateral Accel (m/s^2');
    create_acc_fig(vic_ypr_w(:,2), vic_acc_smoothed_b(:,1), 'Pitch Angle (Rad)' , 'Forward Accel (m/s^2');
end

% for both non-lin and lin models we use 40000 data points for training and
% the rest for validation

% non-linear idnlgrey model turned out to be non-optimal solution for us
% 1. It is quite slow to estimate
% 2. It is not compatible with many of control toolbox functions.
% 3. tan(theta) is almost theta for small values of tilt, therefor using a
% nonlinear state space representation is somehow an overkill.
if false
    fprintf('Estimating parameters of non-linear model ...\n');
    bebop_sys_nonlin = idnlgrey('parrot_bebop_veltilt_m', [2 2 2], [0.4, 0.4, 9.81], [0;0]);
    bebop_sys_nonlin.Parameters(3).Fixed = true;
    bebop_sys_nonlin.InputName = {'pitch', 'roll'};
    bebop_sys_nonlin.InputUnit = {'rad', 'rad'};
    bebop_sys_nonlin.OutputName = {'Forward Velocity', 'Lateral Velocity'};
    bebop_sys_nonlin.OutputUnit = {'m/s', 'm/s'};
    bebop_sys_nonlin.InitialStates(1).Minimum = -pi/2;
    bebop_sys_nonlin.InitialStates(1).Maximum = pi/2;
    bebop_sys_nonlin.InitialStates(2).Minimum = -pi/2;
    bebop_sys_nonlin.InitialStates(2).Maximum = pi/2;
    setpar(bebop_sys_nonlin, 'Minimum', {0, 0, 0});
    setpar(bebop_sys_nonlin, 'Maximum', {5.0, 5.0, 10.0});
    nlopt = nlgreyestOptions('Display','on');

    % bebop_sys_data = iddata(beb_vel_b(:,1), beb_rpy_b_ts.Data(:,2), mean(diff(beb_vel_enu_ts.Time)));
    % bebop_sys_nonlin_refined = nlgreyest(bebop_sys_data(1:2500), bebop_sys_nonlin);
    % compare(bebop_sys_data(2501:end), bebop_sys_nonlin, [], compareOptions('InitialCondition', 'model'));
    
    % vx <-> pitch vy <-> roll
    bebop_sys_data = iddata(vic_vel_smoothed_b(:,1:2), vic_ypr_w(:,2:3), mean(d_vic_t));
    bebop_sys_nonlin_refined = nlgreyest(bebop_sys_data(1:40000,:), bebop_sys_nonlin, nlopt);
    compare(bebop_sys_data(40001:end), bebop_sys_nonlin, [], compareOptions('InitialCondition', 'model'));
end

fprintf('Estimating parameters of a linear model of order 1...\n');

params = {'Cx', 0.4; 'Cy', 0.4; 'g', 9.81};
bebop_sys_lin = idgrey('parrot_bebop_veltilt_lin_m', params, 'c');
bebop_sys_lin.Structure.Parameters(3).Free = false;
bebop_sys_lin.Structure.Parameters(1).Minimum = 0;
bebop_sys_lin.Structure.Parameters(2).Minimum = 0;
bebop_sys_lin.InputName = {'pitch', 'roll'};
bebop_sys_lin.InputUnit = {'rad', 'rad'};
bebop_sys_lin.OutputName = {'Forward Velocity', 'Lateral Velocity'};
bebop_sys_lin.OutputUnit = {'m/s', 'm/s'};
opt = greyestOptions('InitialState','zero','Display','on');
opt.Focus = 'stability';

bebop_sys_data = iddata(vic_vel_smoothed_b(:,1:2), vic_ypr_w(:,2:3), mean(d_vic_t));
bebop_sys_lin_refined = greyest(bebop_sys_data(1:40000), bebop_sys_lin, opt);
compare(bebop_sys_data(40001:end), bebop_sys_lin_refined, [], compareOptions('InitialCondition', 'model'));bebop_sys_lin_refined_plant = idss(bebop_sys_lin_refined);
bebop_sys_lin_refined_plant_vx = bebop_sys_lin_refined_plant(1,1);
bebop_sys_lin_refined_plant_vy = bebop_sys_lin_refined_plant(2,2);


% Great discretization tutorial
% https://en.wikibooks.org/wiki/Control_Systems/Digital_State_Space


% bebop_sys_lin_refined_plant_vy.Ts = 1.0 / 30.0;
% use ss() to convert  the linear mode


% old stuff %

% this is wrong because the accel/roll is a dynamic model, not a state-less
% linear model
%model_acc_y = create_tilt_accel_fit(vic_r_w, vic_accy_smoothed_b, 'Accel Y vs Roll');
%model_acc_x = create_tilt_accel_fit(vic_p_w, vic_accx_smoothed_b, 'Aceel X vs Pitch');

%
%est_accx_b = -0.4 .* vic_vel_smoothed_b(:,1) + 9.81 .* tan(vic_p_w);