%% Description
%
% Simulate an UAV trajectory and plot the orientation and position errors,
% in addition to the estimated and real position and orientation.
%
% It is possible to activate and deactivate the various sensors, and to
% change on the fly their sampling rate.


%% Initialization

clc
clear variables
close all

% Add the folder and the sub-folders to MATLAB path
addpath(genpath('custom_functions'))
addpath(genpath('datasets'))


%% Simulation Setup

% Sensors frequencies [Hz]
imuFs = 160;    % IMU (accelerometer and gyroscope)
magFs = 40;     % magnetometer
gpsFs = 5;      % GPS
uwbFs = 40;     % Ultra Wide Band
visFs = 5;      % vision system (ArUco marker)
barFs = 1;      % barometer
altFs = 50;     % altimeter

Ts = 1/imuFs;   % IMU sampling time


%% Initial position and Earth parameters

% Starting position
refloc = [43.70853, 10.4036, 57];                           % Latitude, longitude [°], height [m]
lat_0 = refloc(1);
lon_0 = refloc(2);
alt_0 = refloc(3);

% Earth parameters and their influence on navigation are neglected.

% % Earth rotation rate
% omega_ei_i = 72.92115*10^-6 * [0; 0; 1];                    % Earth angular velocity in ECI [rad/s]
% omega_ie_i = - omega_ei_i;
% omega_ie = 72.92115*10^-6 * [cos(lat_0); 0; -sin(lon_0)];	  % Earth angular velocity in NED [rad/s]
% 
% % WGS84 initialization
% wgs84 = wgs84Ellipsoid;
% re = wgs84.SemimajorAxis;
% eccentricity = wgs84.Eccentricity;
% % Gravity approximation parameters
% g_wgs0 = 9.7803267714;
% g_wgs1 = 0.00193185138639;


%% Imported UAV Trajectory

% Load the "ground truth" UAV trajectory.
load LoggedQuadcopter.mat trajData;
trajOrient = trajData.Orientation;
trajVel = trajData.Velocity;
trajPos = trajData.Position;
trajPos = trajPos + [0, 0, - max(trajPos(:,3) - 1)];
trajAcc = trajData.Acceleration;
trajAngVel = trajData.AngularVelocity;

% Initialize the random number generator used in the simulation of sensor
% noise.
rng(1)


%% IMU Sensors

imu = imuSensor('accel-gyro-mag', 'SampleRate', imuFs);

% Earth magnetic field in the UAV starting position.
% MATLAB 2020 needed for this command. If a different (older) version is
% being used, you can change the year and the model used (the '2020') to
% make it work for your current version.
magnfield = (wrldmagm(refloc(3), refloc(1), refloc(2), decyear(date, 'dd-mm-yyyy'), '2020') / 1000).';
imu.MagneticField = magnfield;

% Real(istic) Sensors

real_gyro_bias = deg2rad(3.125) * [-1, 0.5, 1.5];
real_acc_bias = 0.19 * [0.5, 1, -1];
real_mag_dist = [0, 0, 0];

% Accelerometer
imu.Accelerometer.MeasurementRange =  19.6133;
imu.Accelerometer.Resolution = 0.0023928;
imu.Accelerometer.ConstantBias = real_acc_bias;
imu.Accelerometer.NoiseDensity = 0.0012356;

% Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250);
imu.Gyroscope.Resolution = deg2rad(0.0625);
imu.Gyroscope.ConstantBias = real_gyro_bias;
imu.Gyroscope.AxesMisalignment = 1.5;
imu.Gyroscope.NoiseDensity = deg2rad(0.025);

% Magnetometer
imu.Magnetometer.MeasurementRange = 1000;
imu.Magnetometer.Resolution = 0.1;
imu.Magnetometer.ConstantBias = real_mag_dist;
imu.Magnetometer.NoiseDensity = 0.3/ sqrt(50);


% Sensor orientations (s_i to body)
% They should be all the same if AHRS
q_sens_acc  = quaternion([1 0 0 0]);
q_sens_gyro = quaternion([1 0 0 0]);
q_sens_magn = quaternion([1 0 0 0]);


%% GPS Sensor

gps = gpsSensor('UpdateRate', gpsFs);
gps.ReferenceLocation = refloc;
gps.DecayFactor = 0.5;              	 % Random walk noise parameter
gps.HorizontalPositionAccuracy = 1.6;
gps.VerticalPositionAccuracy =  1.6;
gps.VelocityAccuracy = 0.1;


%% Ultra Wide Band Sensor

known_beacon_pos = [ 1,   1,  1;
                    10,   0,  0;
                     0,  10,  0;
                    10,  10,  0];

known_beacon_id = linspace(1,size(known_beacon_pos,1),size(known_beacon_pos,1));

uwb = uwb_sensor('beacon_position', known_beacon_pos, ...
                 'beacon_id', known_beacon_id, ...
                 'error_variance', 0.09, ...
                 'range', 40);


%% Vision Sensor

% Camera parameters

% Camera detection cone semi-angle [radians]
camera_cone_semi_angle = 80 * pi / 180;

% Maximum distance that the vision sensor is able to measure
camera_max_distance = 40;

% Minimum distance that the vision sensor is able to measure
camera_min_distance = 0.05;

% The noise on both position and orientation is modeled as the sum
% of a constant noise and a noise linear with the distance of the
% target from the camera.
% In addition, the noise, for both orientation and position, is
% different if the component is perpendicular or parallel to the
% line of sight.
% For position, the error is bigger on the parallel component.
noise_perpendicular_position_constant = (0.1)^2;
noise_parallel_position_constant      = (0.2)^2;
noise_perpendicular_position_linear   = (0.2 / camera_max_distance)^2;
noise_parallel_position_linear        = (0.4 / camera_max_distance)^2;

% For orientation, the error is bigger on the perpendicular
% component.
noise_perpendicular_orientation_constant = (2*pi/180)^2;
noise_parallel_orientation_constant      = (1*pi/180)^2;
noise_perpendicular_orientation_linear   = (4*pi/180 / camera_max_distance)^2;
noise_parallel_orientation_linear        = (2*pi/180 / camera_max_distance)^2;

% Camera frame orientation with respect to the body frame,
% specified as euler angles [yaw, pitch, roll].
camera_orientation = [0, 0, 0];

% Optical center position with respect to the body center of mass 
% in the body frame.
% p_cb_b
camera_position = [0.1, 0, 0.1];


% Marker parameters

% Marker id
known_marker_id = linspace(1, 4, 4);

% Markers reference frame orientation wrt global frame, specified as
% [yaw, pitch, roll] and in [rad]. Row vector.
known_marker_orientation = [0, pi/2, 0;
                            0, pi/2, 0;
                            0, pi/2, 0;
                            0, pi/2, 0]; 

% Markers position in global frame [m].
known_marker_position = [ 20,   0,   0;
                           0,  20,   0;
                         -20,   0,   0;
                           0, -20,   0];


% Aruco sensor simulator full setup ( ͡ʘ ͜ʖ ͡ʘ)
aruco = aruco_vision_sensor;
aruco.camera_cone_semi_angle = camera_cone_semi_angle;
aruco.camera_max_distance = camera_max_distance;
aruco.camera_min_distance = camera_min_distance;

aruco.noise_perpendicular_position_constant = noise_perpendicular_position_constant;
aruco.noise_parallel_position_constant = noise_parallel_position_constant;
aruco.noise_perpendicular_position_linear = noise_perpendicular_position_linear;
aruco.noise_parallel_position_linear = noise_parallel_position_linear;

aruco.noise_perpendicular_orientation_constant = noise_perpendicular_orientation_constant;
aruco.noise_parallel_orientation_constant = noise_parallel_orientation_constant;
aruco.noise_perpendicular_orientation_linear = noise_perpendicular_orientation_linear;
aruco.noise_parallel_orientation_linear = noise_parallel_orientation_linear;

aruco.camera_orientation = camera_orientation;
aruco.camera_position = camera_position;

aruco.marker_id = known_marker_id;
aruco.marker_orientation = known_marker_orientation;
aruco.marker_position = known_marker_position;


%% Barometer Sensor

bar = barometer_sensor;

bar.wn_variance = 0.5^2;
bar.bi_variance = 0.1^2;
bar.decay_factor = 0.001;

bar.quantization = 0.1;


%% Laser (?) Altimeter Sensor

alt = altimeter_sensor;

alt.wn_variance = 0.05^2;
alt.qwn_variance = 1^2 / 7^4;
alt.bi_variance = 0.01^2;
alt.decay_factor = 0.2;

alt.min_alt = 0.1;
alt.max_alt = 5;

alt.quantization = 0.01;

alt.p_0 = [0 0 0.1];


%% Initialize Scopes

useErrScope = true;     % Turn on the streaming error plot
usePoseView = true;     % Turn on the 3-D pose viewer

if useErrScope
    errscope = HelperScrollingPlotter(...
        'NumInputs', 4, ...
        'TimeSpan', 10, ...
        'SampleRate', imuFs, ...
        'YLabel', {'degrees', ...
        'meters', ...
        'meters', ...
        'meters'}, ...
        'Title', {'Quaternion Distance', ...
        'Position X Error', ...
        'Position Y Error', ...
        'Position Z Error'}, ...
        'YLimits', ...
        [   0, 10
           -2,  2
           -2,  2
           -2,  2]);
end

% Without sensor switches (uncomment and comment the other one to use)
% if usePoseView
%     posescope = HelperPoseViewer(...
%         'XPositionLimits', [-15 15], ...
%         'YPositionLimits', [-15 15], ...
%         'ZPositionLimits', [-10 10]);
% end

% With sensor switches
if usePoseView
    posescope = PoseViewerWithSwitches(...
        'XPositionLimits', [-30 30], ...
        'YPositionLimits', [-30, 30], ...
        'ZPositionLimits', [-10 10]);
    
    posescope.imuFs = imuFs;
    posescope.magFs = magFs;
    posescope.gpsFs = gpsFs;
    posescope.barFs = barFs;
    posescope.altFs = altFs;
    posescope.uwbFs = uwbFs;
    posescope.visFs = visFs;
end

f = gcf;    % current figure handle


%% States plots

% States animatedline plot

fig3 = figure('Name', 'States', ...
              'NumberTitle', 'off', ...
              'Units', 'normalized', ...
              'OuterPosition', [0.5, 0, 0.5, 1]);

hax3 = subplot(3,2,1);
title('Gyroscope bias')

hax4 = subplot(3,2,2);
title('Accelerometer bias')

hax5 = subplot(3,2,3);
title('Linear acceleration')

hax6 = subplot(3,2,4);
title('Magnetometer disturbance')

hax7 = subplot(3,2,5);
title('Velocity')

hax8 = subplot(3,2,6);
title('Ground height')

% Estimated values

default_colors = colororder;

% 
% [anim_est_states, anim_real_states, fig3] = anim_states_create(default_colors);


%% Filter setup

% Number of accelerometer and gyroscope measurements which are considered
% together in the Kalman filter. Increase this value to reduce the
% computational cost, at the expense of estimation precision.
decimation_factor = 1;

dt = Ts;

% Filter call and properties initialization
FUSE = my_navigation_filter_old;

% Sample rates
FUSE.sample_rate = imuFs;

FUSE.decimation_factor = decimation_factor;

% Some process noises
FUSE.pos_noise = 1e-3 / dt;
FUSE.vel_noise = 1e-5 / dt;

% Accel, gyro, magn noise
FUSE.acc_noise = 1e+2;
FUSE.magn_noise = 1e+2;
FUSE.gyro_noise = 1e-4 / dt^2;

% IMU sensor frames orientation
FUSE.q_sens_acc = q_sens_acc;
FUSE.q_sens_gyro = q_sens_gyro;
FUSE.q_sens_magn = q_sens_magn;

% Gyroscope drift
FUSE.gyro_drift_noise = 1e-4 / dt;

% Linear acceleration
FUSE.lin_acc_noise = 1e+1 / dt;
FUSE.lin_acc_decay_factor = 1 / dt;

% Magnetometer disturbance
FUSE.magn_disturbance_noise = 1e-1 / dt;
FUSE.magn_disturbance_decay_factor = 1 * imuFs;

% GPS
FUSE.GPS_pos_noise = 1e+1;
FUSE.GPS_vel_noise = 1e+0;

% Acceleration drift (bias)
FUSE.acc_drift_noise = 1e-3 / dt;

% UWB
FUSE.uwb_noise = 1e+2;
FUSE.uwb_beacon_position = known_beacon_pos;
FUSE.uwb_beacon_id = known_beacon_id;

% Computer vision
FUSE.camera_orient = camera_orientation;
FUSE.camera_pos = camera_position;
FUSE.marker_id = known_marker_id;
FUSE.marker_orient = known_marker_orientation;
FUSE.marker_pos = known_marker_position;
FUSE.vision_pos_noise = 1e+2;
FUSE.vision_orient_noise = 1e+2;

% Barometer
FUSE.bar_noise = 1e+2;

% Altimeter
FUSE.alt_pos = [0 0 0.1];
FUSE.alt_noise = 1e+0;
FUSE.alt_threshold = 0.3;

FUSE.ground_height_noise = 1e+0 / dt;
FUSE.ground_height_decay_factor = 1e-1 / dt;


FUSE.init_process_noise = diag([1e-0 * ones(3,1)
                                1e-1 * ones(3,1)
                                1e-2 * ones(3,1)
                                1e-1 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(3,1)
                                1e+0 * ones(1,1)]);

FUSE.magn_field_NED = magnfield.';

FUSE.reference_location = refloc;

FUSE.private_properties_update;


FUSE(1);    % To initialize the discrete state. TODO: eliminate the need for this.


% Filter state initialization using some ad hoc orientation algorithm.
[~, gyro, mag1] = imu(trajAcc(1:100, :), ...
                      trajAngVel(1:100, :), ...
                      quaternion(ones(100,1)*[1, 0, 0, 0]));

[~, ~, mag2] = imu(trajAcc(1:100, :), ...
                   trajAngVel(1:100, :), ...
                   quaternion(ones(100,1)*[0, 0, 0, 1]));

[~, ~, mag3] = imu(trajAcc(1:100, :), ...
                   trajAngVel(1:100, :), ...
                   quaternion(ones(100,1)*[0, 0, 1, 0]));

init_orient = meanrot(trajOrient(1:100));
init_gyro_drift = mean(gyro);
init_gyro_drift = init_gyro_drift.';

init_magn_dist(1) = -1/2 * mean(mag2(:,1) + mag1(:,1));
init_magn_dist(2) = -1/2 * mean(mag2(:,2) + mag1(:,2));
init_magn_dist(3) = -1/2 * mean(mag3(:,3) + mag1(:,3));
init_magn_dist = init_magn_dist.';
init_magn_dist = quat2rotm(init_orient).' * init_magn_dist;

FUSE.set_initial_state('q', compact(init_orient), ...
                       'gyro_offset', [0, 0, 0], ...
                       'pos', trajPos(1,:).', ...
                       'vel', trajVel(1,:).', ...
                       'acc_bias', [0.15, 0.15, 0.15].');


%% Simulation Loop

% Loop setup - |trajData| has about 142 seconds of recorded data.
secondsToSimulate = 50;
numsamples = secondsToSimulate*imuFs;

loopBound = floor(numsamples);
loopBound = floor(loopBound/imuFs)*imuFs;	% ensure enough IMU Samples

% Log data for final metric computation.
pqorient = quaternion.zeros(loopBound,1);
pqpos = zeros(loopBound,3);
pqvel = zeros(loopBound,3);
pqacc = zeros(loopBound,3);

% The starting orientation and position is supposed to be exactly known.
pqorient(1,:) = trajOrient(1);
pqpos(1,:) = trajPos(1,:);
pqvel(1,:) = trajVel(1,:);

fcnt = 1;

% If true (there have been an user interaction with the pose viewer) the
% filter privare properties need to be updated with the method 
% private_properties_update
update_properties_flag = false;

while(fcnt <=loopBound)
    % |predict| loop at IMU update frequency.
    
    if decimation_factor ~= f.UserData.DecimationFactor
        decimation_factor = f.UserData.DecimationFactor;
        FUSE.decimation_factor = decimation_factor;
        update_properties_flag = true;
    end
    
    if imuFs ~= f.UserData.AccelerometerSampleRate
        imuFs = f.UserData.AccelerometerSampleRate;
        FUSE.sample_rate = imuFs;
        update_properties_flag = true;
    end
    
    if magFs ~= f.UserData.MagnetometerSampleRate
        magFs = f.UserData.MagnetometerSampleRate;
        update_properties_flag = true;
    end
    
    if gpsFs ~= f.UserData.GPSSampleRate
        gpsFs = f.UserData.GPSSampleRate;
        update_properties_flag = true;
    end
    
    if uwbFs ~= f.UserData.uwbSampleRate
        uwbFs = f.UserData.uwbSampleRate;
        update_properties_flag = true;
    end
    
    if visFs ~= f.UserData.visSampleRate
        visFs = f.UserData.visSampleRate;
        update_properties_flag = true;
    end
    
    if barFs ~= f.UserData.barSampleRate
        barFs = f.UserData.barSampleRate;
        update_properties_flag = true;
    end
    
    if altFs ~= f.UserData.altSampleRate
        altFs = f.UserData.altSampleRate;
        update_properties_flag = true;
    end
    
    if update_properties_flag
        FUSE.private_properties_update;
    end
    
    
%     % Change the filter properties after some time is passed
%     if fcnt == imuFs * 5
%         FUSE.gyro_drift_noise = 1e-5;
%         
%         FUSE.acc_drift_noise = 1e-5;
%         
%         FUSE.lin_acc_noise = 1e+2 / dt;
%         FUSE.lin_acc_decay_factor = 0.00 / dt;
%         
%         FUSE.GPS_pos_noise = 1e+2;
%         
%         FUSE.private_properties_update;
%     end
    
    
    % Simulate the IMU data from the current pose.
    [accel, gyro, mag] = imu(trajAcc(fcnt,:), ...
                             trajAngVel(fcnt, :), ...
                             trajOrient(fcnt));
	
	% Sensor output
    % accel = - f_b;
    % gyro = omega_ib_b;
	
	% Rotate the outputs because the sensor frame does not coincide with
	% the body frame.
    accel = quat_rot(accel, conj(q_sens_acc));      % conj because we rotate from the boody frame to the sensor frame
    gyro = quat_rot(gyro, conj(q_sens_gyro));
    mag = quat_rot(mag, conj(q_sens_magn));
    
    
    % The filter predict step is done at the IMU (accelerometer and gyro)
    % frequency.
    FUSE.predict(gyro, accel);
    
    
    % The correction using the accelerometer data is done every
    % decimation_factor sample times.
    if mod(fcnt, fix(decimation_factor)) == 0
        if f.UserData.Accelerometer
            % Accelerometer correction
            FUSE.fuse_accel;
        else
            FUSE.reset_store_accel;
        end
    end
    
    
    % Magnetometer correction
    if (f.UserData.Magnetometer) && mod(fcnt, fix(imuFs/f.UserData.MagnetometerSampleRate)) == 0
        % Fuse the magnetomer measurement
        FUSE.fuse_magn(mag);
    end
    
    
    [pqorient(fcnt), pqpos(fcnt, :)] = FUSE.get_pose();
    
    
    fusedOrient =  pqorient(fcnt,:);
    fusedPos =  pqpos(fcnt,:);
    
    
    % Compute the errors and plot.
    if useErrScope
        orientErr = rad2deg(dist(fusedOrient, ...
            trajOrient(fcnt) ));
        posErr = fusedPos - trajPos(fcnt,:);
        errscope(orientErr, posErr(1), posErr(2), posErr(3));
    end
        
    % Update the pose viewer.
    if usePoseView
        posescope(pqpos(fcnt,:), pqorient(fcnt),  trajPos(fcnt,:), ...
            trajOrient(fcnt,:) );
    end
    
    
    % Update the top view
    
    time_x = fcnt / imuFs;
    
    % Estimated
%     anim_states_add(anim_est_states, anim_real_states, time_x, FUSE.getDiscreteState.state, real_gyro_bias, real_acc_bias, trajAcc(fcnt,:), real_mag_dist, trajVel(fcnt,:))

    
    
    % GPS correction step (at the GPS sample rate)
    if (f.UserData.GPS) && mod(fcnt, fix(imuFs/f.UserData.GPSSampleRate)) == 0 
        % Simulate the GPS output based on the current pose.
        [lla, gpsvel] = gps( trajPos(fcnt,:), trajVel(fcnt,:) );
    
        % Correct the filter states based on the GPS data measurements.
        FUSE.fuse_GPS(lla, gpsvel);
    end
    
    
    % UWB correction step
    if (f.UserData.UltraWideBand) && mod(fcnt, fix(imuFs/f.UserData.uwbSampleRate)) == 0
        [uwb_meas_dist, uwb_beacon_id] = uwb(trajPos(fcnt,:), known_beacon_id);
        
        FUSE.fuse_uwb(uwb_meas_dist, uwb_beacon_id);
    end
    
    
    % Vision correction step
    if (f.UserData.Vision) && mod(fcnt, fix(imuFs/f.UserData.visSampleRate)) == 0
        [Tmc, vis_id, vis_index] = aruco(trajPos(fcnt,:), trajOrient(fcnt));
        
        FUSE.fuse_vis(Tmc, vis_id);
    end
    
    
%     % Barometer correction step
%     if (f.UserData.Barometer) && mod(fcnt, fix(imuFs/f.UserData.barSampleRate)) == 0
%         bar_h_meas = bar(trajPos(fcnt,:));
%         
%         pqpos(fcnt+1, 3) = FUSE.fuse_bar(bar_h_meas);
%     end
%     
%     
%     % Altimeter correction step
%     if (f.UserData.Altimeter) && mod(fcnt, fix(imuFs/f.UserData.altSampleRate)) == 0
%         alt_h_meas = alt(trajPos(fcnt,:), trajOrient(fcnt,:));
%         
%         pqpos(fcnt+1, 3) = FUSE.fuse_alt(alt_h_meas);
%     end
    
    
    update_properties_flag = false;
    
    
    fcnt = fcnt + 1;
end


%% Plot

% Comparison between the real and the estimated Euler angles.
% x = linspace(1,loopBound+1,loopBound+1);
% time = x * Ts;
% 
% eul_real = quat2eul(trajOrient(1:loopBound+1));
% 
% eul_meas = quat2eul(pqorient);
% 
% figure(2)
% plot(time, eul_meas(1:loopBound+1,1).', time, eul_real(1:loopBound+1,1).');
% title('Yaw Angle')
% legend('Measured angle', 'Ground-truth angle')
% 
% figure(3)
% plot(time, eul_meas(1:loopBound+1,2).', time, eul_real(1:loopBound+1,2).');
% title('Pitch Angle')
% legend('Measured angle', 'Ground-truth angle')
% 
% figure(4)
% plot(time ,eul_meas(1:loopBound+1,3).', time, eul_real(1:loopBound+1,3).');
% title('Roll Angle')
% legend('Measured angle', 'Ground-truth angle')


% Comparison between the real and the estimated velocity.
% figure(2)
% plot(pqvel(:,:) - trajVel(1:length(pqvel),:))
% title('Velocity')
% legend('Measured velocity', 'Ground-truth velocity')


% Estimated accelerometer bias.
% figure(3)
% plot(temp)
% title('Accelerometer Bias')


%% Error Metric Computation
% Position and orientation estimates were logged throughout the
% simulation. Now compute an end-to-end root mean squared error for both
% position and orientation.

posd = pqpos(1:loopBound,:) - trajPos( 1:loopBound, :);

% For orientation, quaternion distance is a much better alternative to
% subtracting Euler angles, which have discontinuities. The quaternion
% distance can be computed with the |dist| function, which gives the
% angular difference in orientation in radians. Convert to degrees
% for display in the command window. 

quatd = rad2deg(dist(pqorient(1:loopBound), trajOrient(1:loopBound)) );

% Display RMS errors in the command window.
fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');
msep = sqrt(mean(posd.^2));
fprintf('\tX: %.2f , Y: %.2f, Z: %.2f   (meters)\n\n',msep(1), ...
    msep(2), msep(3));

fprintf('End-to-End Quaternion Distance RMS Error (degrees) \n');
fprintf('\t%.2f (degrees)\n\n', sqrt(mean(quatd.^2)));


%% Quaternions

% CONVENTION: Quaternion (used by MATLAB) (Hamilton convention)
%	q = (q_scalar, q_vectorial) = (q_w, q_v) = (q_w, q_x, q_y, q_z)
%   Right-handed: ij = k
%   Passive: rotate frames (not vectors)
%   Local-to-Global: q = q_GL = q_L2G
%                    x_G = q * x_L * conj(q)

% CONVENTION: Euler angles (used by MATLAB):
%   eul = [yaw (z), pitch (y), roll (x)] (in this order)
%   rotation order = ZYX: Rz -> Ry -> Rx

% Quaternions are used to avoid singularities in the representation of the
% orientation.

% Alternative method (less computational cost, worse accuracy).
% Uncomment if you want to use this one and comment the other function.
% function [qt_p] = quat_updt(qt, omega, Ts)
%     % Euler approximation
%     qt_p = qt + 1/2*Ts*quaternion([0,omega])*qt;
%     
%     % Output quaternion normalization, it is necessary in this case.
%     qt_p = 1/norm(qt_p) * qt_p;
% end

% Alternative method (small computational cost, medium accuracy).
% Uncomment if you want to use this one and comment the other function.
% function [qt_p] = quat_updt(qt, omega, Ts)
%     % Euler approximation
%     qt_p = quaternion([1,1/2*Ts*omega]) * qt;
%     
%     % Output quaternion normalization, it is necessary in this case.
%     qt_p = 1/norm(qt_p) * qt_p;
% end

% function [qt_p] = quat_updt(qt, omega, Ts)
% % Quaternion update function, uses the quaternion exponential method.
%     qt_p = exp(1/2*Ts*quaternion([0,omega]))*qt;
%     
%     % The normalization should be unnecessary with this method, it is
%     % still used to prevent the accumulation of numerical errors from
%     % affecting the quaternion unit norm.
%     qt_p = 1/norm(qt_p) * qt_p;
% end


% Rotation with quaternions
function v_out = quat_rot(v, q)
    quat_v = quaternion([0, v(1), v(2), v(3)]);
    
    quat_v_out = q * quat_v * conj(q);
    
    % Quaternion parts extraction
    [~, part1, part2, part3] = parts(quat_v_out);
    
    % With this way, the output is in the same form of the input (row or
    % column vector)
    v(1) = part1;
    v(2) = part2;
    v(3) = part3;
    
    v_out = v;
end

%% Rotations

% CONVENTION: rotation matrices nomenclature
%	Cnb corresponds to a change of the reference frame from frame b to
%   frame n.
%   v_n = Cnb * v_b
%	Cnb = C_b2n = C_b_n

% function out = rotCni(lat, long, time, omega_ie)
% % Rotation matrix from the ECI frame to the navigation frame.
%     lat = lat*pi/180;
%     long = long*pi/180;
%     
%     lon = long + omega_ie*time;
% 
%     out = [[-sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat)];
%            [-sin(lon), cos(lon), 0];
%            [-cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)]];
% end
% 
% function out = rotCnb(eul)
% % Rotation matrix from the body frame to the navigation frame after a
% % yaw-pitch-roll rotation (ZYX).
%     yaw = eul(1);
%     pitch = eul(2);
%     roll = eul(3);
% 
%     out = ( rotCx(roll)*rotCy(pitch)*rotCz(yaw) ).';
% end
% 
% function out = rotCx(ang)
%     out = [[1, 0, 0];
%            [0, cos(ang), sin(ang)];
%            [0, -sin(ang), cos(ang)]];
% end
% 
% function out = rotCy(ang)
%     out = [[cos(ang), 0, -sin(ang)];
%            [0, 1, 0];
%            [sin(ang), 0, cos(ang)]];
% end
% 
% function out = rotCz(ang)
%     out = [[cos(ang), sin(ang), 0];
%            [-sin(ang), cos(ang), 0];
%            [0, 0, 1]];
% end


%% Earth

% function [rn, rm, g] = ellwgs84(re, eccentricity, lat, h, g_wgs0, g_wgs1)
%     rn = re/(1-eccentricity^2*sin(lat)^2)^0.5;                      % R_normal
%     rm = rn * (1-eccentricity^2)/(1-eccentricity^2*sin(lat)^2);     % R_meridian
%     
%     g = g_wgs0 * (1 + g_wgs1 * sin(lat)^2)/(1 - eccentricity^2 * sin(lat)^2) * re^2/(re + h)^2;
% end
% 
% function [lat, lon] = coord_updt(coord, v_n, v_e, rn, rm, Ts)
% 
%     % v_n = v_North     rn = R_normal       Ts = sampling time
%     % v_e = v_East      rm = R_meridian
% 
%     lat = coord(1);
%     lon = coord(2);
%     h = coord(3);
% 
%     latd = v_n / (rm + h);
%     lond = v_e / ((rn + h) * cos(lat));
%     
%     lat = lat + latd * Ts;
%     lon = lon + lond * Ts;
% end
% 
% function omega_en_n = x2omega_en(lat, h, v_n, v_e)
%     latd = v_n / (rm + h);
%     lond = v_e / ((rn + h) * cos(lat));
% 
%     omega_en_n = [ lond * cos(lat);
%                   -latd;
%                   -lond * sin(lat)];
% end

