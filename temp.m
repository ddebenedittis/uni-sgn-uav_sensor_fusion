%% Initialization

clear
close all
clc


%% States

% q_0        = x( 1);
% q_1        = x( 2);
% q_2        = x( 3);
% q_3        = x( 4);
% gyro_b_x   = x( 5);
% gyro_b_y   = x( 6);
% gyro_b_z   = x( 7);
% acc_b_x    = x( 8);
% acc_b_y    = x( 9);
% acc_b_z    = x(10);
% lin_acc_n  = x(11);
% lin_acc_e  = x(12);
% lin_acc_d  = x(13);
% mag_d_x    = x(14);
% mag_d_y    = x(15);
% mag_d_z    = x(16);
% pos_n      = x(17);
% pos_e      = x(18);
% pos_d      = x(19);
% vel_n      = x(20);
% vel_e      = x(21);
% vel_d      = x(22);
% h_g        = x(23);

% x( 1) = q_0
% x( 2) = q_1
% x( 3) = q_2
% x( 4) = q_3
% x( 5) = gyro_b_x
% x( 6) = gyro_b_y
% x( 7) = gyro_b_z
% x( 8) = acc_b_x
% x( 9) = acc_b_y
% x(10) = acc_b_z
% x(11) = lin_acc_n
% x(12) = lin_acc_e
% x(13) = lin_acc_d
% x(14) = mag_d_x
% x(15) = mag_d_y
% x(16) = mag_d_z
% x(17) = pos_n
% x(18) = pos_e
% x(19) = pos_d
% x(20) = vel_n
% x(21) = vel_e
% x(22) = vel_d
% x(23) = h_g


%% State Transition Function & Matrix

syms q_0 q_1 q_2 q_3   gyro_b_x gyro_b_y gyro_b_z   acc_b_x acc_b_y acc_b_z   lin_acc_n lin_acc_e lin_acc_d   mag_d_x mag_d_y mag_d_z   pos_n pos_e pos_d   vel_n vel_e vel_d   h_g
syms dt ni sigma lambda grav
syms gyro_meas_x gyro_meas_y gyro_meas_z   acc_meas_x acc_meas_y acc_meas_z

gyro_bias = [gyro_b_x; gyro_b_y; gyro_b_z];
acc_bias = [acc_b_x; acc_b_y; acc_b_z];
lin_acc = [lin_acc_n; lin_acc_e; lin_acc_d];
mag_dist = [mag_d_x; mag_d_y; mag_d_z];
pos = [pos_n; pos_e; pos_d];
vel = [vel_n; vel_e; vel_d];

q_v = [q_1; q_2; q_3];
R = eye(3) + 2*skew(q_v)*(q_0*eye(3)+skew(q_v));

g_v = + grav * [0;0;1];

acc_meas = [acc_meas_x; acc_meas_y; acc_meas_z];
gyro_meas = [gyro_meas_x; gyro_meas_y; gyro_meas_z];

f = [ q_0 - 1/2*dt*((q_1*gyro_b_x) + (q_2*gyro_b_y) + (q_3*gyro_b_z))           % orientation (quaternion)
      q_1 + 1/2*dt*((q_0*gyro_b_x) - (q_3*gyro_b_y) + (q_2*gyro_b_z))
      q_2 + 1/2*dt*((q_3*gyro_b_x) + (q_0*gyro_b_y) - (q_1*gyro_b_z))
      q_3 + 1/2*dt*((q_1*gyro_b_y) - (q_2*gyro_b_x) + (q_0*gyro_b_z))
      gyro_bias                                               	                % gyroscope bias
      acc_bias                                                	                % acceleration bias
      (1 - dt*ni) * lin_acc                                   	                % linear acceleration
      (1 - dt*sigma) * mag_dist                               	                % magnetometer disturbance
      pos + dt*vel - 1/2*dt^2*(R*acc_bias + g_v)                    	        % position - ATTENTION: g_v derivato fa zero, quindi devo considerarlo altrimenti
      vel - dt*(R*acc_bias + g_v)                                               % velocity
      (1 - dt*lambda) * h_g ];                                                  % ground height


g = [ - 1/2*dt*((q_1*gyro_meas_x) + (q_2*gyro_meas_y) + (q_3*gyro_meas_z))      % orientation error
        1/2*dt*((q_0*gyro_meas_x) - (q_3*gyro_meas_y) + (q_2*gyro_meas_z))
        1/2*dt*((q_3*gyro_meas_x) + (q_0*gyro_meas_y) - (q_1*gyro_meas_z))
        1/2*dt*((q_1*gyro_meas_y) - (q_2*gyro_meas_x) + (q_0*gyro_meas_z))
        zeros(3,1)                                                              % gyroscope bias error
        zeros(3,1)                                                              % acceleration bias error
        zeros(3,1)                                                              % linear acceleration error
        zeros(3,1)                                                              % magnetometer bias error
        1/2*dt^2*R*acc_meas                                                     % position error
        dt*R*acc_meas                                                           % velocity error
        0];                                                                     % ground height error

x = [ q_0 q_1 q_2 q_3   gyro_b_x gyro_b_y gyro_b_z   acc_b_x acc_b_y acc_b_z   lin_acc_n lin_acc_e lin_acc_d   mag_d_x mag_d_y mag_d_z   pos_n pos_e pos_d   vel_n vel_e vel_d   h_g ];

u = [ gyro_meas_x gyro_meas_y gyro_meas_z   acc_meas_x acc_meas_y acc_meas_z ];

dfdx = simplify(jacobian(f, x));

dgdu = simplify(jacobian(g, u));

F = [                                  1,                                          -(dt*gyro_b_x)/2,                                          -(dt*gyro_b_y)/2,                                          -(dt*gyro_b_z)/2, -(dt*q_1)/2,  (dt*q_2)/2,  (dt*q_3)/2,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                         (dt*gyro_b_x)/2,                                                         1,                                           (dt*gyro_b_z)/2,                                          -(dt*gyro_b_y)/2,  (dt*q_0)/2, -(dt*q_3)/2,  (dt*q_2)/2,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                         (dt*gyro_b_y)/2,                                          -(dt*gyro_b_z)/2,                                                         1,                                           (dt*gyro_b_x)/2,  (dt*q_3)/2,  (dt*q_0)/2, -(dt*q_1)/2,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                         (dt*gyro_b_z)/2,                                           (dt*gyro_b_y)/2,                                          -(dt*gyro_b_x)/2,                                                         1, -(dt*q_2)/2,  (dt*q_1)/2,  (dt*q_0)/2,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           1,           0,           0,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           1,           0,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           1,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 1,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 1,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 1,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 0, 1 - dt*ni,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 0,         0, 1 - dt*ni,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 0,         0,         0, 1 - dt*ni,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 0,         0,         0,         0, 1 - dt*sigma,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 0,         0,         0,         0,            0, 1 - dt*sigma,            0, 0, 0, 0,  0,  0,  0,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0, 1 - dt*sigma, 0, 0, 0,  0,  0,  0,             0
        dt^2*(acc_b_y*q_3 - acc_b_z*q_2),                 -(dt^2*(2*acc_b_y*q_2 + 2*acc_b_z*q_3))/2, -(dt^2*(2*acc_b_y*q_1 - 4*acc_b_x*q_2 + 2*acc_b_z*q_0))/2,  (dt^2*(2*acc_b_y*q_0 + 4*acc_b_x*q_3 - 2*acc_b_z*q_1))/2,           0,           0,           0,  (dt^2*(2*q_2^2 + 2*q_3^2 - 1))/2,          dt^2*(q_0*q_3 - q_1*q_2), -(dt^2*(2*q_0*q_2 + 2*q_1*q_3))/2,         0,         0,         0,            0,            0,            0, 1, 0, 0, dt,  0,  0,             0
       -dt^2*(acc_b_x*q_3 - acc_b_z*q_1),  (dt^2*(4*acc_b_y*q_1 - 2*acc_b_x*q_2 + 2*acc_b_z*q_0))/2,                 -(dt^2*(2*acc_b_x*q_1 + 2*acc_b_z*q_3))/2, -(dt^2*(2*acc_b_x*q_0 - 4*acc_b_y*q_3 + 2*acc_b_z*q_2))/2,           0,           0,           0, -(dt^2*(2*q_0*q_3 + 2*q_1*q_2))/2,  (dt^2*(2*q_1^2 + 2*q_3^2 - 1))/2,          dt^2*(q_0*q_1 - q_2*q_3),         0,         0,         0,            0,            0,            0, 0, 1, 0,  0, dt,  0,             0
        dt^2*(acc_b_x*q_2 - acc_b_y*q_1), -(dt^2*(2*acc_b_y*q_0 + 2*acc_b_x*q_3 - 4*acc_b_z*q_1))/2,  (dt^2*(2*acc_b_x*q_0 - 2*acc_b_y*q_3 + 4*acc_b_z*q_2))/2,                 -(dt^2*(2*acc_b_x*q_1 + 2*acc_b_y*q_2))/2,           0,           0,           0,          dt^2*(q_0*q_2 - q_1*q_3), -(dt^2*(2*q_0*q_1 + 2*q_2*q_3))/2,  (dt^2*(2*q_1^2 + 2*q_2^2 - 1))/2,         0,         0,         0,            0,            0,            0, 0, 0, 1,  0,  0, dt,             0
      dt*(2*acc_b_y*q_3 - 2*acc_b_z*q_2),                       -dt*(2*acc_b_y*q_2 + 2*acc_b_z*q_3),       -dt*(2*acc_b_y*q_1 - 4*acc_b_x*q_2 + 2*acc_b_z*q_0),        dt*(2*acc_b_y*q_0 + 4*acc_b_x*q_3 - 2*acc_b_z*q_1),           0,           0,           0,        dt*(2*q_2^2 + 2*q_3^2 - 1),        dt*(2*q_0*q_3 - 2*q_1*q_2),       -dt*(2*q_0*q_2 + 2*q_1*q_3),         0,         0,         0,            0,            0,            0, 0, 0, 0,  1,  0,  0,             0
       -2*dt*(acc_b_x*q_3 - acc_b_z*q_1),        dt*(4*acc_b_y*q_1 - 2*acc_b_x*q_2 + 2*acc_b_z*q_0),                       -dt*(2*acc_b_x*q_1 + 2*acc_b_z*q_3),       -dt*(2*acc_b_x*q_0 - 4*acc_b_y*q_3 + 2*acc_b_z*q_2),           0,           0,           0,       -dt*(2*q_0*q_3 + 2*q_1*q_2),        dt*(2*q_1^2 + 2*q_3^2 - 1),        dt*(2*q_0*q_1 - 2*q_2*q_3),         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  1,  0,             0
      dt*(2*acc_b_x*q_2 - 2*acc_b_y*q_1),       -dt*(2*acc_b_y*q_0 + 2*acc_b_x*q_3 - 4*acc_b_z*q_1),        dt*(2*acc_b_x*q_0 - 2*acc_b_y*q_3 + 4*acc_b_z*q_2),                       -dt*(2*acc_b_x*q_1 + 2*acc_b_y*q_2),           0,           0,           0,        dt*(2*q_0*q_2 - 2*q_1*q_3),       -dt*(2*q_0*q_1 + 2*q_2*q_3),        dt*(2*q_1^2 + 2*q_2^2 - 1),         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  1,             0
                                       0,                                                         0,                                                         0,                                                         0,           0,           0,           0,                                 0,                                 0,                                 0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0, 1 - dt*lambda ];

G = [ -(dt*q_1)/2, -(dt*q_2)/2, -(dt*q_3)/2,                                 0,                                 0,                                 0
       (dt*q_0)/2, -(dt*q_3)/2,  (dt*q_2)/2,                                 0,                                 0,                                 0
       (dt*q_3)/2,  (dt*q_0)/2, -(dt*q_1)/2,                                 0,                                 0,                                 0
      -(dt*q_2)/2,  (dt*q_1)/2,  (dt*q_0)/2,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0,                                 0,                                 0,                                 0
                0,           0,           0, -(dt^2*(2*q_2^2 + 2*q_3^2 - 1))/2,         -dt^2*(q_0*q_3 - q_1*q_2),          dt^2*(q_0*q_2 + q_1*q_3)
                0,           0,           0,          dt^2*(q_0*q_3 + q_1*q_2), -(dt^2*(2*q_1^2 + 2*q_3^2 - 1))/2,         -dt^2*(q_0*q_1 - q_2*q_3)
                0,           0,           0,         -dt^2*(q_0*q_2 - q_1*q_3),          dt^2*(q_0*q_1 + q_2*q_3), -(dt^2*(2*q_1^2 + 2*q_2^2 - 1))/2
                0,           0,           0,       -dt*(2*q_2^2 + 2*q_3^2 - 1),         -2*dt*(q_0*q_3 - q_1*q_2),        dt*(2*q_0*q_2 + 2*q_1*q_3)
                0,           0,           0,        dt*(2*q_0*q_3 + 2*q_1*q_2),       -dt*(2*q_1^2 + 2*q_3^2 - 1),         -2*dt*(q_0*q_1 - q_2*q_3)
                0,           0,           0,         -2*dt*(q_0*q_2 - q_1*q_3),        dt*(2*q_0*q_1 + 2*q_2*q_3),       -dt*(2*q_1^2 + 2*q_2^2 - 1)
                0,           0,           0,                                 0,                                 0,                                 0 ];


syms pos_err_n pos_err_e pos_err_d   vel_err_n vel_err_e vel_err_d   acc_b_err_x acc_b_err_y acc_b_err_z   theta_err_x theta_err_y theta_err_z   gyro_b_err_x gyro_b_err_y gyro_b_err_z   lin_acc_err_n lin_acc_err_e lin_acc_err_d   mag_d_err_x mag_d_err_y mag_d_err_z   h_g_err

theta_err = [theta_err_x; theta_err_y; theta_err_z];
gyro_bias_err = [gyro_b_err_x; gyro_b_err_y; gyro_b_err_z];
acc_bias_err = [acc_b_err_x; acc_b_err_y; acc_b_err_z];
lin_acc_err = [lin_acc_err_n; lin_acc_err_e; lin_acc_err_d];
mag_dist_err = [mag_d_err_x; mag_d_err_y; mag_d_err_z];
pos_err = [pos_err_n; pos_err_e; pos_err_d];
vel_err = [vel_err_n; vel_err_e; vel_err_d];

R = eye(3) + skew(theta_err);

f_err = [ theta_err + dt*gyro_bias_err
          gyro_bias_err
          acc_bias_err
          (1 - dt*ni) * lin_acc_err
          (1 - dt*sigma) * mag_dist_err
          pos_err + dt*vel_err - 1/2*dt^2*(R*acc_bias_err + R*skew(acc_meas - acc_bias)*theta_err)
          vel_err - dt*(R*acc_bias_err + R*skew(acc_meas - acc_bias)*theta_err)
          (1 - dt*lambda) * h_g_err ];

% g_err = [ zeros(3,1)
%           zeros(3,1)
%           zeros(3,1)
%           zeros(3,1)
%           zeros(3,1)
%           1/2*dt^2*(R*skew(acc_meas)*theta_err)
%           dt*(R*skew(acc_meas)*theta_err)
%           zeros(3,1)];

x_err = [ theta_err_x theta_err_y theta_err_z   gyro_b_err_x gyro_b_err_y gyro_b_err_z   acc_b_err_x acc_b_err_y acc_b_err_z   lin_acc_err_n lin_acc_err_e lin_acc_err_d   mag_d_err_x mag_d_err_y mag_d_err_z   pos_err_n pos_err_e pos_err_d   vel_err_n vel_err_e vel_err_d   h_g_err ];

df_errdx_err = simplify(jacobian(f_err, x_err));

df_errdx_err_s = subs(df_errdx_err, x_err, zeros(1,22));

% dg_errdu = simplify(jacobian(g_err, x));

F_err = [                                1,                                0,                                0, dt,  0,  0,       0,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                1,                                0,  0, dt,  0,       0,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                1,  0,  0, dt,       0,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  1,  0,  0,       0,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  1,  0,       0,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  1,       0,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       1,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       1,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       1,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       0, 1 - dt*ni,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       0,         0, 1 - dt*ni,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       0,         0,         0, 1 - dt*ni,            0,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       0,         0,         0,         0, 1 - dt*sigma,            0,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       0,         0,         0,         0,            0, 1 - dt*sigma,            0, 0, 0, 0,  0,  0,  0,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       0,         0,         0,         0,            0,            0, 1 - dt*sigma, 0, 0, 0,  0,  0,  0,             0
                                         0, -(dt^2*(acc_b_z - acc_meas_z))/2,  (dt^2*(acc_b_y - acc_meas_y))/2,  0,  0,  0, -dt^2/2,       0,       0,         0,         0,         0,            0,            0,            0, 1, 0, 0, dt,  0,  0,             0
           (dt^2*(acc_b_z - acc_meas_z))/2,                                0, -(dt^2*(acc_b_x - acc_meas_x))/2,  0,  0,  0,       0, -dt^2/2,       0,         0,         0,         0,            0,            0,            0, 0, 1, 0,  0, dt,  0,             0
          -(dt^2*(acc_b_y - acc_meas_y))/2,  (dt^2*(acc_b_x - acc_meas_x))/2,                                0,  0,  0,  0,       0,       0, -dt^2/2,         0,         0,         0,            0,            0,            0, 0, 0, 1,  0,  0, dt,             0
                                         0,       -dt*(acc_b_z - acc_meas_z),        dt*(acc_b_y - acc_meas_y),  0,  0,  0,     -dt,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  1,  0,  0,             0
                 dt*(acc_b_z - acc_meas_z),                                0,       -dt*(acc_b_x - acc_meas_x),  0,  0,  0,       0,     -dt,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  1,  0,             0
                -dt*(acc_b_y - acc_meas_y),        dt*(acc_b_x - acc_meas_x),                                0,  0,  0,  0,       0,       0,     -dt,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  1,             0
                                         0,                                0,                                0,  0,  0,  0,       0,       0,       0,         0,         0,         0,            0,            0,            0, 0, 0, 0,  0,  0,  0, 1 - dt*lambda ];


%%

syms qw qx qy qz

q = [qw qx qy qz];
qv = [qx qy qz];

R = eye(3) + 2*skew(qv)*(qw*eye(3)+skew(qv));
R = (qw^2 - qv*qv.') * eye(3) + 2*(qv.'*qv) + 2*qw*skew(qv);

R = R.';

Rg = R * [0;0;1];

dRgdq = jacobian(Rg, q);

dRgdq = [ -2*qy,  2*qz, -2*qw, 2*qx
           2*qx,  2*qw,  2*qz, 2*qy
              0, -4*qx, -4*qy,    0 ];

dRgdq = [ -2*qy,  2*qz, -2*qw, 2*qx
           2*qx,  2*qw,  2*qz, 2*qy
           2*qw, -2*qx, -2*qy, 2*qz ];

syms mx my mz

m = [mx; my; mz];

Rm = R*m;

dRmdq = jacobian(Rm, q);

dRmdq = [ 2*my*qz - 2*mz*qy,           2*my*qy + 2*mz*qz, 2*my*qx - 4*mx*qy - 2*mz*qw, 2*my*qw - 4*mx*qz + 2*mz*qx
          2*mz*qx - 2*mx*qz, 2*mx*qy - 4*my*qx + 2*mz*qw,           2*mx*qx + 2*mz*qz, 2*mz*qy - 4*my*qz - 2*mx*qw
          2*mx*qy - 2*my*qx, 2*mx*qz - 2*my*qw - 4*mz*qx, 2*mx*qw + 2*my*qz - 4*mz*qy,           2*mx*qx + 2*my*qy ];

dRmdq = [ 2*mx*qw + 2*my*qz - 2*mz*qy, 2*mx*qx + 2*my*qy + 2*mz*qz, 2*my*qx - 2*mx*qy - 2*mz*qw, 2*my*qw - 2*mx*qz + 2*mz*qx
          2*my*qw - 2*mx*qz + 2*mz*qx, 2*mx*qy - 2*my*qx + 2*mz*qw, 2*mx*qx + 2*my*qy + 2*mz*qz, 2*mz*qy - 2*my*qz - 2*mx*qw
          2*mx*qy - 2*my*qx + 2*mz*qw, 2*mx*qz - 2*my*qw - 2*mz*qx, 2*mx*qw + 2*my*qz - 2*mz*qy, 2*mx*qx + 2*my*qy + 2*mz*qz ];


%% Accel

% syms q_0 q_1 q_2 q_3 a_n a_e a_d g_x g_y g_z acc_b_x acc_b_y acc_b_z
% 
% h = [ acc_b_x - (a_n - g_x)*(q_0^2 + q_1^2 - q_2^2 - q_3^2) + (a_d - g_z)*(2*q_0*q_2 - 2*q_1*q_3) - (a_e - g_y)*(2*q_0*q_3 + 2*q_1*q_2)
%       acc_b_y - (a_e - g_y)*(q_0^2 - q_1^2 + q_2^2 - q_3^2) - (a_d - g_z)*(2*q_0*q_1 + 2*q_2*q_3) + (a_n - g_x)*(2*q_0*q_3 - 2*q_1*q_2)
%       acc_b_z - (a_d - g_z)*(q_0^2 - q_1^2 - q_2^2 + q_3^2) + (a_e - g_y)*(2*q_0*q_1 - 2*q_2*q_3) - (a_n - g_x)*(2*q_0*q_2 + 2*q_1*q_3) ];
% 
% x = [ q_0 q_1 q_2 q_3   a_n a_e a_d   acc_b_x acc_b_y acc_b_z ];
% 
% H = jacobian(h, x)

% H = [ 2*q_2*(a_d - g_z) - 2*q_3*(a_e - g_y) - 2*q_0*(a_n - g_x), - 2*q_2*(a_e - g_y) - 2*q_3*(a_d - g_z) - 2*q_1*(a_n - g_x),   2*q_0*(a_d - g_z) - 2*q_1*(a_e - g_y) + 2*q_2*(a_n - g_x),   2*q_3*(a_n - g_x) - 2*q_1*(a_d - g_z) - 2*q_0*(a_e - g_y), - q_0^2 - q_1^2 + q_2^2 + q_3^2,         - 2*q_0*q_3 - 2*q_1*q_2,           2*q_0*q_2 - 2*q_1*q_3, 1, 0, 0
%       2*q_3*(a_n - g_x) - 2*q_1*(a_d - g_z) - 2*q_0*(a_e - g_y),   2*q_1*(a_e - g_y) - 2*q_0*(a_d - g_z) - 2*q_2*(a_n - g_x), - 2*q_2*(a_e - g_y) - 2*q_3*(a_d - g_z) - 2*q_1*(a_n - g_x),   2*q_3*(a_e - g_y) - 2*q_2*(a_d - g_z) + 2*q_0*(a_n - g_x),           2*q_0*q_3 - 2*q_1*q_2, - q_0^2 + q_1^2 - q_2^2 + q_3^2,         - 2*q_0*q_1 - 2*q_2*q_3, 0, 1, 0
%       2*q_1*(a_e - g_y) - 2*q_0*(a_d - g_z) - 2*q_2*(a_n - g_x),   2*q_0*(a_e - g_y) + 2*q_1*(a_d - g_z) - 2*q_3*(a_n - g_x),   2*q_2*(a_d - g_z) - 2*q_3*(a_e - g_y) - 2*q_0*(a_n - g_x), - 2*q_2*(a_e - g_y) - 2*q_3*(a_d - g_z) - 2*q_1*(a_n - g_x),         - 2*q_0*q_2 - 2*q_1*q_3,           2*q_0*q_1 - 2*q_2*q_3, - q_0^2 + q_1^2 + q_2^2 - q_3^2, 0, 0, 1 ];


%% Magnetometer

% syms q_0 q_1 q_2 q_3 mag_x mag_y mag_z mag_d_x mag_d_y mag_d_z
% 
% h = [ mag_d_x + mag_x*(q_0^2 + q_1^2 - q_2^2 - q_3^2) - mag_z*(2*q_0*q_2 - 2*q_1*q_3) + mag_y*(2*q_0*q_3 + 2*q_1*q_2)
%       mag_d_y + mag_y*(q_0^2 - q_1^2 + q_2^2 - q_3^2) + mag_z*(2*q_0*q_1 + 2*q_2*q_3) - mag_x*(2*q_0*q_3 - 2*q_1*q_2)
%       mag_d_z + mag_z*(q_0^2 - q_1^2 - q_2^2 + q_3^2) - mag_y*(2*q_0*q_1 - 2*q_2*q_3) + mag_x*(2*q_0*q_2 + 2*q_1*q_3) ];
% 
% x = [ q_0 q_1 q_2 q_3 mag_x mag_y mag_z mag_d_x mag_d_y mag_d_z ];
% 
% H = jacobian(h, x);

% H = [ 2*mag_x*q_0 + 2*mag_y*q_3 - 2*mag_z*q_2, 2*mag_x*q_1 + 2*mag_y*q_2 + 2*mag_z*q_3, 2*mag_y*q_1 - 2*mag_x*q_2 - 2*mag_z*q_0, 2*mag_y*q_0 - 2*mag_x*q_3 + 2*mag_z*q_1, q_0^2 + q_1^2 - q_2^2 - q_3^2,         2*q_0*q_3 + 2*q_1*q_2,         2*q_1*q_3 - 2*q_0*q_2, 1, 0, 0
%       2*mag_y*q_0 - 2*mag_x*q_3 + 2*mag_z*q_1, 2*mag_x*q_2 - 2*mag_y*q_1 + 2*mag_z*q_0, 2*mag_x*q_1 + 2*mag_y*q_2 + 2*mag_z*q_3, 2*mag_z*q_2 - 2*mag_y*q_3 - 2*mag_x*q_0,         2*q_1*q_2 - 2*q_0*q_3, q_0^2 - q_1^2 + q_2^2 - q_3^2,         2*q_0*q_1 + 2*q_2*q_3, 0, 1, 0
%       2*mag_x*q_2 - 2*mag_y*q_1 + 2*mag_z*q_0, 2*mag_x*q_3 - 2*mag_y*q_0 - 2*mag_z*q_1, 2*mag_x*q_0 + 2*mag_y*q_3 - 2*mag_z*q_2, 2*mag_x*q_1 + 2*mag_y*q_2 + 2*mag_z*q_3,         2*q_0*q_2 + 2*q_1*q_3,         2*q_2*q_3 - 2*q_0*q_1, q_0^2 - q_1^2 - q_2^2 + q_3^2, 0, 0, 1 ];


%% GPS

% syms p_n p_e p_d v_n v_e v_d
% 
% h = [ p_n p_e p_d v_n v_e v_d ].';
% 
% x = [ p_n p_e p_d v_n v_e v_d ];
% 
% H = jacobian(h, x);

% H = [ eye(6) ];


%% UWB


%% Vision


%% Barometer


%% Altimeter

syms h
syms p0x p0y p0z

p0 = [p0x p0y p0z].';

Alt = (h - [0 0 1]*R * p0) / R(3,3);

dAltdq = jacobian(Alt, q);

dAltdq = [ - (2*p0z*qw - 2*p0y*qx + 2*p0x*qy)/(qw^2 - qx^2 - qy^2 + qz^2) - (2*qw*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2, (2*p0y*qw + 2*p0z*qx - 2*p0x*qz)/(qw^2 - qx^2 - qy^2 + qz^2) + (2*qx*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2, (2*qy*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2 - (2*p0x*qw - 2*p0z*qy + 2*p0y*qz)/(qw^2 - qx^2 - qy^2 + qz^2), - (2*p0x*qx + 2*p0y*qy + 2*p0z*qz)/(qw^2 - qx^2 - qy^2 + qz^2) - (2*qz*(h - p0z*(qw^2 - qx^2 - qy^2 + qz^2) - p0x*(2*qw*qy + 2*qx*qz) + p0y*(2*qw*qx - 2*qy*qz)))/(qw^2 - qx^2 - qy^2 + qz^2)^2];

