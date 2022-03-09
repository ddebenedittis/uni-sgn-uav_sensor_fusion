function anim_states_add(anim_est_states, anim_real_states, time_x, state, real_gyro_bias, real_acc_bias, trajAcc, real_mag_dist, trajVel, real_ground_height)
    addpoints(anim_est_states( 1),time_x, state( 5));
    addpoints(anim_est_states( 2),time_x, state( 6));
    addpoints(anim_est_states( 3),time_x, state( 7));

    addpoints(anim_est_states( 4),time_x, state( 8));
    addpoints(anim_est_states( 5),time_x, state( 9));
    addpoints(anim_est_states( 6),time_x, state(10));

    addpoints(anim_est_states( 7),time_x, state(11));
    addpoints(anim_est_states( 8),time_x, state(12));
    addpoints(anim_est_states( 9),time_x, state(13));

    addpoints(anim_est_states(10),time_x, state(14));
    addpoints(anim_est_states(11),time_x, state(15));
    addpoints(anim_est_states(12),time_x, state(16));

    addpoints(anim_est_states(13),time_x, state(20));
    addpoints(anim_est_states(14),time_x, state(21));
    addpoints(anim_est_states(15),time_x, state(22));

    addpoints(anim_est_states(16),time_x, state(23));
    
    % Real
    addpoints(anim_real_states( 1),time_x, real_gyro_bias(1));
    addpoints(anim_real_states( 2),time_x, real_gyro_bias(2));
    addpoints(anim_real_states( 3),time_x, real_gyro_bias(3));

    addpoints(anim_real_states( 4),time_x, real_acc_bias(1));
    addpoints(anim_real_states( 5),time_x, real_acc_bias(2));
    addpoints(anim_real_states( 6),time_x, real_acc_bias(3));

    addpoints(anim_real_states( 7),time_x, trajAcc(1));
    addpoints(anim_real_states( 8),time_x, trajAcc(2));
    addpoints(anim_real_states( 9),time_x, trajAcc(3));

    addpoints(anim_real_states(10),time_x, real_mag_dist(1));
    addpoints(anim_real_states(11),time_x, real_mag_dist(2));
    addpoints(anim_real_states(12),time_x, real_mag_dist(3));

    addpoints(anim_real_states(13),time_x, trajVel(1));
    addpoints(anim_real_states(14),time_x, trajVel(2));
    addpoints(anim_real_states(15),time_x, trajVel(3));

    addpoints(anim_real_states(16),time_x, real_ground_height);
    
    drawnow limitrate   % limits the maximum update frequency
end