void measure_offsets() {
    int iterations = 10;

    double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

    if (EZchassis.odom_tracker_left) EZchassis.odom_tracker_left->reset();
    if (EZchassis.odom_tracker_right) EZchassis.odom_tracker_right->reset();
    if (EZchassis.odom_tracker_back) EZchassis.odom_tracker_back->reset();
    if (EZchassis.odom_tracker_front) EZchassis.odom_tracker_front->reset();

    for (int i = 0; i < iterations; i++) {
        EZchassis.pid_targets_reset();
        EZchassis.drive_imu_reset();
        EZchassis.drive_sensor_reset();
        EZchassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
        EZchassis.odom_xyt_set(0, 0, 0);

        double imu_start = EZchassis.odom_theta_get();
        double target = (i % 2 == 0) ? 90 : 270;

        EZchassis.pid_turn_set(target, 63, ez::raw);
        EZchassis.pid_wait();
        pros::delay(250);

        double t_delta = ez::util::to_rad(fabs(ez::util::wrap_angle(EZchassis.odom_theta_get() - imu_start)));

        double l_delta = EZchassis.odom_tracker_left ? EZchassis.odom_tracker_left->get() : 0.0;
        double r_delta = EZchassis.odom_tracker_right ? EZchassis.odom_tracker_right->get() : 0.0;
        double b_delta = EZchassis.odom_tracker_back ? EZchassis.odom_tracker_back->get() : 0.0;
        double f_delta = EZchassis.odom_tracker_front ? EZchassis.odom_tracker_front->get() : 0.0;

        l_offset += l_delta / t_delta;
        r_offset += r_delta / t_delta;
        b_offset += b_delta / t_delta;
        f_offset += f_delta / t_delta;
    }

    l_offset /= iterations;
    r_offset /= iterations;
    b_offset /= iterations;
    f_offset /= iterations;

    if (EZchassis.odom_tracker_left) EZchassis.odom_tracker_left->distance_to_center_set(l_offset);
    if (EZchassis.odom_tracker_right) EZchassis.odom_tracker_right->distance_to_center_set(r_offset);
    if (EZchassis.odom_tracker_back) EZchassis.odom_tracker_back->distance_to_center_set(b_offset);
    if (EZchassis.odom_tracker_front) EZchassis.odom_tracker_front->distance_to_center_set(f_offset);
}