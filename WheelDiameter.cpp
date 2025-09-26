double calcwheelDiam(double disttravR){
  double wheelval;
  double disttravS;
  double prevD = (EZchassis.odom_tracker_left->wheel_diameter_get() + EZchassis.odom_tracker_right->wheel_diameter_get())/2;

  disttravS = (EZchassis.odom_tracker_left->get() + EZchassis.odom_tracker_right->get())/2;
  wheelval = prevD * (disttravR / disttravS);

  return wheelval;

}