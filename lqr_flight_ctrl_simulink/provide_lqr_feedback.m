function U = provide_lqr_feedback( state_vec )

  % Matlab interpreted function to close the loop with some form of
  % control - I think this is going to be -LK * X.
  % Note: X is a 7-vector [x y z xdot ydot zdot yawrate]
  % Note: XREF should be a 4 vector [xt yt zt yawt]
  % Output is a 4 vector: [Thrust pitch_angle roll_angle yaw_angle]

  global LK total_mass u_collector thrust_collector
  
  rpy = state_vec(1:3);
  yaw = rpy(3);
  rpy_rates = state_vec(4:6);
  reduced_state = [state_vec(7:12); yaw];
  cur_time = state_vec(end);
  
  Ryaw = [  cos(yaw)   sin(yaw)  0; ...
           -sin(yaw)   cos(yaw)  0; ...
            0           0        1 ];

  
  % For quad_model2
  
  % compute LQR's response
  ctrl_input = -1 * LK * reduced_state;
  u_collector = [u_collector, ctrl_input];
  ctrl_input(3) = min( ctrl_input(3), 8.0 );
  ctrl_input(3) = ctrl_input(3) - 9.81;
  
  % get the current cmd ref
  %ctrl_input = ctrl_input + cmd_ref_provider( cur_time ) - 9.81;
  
  % decompose into [up; upsi] and invert
  T = total_mass * norm( ctrl_input(1:3) );% - total_mass*9.81;
  thrust_collector = [thrust_collector; T];
  
  z = Ryaw * ctrl_input(1:3) * (-total_mass/T);
  roll = asin( -z(2) );
  pitch = atan( z(1)/z(3) );
  %yawrate = rpy_rates(3) * cos(rpy(2)) * cos(rpy(1)) - rpy_rates(2)*sin(rpy(1));
  yawrate = ctrl_input(4);

  %pitch = min( 0.9, max( -0.9, pitch ) );
  %roll = min( 0.9, max( -0.9, roll ) );
  %yawrate = min( 1.0, max( -1.0, yawrate ) );
  U = [T; roll; pitch; yawrate]; %ctrl_input(3); T];
  %U = [2.9; -0.1; 0; 0];
end