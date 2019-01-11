void StateManager::viconCallback( const TFStamped::ConstPtr &msg )
{
  /* Handle most of the state information directly, except for velocity
    and that needs to be computed after filtering positions */
    
  /* Note that Vicon gives us ENU, while we use NED */
  
  double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
  
  double x, y, z;
  x = msg -> transform.translation.y;
  y = msg -> transform.translation.x;
  z = -(msg -> transform.translation.z);

  prev_pn_.erase( prev_pn_.begin() );
  prev_pn_.push_back( x );
  prev_pe_.erase( prev_pe_.begin() );
  prev_pe_.push_back( y );
  prev_pd_.erase( prev_pd_.begin() );
  prev_pd_.push_back( z );

  //AjFilterCollection::filterObservations( filter_type_, prev_pn_, x );
  //AjFilterCollection::filterObservations( filter_type_, prev_pe_, y );
  //AjFilterCollection::filterObservations( filter_type_, prev_pd_, z );
  pose_filter_.filterObservations( prev_pn_, prev_pe_, prev_pd_, x, y, z );

  /* positions */
  state_vector_[0] = x;
  state_vector_[1] = y;
  state_vector_[2] = z;
  
  /* velocities */
  double vx, vy, vz;
  vx = ( x - last_pn_ )/time_since; //[3]
  vy = ( y - last_pe_ )/time_since; //[4]
  vz = ( z - last_pd_ )/time_since;
  prev_vn_.erase( prev_vn_.begin() );
  prev_vn_.push_back( vx );
  prev_ve_.erase( prev_ve_.begin() );
  prev_ve_.push_back( vy );
  prev_vd_.erase( prev_vd_.begin() );
  prev_vd_.push_back( vz );
  //AjFilterCollection::filterObservations( filter_type_, prev_vn_, vx );
  //AjFilterCollection::filterObservations( filter_type_, prev_ve_, vy );
  //AjFilterCollection::filterObservations( filter_type_, prev_vd_, vz );
  rate_filter_.filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );
  state_vector_[3] = vx;
  state_vector_[4] = vy;
  state_vector_[5] = vz;
  
  /* rpy */
  tf::Quaternion q;
  tf::quaternionMsgToTF( msg -> transform.rotation, q );
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY( roll, pitch, yaw );
  state_vector_[6] = roll;
  state_vector_[7] = pitch;
  state_vector_[8] = -yaw;

  
  /* rpy rates (no filter yet, use with caution!) */
  state_vector_[9] = ( roll - last_roll_ )/time_since;
  state_vector_[10] = ( pitch - last_pitch_ )/time_since;
  state_vector_[11] = ( yaw - last_yaw_ )/time_since;
  
  /* age of this data */
  state_vector_[12] = time_since;
  
  /* Update the current time this happened */
  lastUpdateTime_ = ros::Time::now();
  
  /* bookkeeping stuff */
  last_pn_ = state_vector_[0];
  last_pe_ = state_vector_[1];
  last_pd_ = state_vector_[2];
  last_roll_ = roll;
  last_pitch_ = pitch;
  last_yaw_ = yaw;
  
  /* Copy over and publish right away */
  common_msgs::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );
}

void StateManager::asctecDataCallback( const common_msgs::AsctecData::ConstPtr &msg )
{
  double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
  double x, y, z, vx, vy, vz;
  
  if( (msg -> motor1rpm) > 0 && !have_location_fix_ )
  {
    have_location_fix_ = true;
    home_lat_ = (msg -> best_lat)/10000000.0;
    home_lon_ = (msg -> best_lon)/10000000.0;
  }
  
  if( !have_location_fix_ )
    return;
  
  x = ( (msg->best_lat)/10000000.0 - home_lat_ )*111050.51;
  y = ( (msg->best_lon)/10000000.0 - home_lon_ )*84356.28;
  z = -(msg -> hgt)/1000.0;
  
  vy = (msg -> best_sp_x)/1000.0;
  vx = (msg -> best_sp_y)/1000.0;
  vz = -(msg -> best_sp_z)/1000.0;
  prev_vn_.erase( prev_vn_.begin() );
  prev_vn_.push_back( vx );
  prev_ve_.erase( prev_ve_.begin() );
  prev_ve_.push_back( vy );
  prev_vd_.erase( prev_vd_.begin() );
  prev_vd_.push_back( vz );
  rate_filter_.filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );
  /*
  AjFilterCollection::filterObservations( "gauss", prev_pn_, x );
  AjFilterCollection::filterObservations( "gauss", prev_pe_, y );
  AjFilterCollection::filterObservations( "gauss", prev_pd_, z );
  */
  /* positions */
  state_vector_[0] = x;
  state_vector_[1] = y;
  state_vector_[2] = z;
  
  /* velocities from vehicle */
  state_vector_[3] = vx; //(msg -> best_sp_x)/1000.0;
  state_vector_[4] = vy; //(msg -> best_sp_y)/1000.0;
  state_vector_[5] = vz; //(msg -> best_sp_z)/1000.0;
  
  /* Attitude */
  state_vector_[6] = (msg -> roll_angle)/1000.0;
  state_vector_[7] = (msg -> pitch_angle)/1000.0;
  double yaw_c = DEG2RAD( (msg -> yaw_angle)/1000.0 );
  state_vector_[8] = ( yaw_c > AJ_PI )? yaw_c - 2*AJ_PI : yaw_c;
  
  /* rpy rates */
  state_vector_[9] = 0.0;
  state_vector_[10] = 0.0;
  state_vector_[11] = 0.0;
  
  /* age */
  state_vector_[12] = time_since;
  
  /* Update the current time this happened */
  lastUpdateTime_ = ros::Time::now();
  
  /* bookkeeping stuff */
  last_pn_ = state_vector_[0];
  last_pe_ = state_vector_[1];
  last_pd_ = state_vector_[2];
  
  /* Copy over and publish right away */
  common_msgs::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );
}

void StateManager::mavrosGpsCallback( const sensor_msgs::NavSatFix::ConstPtr& msg )
{
  if( msg -> status.status >= 0 && !have_location_fix_ )
  {
    /* first location fix */
    home_lat_ = msg -> latitude;
    home_lon_ = msg -> longitude;
    have_location_fix_ = true;
  }
  else
    return;
    
  state_vector_[0] = ( (msg->latitude)/10000000.0 - home_lat_ )*111050.51;
  state_vector_[1] = ( (msg->longitude)/10000000.0 - home_lon_ )*84356.28;
  
}

void StateManager::mavrosGpsVelCallback( const TwStamped::ConstPtr &msg )
{
  if( !have_location_fix_ )
    return;
  
  double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
  
  state_vector_[3] = msg -> twist.linear.x;
  state_vector_[4] = msg -> twist.linear.y;
  state_vector_[5] = msg -> twist.linear.z;
  
  state_vector_[12] = time_since;
  
  lastUpdateTime_ = ros::Time::now();

  common_msgs::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );

}
