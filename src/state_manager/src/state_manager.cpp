
#include "state_manager.h"
#include "aj_filter_collection.cpp"

#define ROS_NODE_NAME "state_manager"
StateManager::StateManager() : nh_()
{
  state_vector_.resize( STATE_VECTOR_LEN );
  
  std::string vicon_topic( "/vicon/ARGENTINA/ARGENTINA" );
  nh_.param( "vicon_object", vicon_topic, vicon_topic );
  
  /* Associate vicon callback */
  vicon_data_sub_ = nh_.subscribe( vicon_topic, 1,
                                    &StateManager::viconCallback, this );
                                    
  std::string vehicle_topic( "/asctec_onboard_data" );
  nh_.param( "vehicle_topic", vehicle_topic, vehicle_topic );
  /*Associate a vehicle data callback */
  asctec_data_sub_ = nh_.subscribe( vehicle_topic, 1,
                                &StateManager::asctecDataCallback, this );
                                
  /* Announce state publisher */
  state_pub_ = nh_.advertise <state_manager::CurrentState>
                  ( "current_state", 1, true ); 

  /* Init filter: mean 10, stddev = 5 */
  filter_len_ = 21;
  std::vector<double> fc = { 0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484, 
                            0.0579, 0.0666, 0.0737, 0.0782, 0.0798, 0.0782,
                            0.0737, 0.0666, 0.0579, 0.0484, 0.0388, 0.0299,
                            0.0222, 0.0158, 0.0108};
  AjFilterCollection::initGaussianFilter( fc, filter_len_ );
  prev_pn_.resize( filter_len_ );
  prev_pe_.resize( filter_len_ );
  prev_pd_.resize( filter_len_ );
  lastUpdateTime_ = ros::Time::now();
}

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
//  ROS_INFO("%0.3f", x);
  prev_pn_.erase( prev_pn_.begin() );
  prev_pn_.push_back( x );
  prev_pe_.erase( prev_pe_.begin() );
  prev_pe_.push_back( y );
  prev_pd_.erase( prev_pd_.begin() );
  prev_pd_.push_back( z );
  
  AjFilterCollection::filterObservations( "gauss", prev_pn_, x );
  AjFilterCollection::filterObservations( "gauss", prev_pe_, y );
  AjFilterCollection::filterObservations( "gauss", prev_pd_, z );
//  ROS_INFO("%0.3f", x);
  /* positions */
  state_vector_[0] = x;
  state_vector_[1] = y;
  state_vector_[2] = z;
  
  /* velocities */
  state_vector_[3] = ( x - last_pn_ )/time_since;
  state_vector_[4] = ( y - last_pe_ )/time_since;
  state_vector_[5] = ( z - last_pd_ )/time_since;
  
  /* rpy */
  tf::Quaternion q;
  tf::quaternionMsgToTF( msg -> transform.rotation, q );
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY( roll, pitch, yaw );
  state_vector_[6] = roll;
  state_vector_[7] = pitch;
  state_vector_[8] = -yaw;
  
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
  state_manager::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );
}

void StateManager::asctecDataCallback( const asctec_handler::AsctecData::ConstPtr &msg )
{
  double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
  double x, y, z;
  
  x = msg -> best_lat;
  y = msg -> best_lon;
  z = -(msg -> hgt);
  
  prev_pn_.erase( prev_pn_.begin() );
  prev_pn_.push_back( x );
  prev_pe_.erase( prev_pe_.begin() );
  prev_pe_.push_back( y );
  prev_pd_.erase( prev_pd_.begin() );
  prev_pd_.push_back( z );
  
  AjFilterCollection::filterObservations( "gauss", prev_pn_, x );
  AjFilterCollection::filterObservations( "gauss", prev_pe_, y );
  AjFilterCollection::filterObservations( "gauss", prev_pd_, z );
//  ROS_INFO("%0.3f", x);
  /* positions */
  state_vector_[0] = x;
  state_vector_[1] = y;
  state_vector_[2] = z;
  
  /* velocities from vehicle */
  state_vector_[3] = msg -> best_sp_x;
  state_vector_[4] = msg -> best_sp_y;
  state_vector_[5] = msg -> best_sp_z;
  
  /* Attitude */
  state_vector_[6] = msg -> roll_angle;
  state_vector_[7] = msg -> pitch_angle;
  state_vector_[8] = msg -> yaw_angle;
  
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
  state_manager::CurrentState state_msg;
  state_msg.header.stamp = ros::Time::now();
  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  state_pub_.publish( state_msg );
}

void StateManager::computeVelocity( float &x, float &y, float &z, float &t )
{

  /*
  AjFilterCollection::filterObservations( "gauss", prev_pn_, &state_vector_[3] );
  AjFilterCollection::filterObservations( "gauss", prev_pe_, &state_vector_[4] );
  AjFilterCollection::filterObservations( "gauss", prev_pd_, &state_vector_[5] );
  */
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  StateManager sm;
  ros::spin();
  
  return 0;
}
