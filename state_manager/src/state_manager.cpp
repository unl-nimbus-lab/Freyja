
#include "state_manager.h"
#include "callback_implementations.cpp"

#define ROS_NODE_NAME "state_manager"


StateManager::StateManager() : nh_(), priv_nh_("~")
{
  state_vector_.resize( STATE_VECTOR_LEN );
  
  /* find out what the source of state information is:
      > "vicon" for a motion capture system with ENU frame
      > "asctec" for Ascending Tech autopilots (see corresponding handler)
      > "apm" for ardupilot (uses mavros, see corresponding handler)
      > "onboard_camera" for a downward-facing camera (pose+velocity)
  */ 
  std::string default_state_source( "vicon" );
  priv_nh_.param( "state_source", state_source_, default_state_source );
  
  if( state_source_ == "vicon" )
    initViconManager();
  else if( state_source_ == "asctec" )
    initAsctecManager();
  else if (state_source_ == "apm" )
    initPixhawkManager();
  else if( state_source_ == "onboard_camera" )
    initCameraManager();


  /* Announce state publisher */
  state_pub_ = nh_.advertise <freyja_msgs::CurrentState>
                  ( "current_state", 1, true ); 
  
  /* Instantiate filters. Useful for vicon. Autopilots have their own filters */                
  priv_nh_.param( "filter_type", filter_type_, std::string("lwma") );
  priv_nh_.param( "filter_length", filter_len_, int(21) );

  if( filter_type_ == "gauss" )
  {
    std::vector<double> fc;
    /* Init filter: mean 10, stddev = 5 */
    if( filter_len_ == 21 )
      fc = { 0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484, 0.0579, 0.0666,
             0.0737, 0.0782, 0.0798, 0.0782, 0.0737, 0.0666, 0.0579, 0.0484,
             0.0388, 0.0299, 0.0222, 0.0158, 0.0108 };
    else if( filter_len_ == 5 )
      fc = { 0.1534,	0.2214,	0.2504,	0.2214,	0.1534 };

    pose_filter_ = FreyjaFilters( filter_len_, "gauss", "~", fc );
    rate_filter_ = FreyjaFilters( filter_len_, "gauss", "~", fc );
    ROS_INFO( "Gaussian filter init!" );
  }
  else if( filter_type_ == "lwma" )
  {
    /* The init function automatically fills in the coeffs for lwma */
    //filter_len_ = 20;
    pose_filter_ = FreyjaFilters( filter_len_, "lwma", "cubic" );
    rate_filter_ = FreyjaFilters( filter_len_, "lwma", "cubic" );
    ROS_INFO( "LWMA filter init!" );
  }
  else if( filter_type_ == "median" )
  {
    pose_filter_ = FreyjaFilters( -1, "median", "~" );
    rate_filter_ = FreyjaFilters( -1, "median", "~" );
    filter_len_ = pose_filter_.getCurrentFilterLen();
  }
  
  // init history containers
  prev_pn_.resize( filter_len_ );
  prev_pe_.resize( filter_len_ );
  prev_pd_.resize( filter_len_ );
  
  prev_vn_.resize( filter_len_ );
  prev_ve_.resize( filter_len_ );
  prev_vd_.resize( filter_len_ );
  lastUpdateTime_ = ros::Time::now();
  have_location_fix_ = false;
}

void StateManager::initViconManager()
{
  std::string vicon_topic( "/vicon/FENRIR/FENRIR" );
  priv_nh_.param( "vicon_topic", vicon_topic, vicon_topic );

  /* Associate vicon callback */
  vicon_data_sub_ = nh_.subscribe( vicon_topic, 1,
                                    &StateManager::viconCallback, this,
                                    ros::TransportHints().tcpNoDelay() );
}

void StateManager::initAsctecManager()
{
  std::string vehicle_topic( "/asctec_onboard_data" );
  nh_.param( "vehicle_topic", vehicle_topic, vehicle_topic );
  /*Associate a vehicle data callback */
  asctec_data_sub_ = nh_.subscribe( vehicle_topic, 1,
                                &StateManager::asctecDataCallback, this );
}

void StateManager::initPixhawkManager()
{
  have_arming_origin_ = false;
  mavros_gpsraw_sub_ = nh_.subscribe( "/mavros/global_position/global", 1,
                                &StateManager::mavrosGpsRawCallback, this );
  mavros_gpsodom_sub_ = nh_.subscribe( "/mavros/global_position/local", 1,
                                &StateManager::mavrosGpsOdomCallback, this,
                                ros::TransportHints().tcpNoDelay() );
  mavros_rtk_sub_ = nh_.subscribe( "/ublox_f9p_rtkbaseline", 1,
                                &StateManager::mavrosRtkBaselineCallback, this );
  compass_sub_ = nh_.subscribe( "/mavros/global_position/compass_hdg", 1, 
				                &StateManager::mavrosCompassCallback, this );
				                
  maplock_srv_ = nh_.advertiseService( "/lock_arming_mapframe", 
                        &StateManager::maplockArmingHandler, this );
}

void StateManager::initCameraManager()
{
  camera_estimate_sub_ = nh_.subscribe( "/onboard_camera/position_velocity", 1,
                                &StateManager::cameraUpdatesCallback, this );
}


int main( int argc, char **argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  StateManager sm;
  ros::spin();
  
  return 0;
}
