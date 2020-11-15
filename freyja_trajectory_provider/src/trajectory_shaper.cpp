#include <cmath>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <freyja_msgs/ReferenceState.h>
#include <freyja_msgs/CurrentState.h>
#include <freyja_msgs/TrajectoryDebug.h>

#include "utilities.cpp"

#define ROS_NODE_NAME "trajectory_shaper"
typedef freyja_msgs::ReferenceState TrajRef;
typedef freyja_msgs::TrajectoryDebug TrajDebug;

enum class SystemState
{
  UNKNOWN = 0,
  HOVERING_SOMEWHERE,
  HOVERING_CLOSE,
  NOISY_POSITIONING,
  APPROACHING_TARGET,
};

// const defines
const float BEGIN_APPROACH_DIST = 1.0; // after this we begin approach to grasp
const int OBS_HISTORY_LEN = 200;  // collect this many observations to decide
const float ACCEPTABLE_VARIANCE = 0.2*0.2;  // this defines "steady hover"
const float APPROACH_VELOCITY = 0.3;
const float IDEAL_HOVER_ZPOS = -1.0;

// global decls
volatile float target_distance = 0.0;
std::vector<float> observation_history( OBS_HISTORY_LEN, 0.0 );
volatile SystemState estimated_system_state = SystemState::UNKNOWN;
ros::Time approach_init_time;
volatile float last_updated_z = 0;
volatile float HOVER_ZPOS_TARGET = IDEAL_HOVER_ZPOS;
volatile float approach_init_z = IDEAL_HOVER_ZPOS;


void assess_approach_confidence();
void initialise_approach_parameters();
void update_approach_location();
void hover_trajectory( TrajRef& );
void approach_trajectory( TrajRef& );

void stateCallback( const freyja_msgs::CurrentState::ConstPtr &msg )
{
  static int obs_idx = 0;
  
  // get callback data
  float pn = msg -> state_vector[0];
  float pe = msg -> state_vector[1];
  float pd = msg -> state_vector[2];
  last_updated_z = pd;
   
  
  target_distance = std::sqrt( pn*pn + pe*pe + pd*pd );
  observation_history[obs_idx++] = target_distance;
  obs_idx = (obs_idx == OBS_HISTORY_LEN) ? 0 : obs_idx;
  
  assess_approach_confidence();
}

void initialise_approach_parameters()
{
  approach_init_time = ros::Time::now();
  approach_init_z = last_updated_z;
}

void assess_approach_confidence()
{
  // good golly, this will be quite something
  
  // how steady are we?
  float mean_dist = MeanFixedLength( observation_history, OBS_HISTORY_LEN );
  float varc_dist = VarianceFixedLength( observation_history, OBS_HISTORY_LEN, mean_dist );
  
  if( estimated_system_state == SystemState::HOVERING_CLOSE )
  {
    // special case: we can transition into descending
    estimated_system_state = SystemState::APPROACHING_TARGET;
    initialise_approach_parameters();

    return;
  }
  else if( estimated_system_state == SystemState::APPROACHING_TARGET )
  {
    // special case: we are already descending, do nothing
    return;
  }
  
  if( varc_dist < ACCEPTABLE_VARIANCE )
  {
    // we are hovering pretty decently
    if( mean_dist < HOVER_ZPOS_TARGET )
    {
      // we are hovering close enough, begin approach
      estimated_system_state = SystemState::HOVERING_CLOSE;
    }
    else
      estimated_system_state = SystemState::HOVERING_SOMEWHERE;
  }
  else  
    estimated_system_state = SystemState::NOISY_POSITIONING;
  
  
}

void getCurrentReference( TrajRef &ref_state )
{
  switch( estimated_system_state )
  {
    case SystemState::UNKNOWN : hover_trajectory( ref_state );
                                break;
                                
    case SystemState::HOVERING_SOMEWHERE :
                                update_approach_location();
                                break;
                                
    case SystemState::HOVERING_CLOSE:
                                hover_trajectory( ref_state );
                                break;
                                
    case SystemState::NOISY_POSITIONING:
                                hover_trajectory( ref_state );
                                break;
                                
    case SystemState::APPROACHING_TARGET:
                                approach_trajectory( ref_state );
                                break;                            
  
    default:
                                hover_trajectory( ref_state );
  }

}



void approach_trajectory( TrajRef &ref_state )
{
  /*
  This function should (re)begin ticking clock when we (re)begin the approach.
  Keep its own time, handle external reinit using state.
  
  Also need to make sure we are doing good - if the approach is not finished in 
  finite time, then reinit. If we get too close, then reinit.
  */
  float approach_timer = ( ros::Time::now() - approach_init_time ).toSec();
  float zvel = APPROACH_VELOCITY;
  float zpos = approach_init_z + approach_timer*APPROACH_VELOCITY;
  
  // ensure that we're not plagued by false readings
  zpos = ( zpos < 0 )? zpos : approach_init_z;
  
  // check how we're doing on time..
  if( approach_timer > approach_init_z/APPROACH_VELOCITY )
  {
    zpos = approach_init_z;
    zvel = 0.0;
    initialise_approach_parameters();
  }
  
  ref_state.pd = zpos;
  ref_state.vd = zvel;
}

void hover_trajectory( TrajRef &ref_state )
{
  ref_state.pd = HOVER_ZPOS_TARGET;
  ref_state.vd = 0.0;
}

void update_approach_location()
{
  /*
  If our variance has been pretty low (meaning we are hovering), but the mean is
  not where we'd like, then update hover target to this mean and call it good.
  */
  HOVER_ZPOS_TARGET = last_updated_z;
}

int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  ros::NodeHandle nh;

  /* Publisher for trajectory and debug */
  ros::Publisher traj_pub;
  traj_pub = nh.advertise <TrajRef> ( "/reference_state", 1, true );
  
  ros::Publisher traj_debug_pub;
  traj_debug_pub = nh.advertise <TrajDebug> ( "/trajectory_debug", 1, true );
  
  /* Create subscriber for state */
  ros::Subscriber current_state_sub;
  current_state_sub = nh.subscribe( "/current_state", 1, stateCallback );

  /* How fast should a trajectory update be made? */
  ros::Rate update_rate(30);
  
  TrajRef ref_state;
  TrajDebug debug_msg;
  while( ros::ok() )
  {
    auto t = ros::Time::now();
    ref_state.header.stamp = t;
    getCurrentReference( ref_state );
    traj_pub.publish( ref_state );
    
    debug_msg.header.stamp = t;
    debug_msg.system_state = (uint8_t)estimated_system_state;
    debug_msg.hover_z_target = HOVER_ZPOS_TARGET;
    traj_debug_pub.publish( debug_msg );

    ros::spinOnce();
    update_rate.sleep();
  }

  return 0;
}
