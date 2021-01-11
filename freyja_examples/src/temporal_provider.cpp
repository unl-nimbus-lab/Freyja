/* Provides a trajectory for the vehicle to follow.

  EXAMPLE FILE; ONLY FOR SUPPORT.

  Whatever you do here, output a time-based continuous function to follow.
  This node should generate a 7 vector: [pn pe pd vn ve vd yaw]' for the vehicle
  to follow. The controller currently listens to this reference trajectory
  and updates its knowledge of the "latest" reference point.
  
  -- aj / 23rd Nov, 2017.
*/
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <freyja_msgs/ReferenceState.h>

#define ROS_NODE_NAME "trajectory_provider"
typedef freyja_msgs::ReferenceState TrajRef;

#define DEG2RAD(D) ((D)*3.1415326/180.0)

// global decl for "start time". This can be reset by a callback
ros::Time init_time;

void timeResetCallback( const std_msgs::UInt8::ConstPtr &msg )
{
  if( msg -> data == 1 )
  {
    init_time = ros::Time::now();
    ROS_WARN( "%s: Time reset requested!", ROS_NODE_NAME );
  }
}

// HOVER AT A POINT 
TrajRef getHoverReference( const ros::Duration &cur_time )
{
  TrajRef ref_state;
  ref_state.pn = 0.0;
  ref_state.pe = 0.0;
  ref_state.pd = -0.75;
  ref_state.vn = 0.0;
  ref_state.ve = 0.0;
  ref_state.vd = 0.0;
  ref_state.yaw = DEG2RAD(0.0);
  ref_state.an = 0.0;
  ref_state.ae = 0.0;
  ref_state.ad = 0.0;
  ref_state.header.stamp = ros::Time::now();
  return ref_state;
}

// CIRCLE: pn = A*sin(wt), pe = A*cos(wt), vn = A*w*cos(wt) ..
TrajRef getCircleReference( const ros::Duration &cur_time, const int agg_level)
{
  // A is amplitude (radius); w angular rate such that 2pi/w = (seconds for one rev)
  float A = 0.5;
  float w = 0.5;

  // Set A and w based on agg_level
  switch(agg_level) {
    case 1 :
      break;
    case 2 :
      A = 0.5;
      w = 1;
      break;
    case 3 :
      A = 1;
      w = 3;
      break;
    default :
      ROS_WARN_STREAM("Circle aggression " << agg_level << " not supported, defaulting to agg_level 1");
  }

  float t = cur_time.toSec();

  // Create reference state
  TrajRef ref_state;
  ref_state.header.stamp = ros::Time::now();

  ref_state.pn = A*std::sin( w*t );
  ref_state.pe = A*std::cos( w*t );
  ref_state.pd = -1.0;
  
  ref_state.vn = A*w*std::cos( w*t );
  ref_state.ve = -A*w*std::sin( w*t );
  ref_state.vd = 0.0;

  ref_state.yaw = 0.0;

  // set an, ae, ad to second derivatives if needed for FF..
  return ref_state;
}

TrajRef getDefaultReference( const ros::Duration &cur_time )
{
  TrajRef ref_state;
  ref_state.pn = 0.5;
  ref_state.pe = 0.5;
  ref_state.pd = -1.0;
  ref_state.vn = 0.0;
  ref_state.ve = 0.0;
  ref_state.vd = 0.0;
  ref_state.yaw = DEG2RAD(0.0);
  ref_state.an = 0.0;
  ref_state.ae = 0.0;
  ref_state.ad = 0.0;
  ref_state.header.stamp = ros::Time::now();
  return ref_state;
}

int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  ros::NodeHandle nh, priv_nh("~");

  /* Publisher for trajectory */
  ros::Publisher traj_pub;
  traj_pub = nh.advertise <TrajRef> ( "/reference_state", 1, true );
  
  /* Create subscriber for resetting time -- restart the trajectory */
  ros::Subscriber time_reset_sub;
  time_reset_sub = nh.subscribe( "/reset_trajectory_time", 1, timeResetCallback );
  
  std::string traj_type;
  priv_nh.param( "example_traj_type", traj_type, std::string("hover") );

  /* How fast should a trajectory update be made? */
  ros::Rate update_rate(50);
  init_time = ros::Time::now();

  while( ros::ok() )
  {
    TrajRef ref_state;
    if( traj_type == "circle1" )
      ref_state = getCircleReference( ros::Time::now() - init_time, 1 );
    else if( traj_type == "circle2" )
      ref_state = getCircleReference( ros::Time::now() - init_time, 2 );
    else if( traj_type == "circle3" )
      ref_state = getCircleReference( ros::Time::now() - init_time, 3 );
    else if( traj_type == "hover" )
      ref_state = getHoverReference( ros::Time::now() - init_time );
    else
      ref_state = getDefaultReference( ros::Time::now() - init_time );
    traj_pub.publish( ref_state );

    ros::spinOnce();
    update_rate.sleep();
  }

  return 0;
}
