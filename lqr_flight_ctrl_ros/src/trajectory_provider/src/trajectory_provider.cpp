/* Provides a trajectory for the vehicle to follow.

  Whatever you do here, output a time-based continous function to follow.
  This node should generate a 7 vector: [pn pe pd vn ve vd yaw]' for the vehicle
  to follow. The controller currently listens to this reference trajectory
  and updates its knowledge of the "latest" reference point.
  
  -- aj / 23rd Nov, 2017.
*/
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <common_msgs/ReferenceState.h>

#define ROS_NODE_NAME "trajectory_provider"
typedef common_msgs::ReferenceState TrajRef;

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
  ref_state.pd = -1.0;
  ref_state.vn = 0.0;
  ref_state.ve = 0.0;
  ref_state.vd = 0.0;
  ref_state.yaw = 0.0;
  ref_state.an = 0.0;
  ref_state.ae = 0.0;
  ref_state.ad = 0.0;
  ref_state.header.stamp = ros::Time::now();
  return ref_state;
}
// CIRCLE: pn = A*sin(wt), pe = A*cos(wt), vn = A*w*cos(wt) ..
TrajRef getCircleReference( const ros::Duration &cur_time )
{
  TrajRef ref_state;
  float t = cur_time.toSec();
  float xvel, xpos;
  ref_state.header.stamp = ros::Time::now();
  
  ref_state.pn = 1.0*std::sin( 3.0*t );
  ref_state.pe = 1.0*std::cos( 3.0*t );
  ref_state.pd = -1.0;
  
  ref_state.vn = 3.0*std::cos( 3.0*t );
  ref_state.ve = -3.0*std::sin( 3.0*t );
  ref_state.vd = 0.0;
  
  ref_state.yaw = 0.0;

  // set an, ae, ad to second derivatives if needed for FF..
  return ref_state;
}

TrajRef getDefaultReference( const ros::Duration &cur_time )
{
  TrajRef ref_state;
  ref_state.pn = 1.0;
  ref_state.pe = 1.0;
  ref_state.pd = -1.5;
  ref_state.vn = 0.0;
  ref_state.ve = 0.0;
  ref_state.vd = 0.0;
  ref_state.yaw = DEG2RAD(-120.0);
  ref_state.an = 0.0;
  ref_state.ae = 0.0;
  ref_state.ad = 0.0;
  ref_state.header.stamp = ros::Time::now();
  return ref_state;
}

/*
  Ascend at point A, hover, go to B ..
  hover at B, descend, ascend at B, hover ..
  go to A, hover, descend at A .. Repeat.
  
TrajRef getCurrentReference( const ros::Duration &cur_time )
{
  TrajRef ref_state;
  float t = cur_time.toSec();
  float mod_t = std::fmod( t, 72.0 );
  float xpos, ypos, zpos, xvel, yvel, zvel;
  ref_state.header.stamp = ros::Time::now();
  if( mod_t < 5.0 )
  {
    // ascend at A
    xpos = -2.0;
    ypos = 0.0;
    xvel = 0.0;
    yvel = 0.0;
    zpos = -mod_t*9.0/50.0 - 0.1;
    zvel = -9.0/50.0;
  }
  else if( mod_t < 8.0 )
  {
    // hover at A
    xpos = -2.0;
    ypos = 0.0;
    xvel = 0.0;
    yvel = 0.0;
    zpos = -1.0;
    zvel = 0.0;
  }
  else if( mod_t < 28.0 )
  {
    // from A to B
    float lat_timer = mod_t - 8.0;
    xpos = lat_timer * 0.2 - 2.0;
    ypos = 0.0;
    xvel = 0.2;
    yvel = 0.0;
    zpos = -1.0;
    zvel = 0.0;
  }
  else if( mod_t < 31.0 )
  {
    // hover at B
    xpos = 2.0;
    ypos = 0.0;
    xvel = 0.0;
    yvel = 0.0;
    zpos = -1.0;
    zvel = 0.0;
  }
  else if( mod_t < 36.0 )
  {
    // descend at B
    float desc_timer = mod_t - 31.0;
    xpos = 2.0;
    ypos = 0.0;
    xvel = 0.0;
    yvel = 0.0;
    zpos = desc_timer*9.0/50.0 - 1.0;
    zvel = 9.0/50.0;
  }
  else if( mod_t < 41.0 )
  {
    // ascend at B
    float asc_timer = mod_t - 36.0;
    xpos = 2.0;
    ypos = 0.0;
    xvel = 0.0;
    yvel = 0.0;
    zpos = -asc_timer*9.0/50.0 - 0.1;
    zvel = -9.0/50.0;
  }
  else if( mod_t < 44.0 )
  {
    // hover at B
    xpos = 2.0; ypos = 0.0;
    xvel = yvel = 0.0;
    zpos = -1.0;
    zvel = 0.0;
  }
  else if( mod_t < 64.0 )
  {
    // from B to A
    float lat_timer = mod_t - 44.0;
    xpos = -lat_timer*0.2 + 2.0;
    ypos = 0.0;
    xvel = -0.2;
    yvel = 0.0;
    zpos = -1.0;
    zvel = 0.0;
  }
  else if( mod_t < 67.0 )
  {
    // hover at A
    xpos = -2.0; ypos = 0.0;
    xvel = 0.0; yvel = 0.0;
    zpos = -1.0;
    zvel = 0.0;
  }
  else if( mod_t < 72.0 )
  {
    // descend at B
   float desc_timer = mod_t - 67.0;
   xpos = -2.0; ypos = 0.0;
   xvel = 0.0; yvel = 0.0;
   zpos = desc_timer*9.0/50.0 - 1.0;
   zvel = 9.0/50.0;
  }
  
  ref_state.pn = ypos;
  ref_state.pe = xpos;
  ref_state.pd = zpos;
  ref_state.vn = yvel;
  ref_state.ve = xvel;
  ref_state.vd = zvel;
  ref_state.yaw = 0.0;
  return ref_state;
}
*/

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
    if( traj_type == "circle" )
      ref_state = getCircleReference( ros::Time::now() - init_time );
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

