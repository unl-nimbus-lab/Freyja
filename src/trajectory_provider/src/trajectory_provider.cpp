/* Provides a trajectory for the vehicle to follow.

  Whatever you do here, output a time-based continous function to follow.
  This node should generate a 7 vector: [pn pe pd vn ve vd yaw]' for the vehicle
  to follow. The controller currently listens to this reference trajectory
  and updates its knowledge of the "latest" reference point.
  
  -- aj / 23rd Nov, 2017.
*/
#include <cmath>
#include <ros/ros.h>
#include <trajectory_provider/ReferenceState.h>

#define ROS_NODE_NAME "trajectory_provider"
typedef trajectory_provider::ReferenceState TrajRef;

/* HOVER AT A POINT 
TrajRef getCurrentReference( const ros::Duration &cur_time )
{
  TrajRef ref_state;
  ref_state.pn = 0.0;
  ref_state.pe = 0.0;
  ref_state.pd = -1.0;
  ref_state.vn = 0.0;
  ref_state.ve = 0.0;
  ref_state.vd = 0.0;
  ref_state.yaw = 0.0;
  ref_state.header.stamp = ros::Time::now();
  return ref_state;
}
*/

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

/*
TrajRef getCurerntReference( const ros::Duration &cur_time )
{
  TrajRef ref_state;
  float t = cur_time.toSec();
  float xvel, xpos;
  ref_state.header.stamp = ros::Time::now();
  
  ref_state.pn = 0.8*std::sin( 4.0*cur_time.toSec() );
  ref_state.pe = 0.8*std::cos( 4.0*cur_time.toSec() );
  ref_state.pd = -1.0;
  
  ref_state.vn = 3.2*std::cos( 4.0*cur_time.toSec() );
  ref_state.ve = -3.2*std::sin( 4.0*cur_time.toSec() );
  ref_state.vd = 0.0;
  
  ref_state.yaw = 0.0;
  /*
  float mod_t = std::fmod(t, 20.0);
  if( mod_t < 10.0 )
  {
    xpos = 0.5*mod_t - 2.5;
    xvel = 0.5;
  }
  else
  {
    xpos = -0.5*mod_t + 7.5;
    xvel = -0.5;
  }
  ref_state.pn = 0.0;
  ref_state.pe = xpos;
  ref_state.pd = -1.0;
  
  ref_state.vn = 0;
  ref_state.ve = xvel;
  ref_state.vd = 0.0;
  
  ref_state.yaw = 0.0;
  */
//  return ref_state;
//}

int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  ros::NodeHandle nh;

  /* Publisher for trajectory */
  ros::Publisher traj_pub;
  traj_pub = nh.advertise <TrajRef> ( "/reference_state", 1, true );

  /* How fast should a trajectory update be made? */
  ros::Rate update_rate(50);
  ros::Time init_time = ros::Time::now();

  while( ros::ok() )
  {
    TrajRef ref_state;
    ref_state = getCurrentReference( ros::Time::now() - init_time );
    traj_pub.publish( ref_state );

    ros::spinOnce();
    update_rate.sleep();
  }

  return 0;
}

