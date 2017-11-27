/* Provides a trajectory for the vehicle to follow.

  Whatever you do here, output a time-based continous function to follow.
  This node should generate a 7 vector: [pn pe pd vn ve vd yaw]' for the vehicle
  to follow. The controller currently listens to this reference trajectory
  and updates its knowledge of the "latest" reference point.
  
  -- aj / 23rd Nov, 2017.
*/

#include <ros/ros.h>
#include <trajectory_provider/ReferenceState.h>

#define ROS_NODE_NAME "trajectory_provider"
typedef trajectory_provider::ReferenceState TrajRef;

TrajRef getCurerntReference( const ros::Time &cur_time )
{
  TrajRef ref_state;
  
  ref_state.header.stamp = cur_time;
  ref_state.pn = 1.2*std::sin( cur_time.toSec() );
  ref_state.pe = 1.2*std::cos( cur_time.toSec() );
  ref_state.pd = -1.0;
  
  ref_state.vn = 1.2*std::cos( cur_time.toSec() );
  ref_state.ve = -1.2*std::sin( cur_time.toSec() );
  ref_state.vd = 0.0;
  
  ref_state.yaw = 0.0;
  
  return ref_state;
}

int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  ros::NodeHandle nh;
  
  /* Publisher for trajectory */
  ros::Publisher traj_pub;
  traj_pub = nh.advertise <TrajRef> ( "/reference_state", 1, true );
  
  /* How fast should a trajectory update be made? */
  ros::Rate update_rate(50);
  
  while( ros::ok() )
  {
    TrajRef ref_state;
    ref_state = getCurerntReference( ros::Time::now() );
    traj_pub.publish( ref_state );
    
    ros::spinOnce();
    update_rate.sleep();
  }
  
  return 0;
}

