/* This takes the output of the controller node, which is usually target angles (rad)
  and target thrust (Newtons), and maps them into values that APM understands.
  The translation is specific to vehicles but not to frametypes (the controller makes
  no distinction between hex- or quad- or deca- copters).
  Note that APM/Pixhawk makes things easier by means of mavros that does that actual
  communication part. We only need to do the scaling here.
  
  -- aj / Nov 10, 2018.
*/


#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <common_msgs/CtrlCommand.h>

#define PITCH_MAX 0.785398
#define PITCH_MIN -0.785398
#define ROLL_MAX 0.785398
#define ROLL_MIN -0.785398
#define YAW_MAX 0.785398
#define YAW_MIN -0.785398
#define THRUST_MAX 60

#define ROS_NODE_NAME "mavros_translator"

typedef mavros_msgs::AttitudeTarget AttiTarget;
typedef common_msgs::CtrlCommand::ConstPtr CtrlInput;

uint8_t ignore_rates = AttiTarget::IGNORE_ROLL_RATE |
                        AttiTarget::IGNORE_PITCH_RATE; // |
                        //AttiTarget::IGNORE_YAW_RATE;
void sendToMavros( const double&, const double&, const double&, const double& );
void anglesToDouble( double &tgt_r, double &tgt_p, double &tgt_y )
{
  tgt_r = tgt_r / ROLL_MAX;
  tgt_p = tgt_p / PITCH_MAX;
  tgt_y = tgt_y / YAW_MAX;
}

void thrustToDouble( double &t )
{
  t /= THRUST_MAX;
}
void rpytCommandCallback( const CtrlInput &msg )
{
  /*
  The output from the controller is target roll, pitch and yawrate values,
  along with target thrust. It also outputs a bitmask for the contolled axes,
  but I'm going to ignore it for a moment.
  @TODO: do something with the control bitmask.
  */
  double tgt_roll = msg -> roll;
  double tgt_pitch = -( msg -> pitch );
  double tgt_yawrate = msg -> yaw;
  double tgt_thrust = msg -> thrust;
  
  /* map angles into -1..+1 */
  //anglesToDouble( tgt_roll, tgt_pitch, tgt_yawrate );
  /* map thrust into 0..1 */
  thrustToDouble( tgt_thrust );
  
  /* clip to hard limits */
  tgt_pitch = std::max( -1.0, std::min( 1.0, tgt_pitch ) );
  tgt_roll = std::max( -1.0, std::min( 1.0, tgt_roll ) );
  tgt_yawrate = std::max( -45.0, std::min( 45.0, tgt_yawrate ) );
  tgt_thrust = std::max( 0.1, std::min( 1.0, tgt_thrust ) );
  
  /* call mavros helper function */
  sendToMavros( tgt_pitch, tgt_roll, tgt_yawrate, tgt_thrust );
}

ros::Publisher atti_pub;
void sendToMavros( const double &p, const double &r, const double &y, const double &t )
{
  geometry_msgs::Quaternion tgt_q;
  tgt_q = tf::createQuaternionMsgFromRollPitchYaw( r, p, y );
  
  AttiTarget atti_tgt;
  atti_tgt.type_mask = ignore_rates;
  atti_tgt.orientation = tgt_q;
  atti_tgt.thrust = t;
  atti_tgt.body_rate.z = -y;
  atti_tgt.header.stamp = ros::Time::now();
    
  atti_pub.publish( atti_tgt );
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  
  ros::NodeHandle nh;
  atti_pub = nh.advertise <AttiTarget>
                            ( "/mavros/setpoint_raw/attitude", 1, true );
  ros::Subscriber rpyt_sub = nh.subscribe
                            ( "/rpyt_command", 1, rpytCommandCallback );
  ros::spin();
  return 0;
}
