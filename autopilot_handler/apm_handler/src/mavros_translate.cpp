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
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>

#include <std_srvs/SetBool.h>

#include <freyja_msgs/CtrlCommand.h>

#define PITCH_MAX 0.785398
#define PITCH_MIN -0.785398
#define ROLL_MAX 0.785398
#define ROLL_MIN -0.785398
#define YAW_MAX 0.785398
#define YAW_MIN -0.785398

#define ROS_NODE_NAME "mavros_translator"

double THRUST_MAX = 1.0;
double THRUST_MIN = 0.02;
double THRUST_SCALER = 200.0;

typedef mavros_msgs::AttitudeTarget AttiTarget;
typedef freyja_msgs::CtrlCommand::ConstPtr CtrlInput;

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

inline void thrustToDouble( double &t )
{
  t /= THRUST_SCALER;
}
void rpytCommandCallback( const CtrlInput &msg )
{
  /*
  The output from the controller is target roll, pitch and yaw/yawrate values,
  along with target thrust. The bit mask contains additional information flags.
  @TODO: do something with the control bitmask if needed.
  */
  double tgt_roll = msg -> roll;
  double tgt_pitch = -( msg -> pitch );
  double tgt_yawrate = msg -> yaw;
  double tgt_thrust = msg -> thrust;
  
  /* map angles into -1..+1 -- some systems might need this */
  //anglesToDouble( tgt_roll, tgt_pitch, tgt_yawrate );
  
  /* map thrust into 0..1 */
  thrustToDouble( tgt_thrust );
  
  /* clip to hard limits */
  tgt_pitch = std::max( -1.0, std::min( 1.0, tgt_pitch ) );
  tgt_roll = std::max( -1.0, std::min( 1.0, tgt_roll ) );
  tgt_yawrate = std::max( -45.0, std::min( 45.0, tgt_yawrate ) );
  tgt_thrust = std::max( THRUST_MIN, std::min( THRUST_MAX, tgt_thrust ) );
  
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

ros::ServiceClient map_lock;
ros::ServiceClient bias_comp;
bool vehicle_armed_ = false;
bool in_comp_mode_ = false;
void mavrosStateCallback( const mavros_msgs::State::ConstPtr &msg )
{
  /* call map lock/unlock service when arming/disarming */
  std_srvs::SetBool lockreq;
  if( msg->armed == true && !vehicle_armed_ )
  {
    vehicle_armed_ = true;
    lockreq.request.data = true;
    map_lock.call( lockreq );
  }
  else if( msg->armed == false && vehicle_armed_ )
  {
    vehicle_armed_ = false;
    lockreq.request.data = false;
    map_lock.call( lockreq );
  }
  
  /* call bias compensation service when switching in/out of computer */
  std_srvs::SetBool biasreq;
  if( msg->mode == "CMODE(25)" && !in_comp_mode_ )
  {
    in_comp_mode_ = true;
    biasreq.request.data = true;
    bias_comp.call( biasreq );
  }
  else if( msg->mode != "CMODE(25)" && in_comp_mode_ )
  {
    in_comp_mode_ = false;
    biasreq.request.data = false;
    bias_comp.call( biasreq );
  }
}


void rcdataCallback( const mavros_msgs::RCIn::ConstPtr &msg )
{
  /*  !!NOTE: mavros can publish a zero length array if no rc.
    Check array length first, or wrap in try-catch */
  /*
  try
  {
    if( msg -> channels[5] < 1500 )
    {
      // do something
    }
    else if( msg -> channels[5] >=1500 )
    {
      // do something
    }
  }
  catch( ... )
  {
    // skip
  }
  
  */
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  ros::NodeHandle nh, priv_nh("~");
  
  /* Load parameters */
  priv_nh.param( "thrust_scaler", THRUST_SCALER, double(200.0) );
  priv_nh.param( "min_thrust_clip", THRUST_MIN, double(0.04) );
  priv_nh.param( "max_thrust_clip", THRUST_MAX, double(1.0) );
  
  atti_pub = nh.advertise <AttiTarget>
                            ( "/mavros/setpoint_raw/attitude", 1, true );
  ros::Subscriber rpyt_sub = nh.subscribe
                            ( "/rpyt_command", 1, rpytCommandCallback );
                            
  ros::Subscriber mavstate_sub = nh.subscribe
                            ( "/mavros/state", 1, mavrosStateCallback );
  ros::Subscriber mavrc_sub = nh.subscribe
                            ( "/mavros/rc/in", 1, rcdataCallback );

  map_lock = nh.serviceClient<std_srvs::SetBool>("/lock_arming_mapframe");
  bias_comp = nh.serviceClient<std_srvs::SetBool>( "/set_bias_compensation");
  
  ros::MultiThreadedSpinner(2).spin();
  return 0;
}
