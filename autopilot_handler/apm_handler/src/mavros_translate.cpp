/* This takes the output of the controller node, which is usually target angles (rad)
  and target thrust (Newtons), and maps them into values that APM understands.
  The translation is specific to vehicles but not to frametypes (the controller makes
  no distinction between hex- or quad- or deca- copters).
  Note that APM/Pixhawk makes things easier by means of mavros that does that actual
  communication part. We only need to do the scaling here.
  
  -- aj / Nov 10, 2018.
*/

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/rc_in.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/quaternion.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "freyja_msgs/msg/ctrl_command.hpp"

#define PITCH_MAX 0.785398
#define PITCH_MIN -0.785398
#define ROLL_MAX 0.785398
#define ROLL_MIN -0.785398
#define YAW_MAX 0.785398
#define YAW_MIN -0.785398

#define ROS_NODE_NAME "mavros_translator"

double THRUST_MAX = 1.0;
double THRUST_MIN = -1.0; //0.02;
double THRUST_SCALER = 200.0;

typedef mavros_msgs::msg::AttitudeTarget AttiTarget;
typedef freyja_msgs::msg::CtrlCommand    CtrlCommand;
typedef mavros_msgs::msg::State          MavState;
typedef mavros_msgs::msg::RCIn           RCInput;
typedef std_srvs::srv::SetBool		 BoolServ;
using std::placeholders::_1;
using std::placeholders::_2;

uint8_t ignore_rates = AttiTarget::IGNORE_ROLL_RATE |
                       AttiTarget::IGNORE_PITCH_RATE; // |
                        //AttiTarget::IGNORE_YAW_RATE;


class MavrosHandler : public rclcpp::Node
{
  bool vehicle_armed_;
  bool in_computer_mode_;

  public:
    MavrosHandler();

    rclcpp::Publisher<AttiTarget>::SharedPtr atti_pub_;

    rclcpp::Subscription<CtrlCommand>::SharedPtr ctrlCmd_sub_;
    void rpytCommandCallback( const CtrlCommand::ConstSharedPtr );

    rclcpp::Subscription<MavState>::SharedPtr mavState_sub_;
    void mavrosStateCallback( const MavState::ConstSharedPtr );

    rclcpp::Subscription<RCInput>::SharedPtr rcInput_sub_;
    void mavrosRCCallback( const RCInput::ConstSharedPtr );

    rclcpp::Client<BoolServ>::SharedPtr map_lock_;
    rclcpp::Client<BoolServ>::SharedPtr bias_comp_;

    void sendToMavros( const double&, const double&, const double&, const double& );
    inline void anglesToDouble( double &tgt_r, double &tgt_p, double &tgt_y )
    {
      tgt_r = tgt_r / ROLL_MAX;
      tgt_p = tgt_p / PITCH_MAX;
      tgt_y = tgt_y / YAW_MAX;    
    }

    inline void thrustToDouble( double &t )
    {
      t /= THRUST_SCALER;
    }
};

MavrosHandler::MavrosHandler() :  Node( ROS_NODE_NAME )
{
  auto qos = rclcpp::QoS( rclcpp::KeepLast(1) ).best_effort().durability_volatile();
  vehicle_armed_ = false;
  in_computer_mode_ = false;
  declare_parameter<double>( "thrust_scaler", 200.0 );
  get_parameter( "thrust_scaler", THRUST_SCALER );

  ctrlCmd_sub_ = create_subscription <CtrlCommand> ( "rpyt_command", 1,
                          std::bind( &MavrosHandler::rpytCommandCallback, this, _1 ) );
  mavState_sub_ = create_subscription <MavState> ( "mavros/state", qos,
                          std::bind( &MavrosHandler::mavrosStateCallback, this, _1 ) );
  rcInput_sub_ = create_subscription <RCInput> ( "mavros/rc/in", 1, 
                          std::bind( &MavrosHandler::mavrosRCCallback, this, _1 ) );
  
  map_lock_ = create_client <BoolServ> ("lock_arming_mapframe" );
  bias_comp_ = create_client <BoolServ> ( "set_auto_biascomp" );

  atti_pub_ = create_publisher <AttiTarget> ( "mavros/setpoint_raw/attitude", 1 );
}

void MavrosHandler::rpytCommandCallback( const CtrlCommand::ConstSharedPtr msg )
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

void MavrosHandler::sendToMavros( const double &p, const double &r, const double &y, const double &t )
{
  tf2::Quaternion tgt_q;
  tgt_q.setRPY( r, p, y );
  
  static AttiTarget atti_tgt;
  atti_tgt.type_mask = ignore_rates;
  atti_tgt.orientation = tf2::toMsg( tgt_q );
  atti_tgt.thrust = t;
  atti_tgt.body_rate.z = -y;
  atti_tgt.header.stamp = now();
    
  atti_pub_ -> publish( atti_tgt );
}


void MavrosHandler::mavrosStateCallback( const MavState::ConstSharedPtr msg )
{
  /* call map lock/unlock service when arming/disarming */
  RCLCPP_INFO( get_logger(), "state cb" );

  auto lockreq = std::make_shared<BoolServ::Request> ();
  if( msg->armed == true && !vehicle_armed_ )
  {
    vehicle_armed_ = true;
    lockreq -> data = true;
    map_lock_ -> async_send_request( lockreq );
    RCLCPP_INFO( get_logger(), "Armed" );
  }
  else if( msg->armed == false && vehicle_armed_ )
  {
    vehicle_armed_ = false;
    lockreq -> data = false;
    map_lock_ -> async_send_request( lockreq );
  }
  
  /* call bias compensation service when switching in/out of computer */
  BoolServ::Request::SharedPtr biasreq;
  if( msg->mode == "CMODE(25)" && !in_computer_mode_ )
  {
    in_computer_mode_ = true;
    biasreq -> data = true;
    bias_comp_ -> async_send_request( biasreq );
  }
  else if( msg->mode != "CMODE(25)" && in_computer_mode_ )
  {
    in_computer_mode_ = false;
    biasreq -> data = false;
    bias_comp_ -> async_send_request( biasreq );
  }
}


void MavrosHandler::mavrosRCCallback( const RCInput::ConstSharedPtr msg )
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
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<MavrosHandler>() );
  rclcpp::shutdown();

  return 0;
}
