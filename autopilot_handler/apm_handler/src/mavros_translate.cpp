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
#include "mavros_msgs/msg/rtk_baseline.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/quaternion.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "freyja_msgs/msg/ctrl_command.hpp"
#include "freyja_msgs/msg/freyja_interface_status.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Polynomials>

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

typedef mavros_msgs::msg::AttitudeTarget  AttiTarget;
typedef mavros_msgs::msg::State           MavState;
typedef mavros_msgs::msg::RCIn            RCInput;
typedef mavros_msgs::msg::RTKBaseline     RTKBaseline;

typedef sensor_msgs::msg::BatteryState    Battery;
typedef std_srvs::srv::SetBool		        BoolServ;
typedef geometry_msgs::msg::Vector3       GeomVec3;

typedef freyja_msgs::msg::FreyjaInterfaceStatus   FreyjaIfaceStatus;
typedef freyja_msgs::msg::CtrlCommand             CtrlCommand;

typedef Eigen::Matrix<double,1,3>         RowVector3d;

using std::placeholders::_1;
using std::placeholders::_2;

uint8_t ignore_rates = AttiTarget::IGNORE_ROLL_RATE |
                       AttiTarget::IGNORE_PITCH_RATE; // |
                        //AttiTarget::IGNORE_YAW_RATE;


class ThrustCalibration
{
  RowVector3d A_, B_;
  double Const_;
  std::function<RowVector3d(double)> Bprime_;
  std::function<double(double)> volt_normed_;
  std::function<double(double)> throttle_normed_;
  std::function<double(double)> throttle_unnormed_;
  std::function<double(double,double)> getExpectedThrust_;

  Eigen::PolynomialSolver<double, 3> rootSolver_;

  double THRUST_TOL;
  double batt_volt_normed_;
  double calib_throttle_;       // final output, in range [0..1]
  public:
    /*
      v_normed = @(v) (v - 1486)/113.3;
      t_normed = @(t) (t - 0.14)/0.1207;
      t_unnormed = @(t) t*0.1207 + 0.14;
      getExpectedThrust = @(v,t) A*[v;v.^2;v.^3] + Bp(v)*[t;t.^2;t.^3] + 224.2 ;
      Bp = @(v) B + [18.109*v+0.5573*v.^2, -1.207, 0];
    */
    ThrustCalibration()
    { 
      THRUST_TOL = 20.0;       // allowed error in reprojected thrust
      A_ << 18.0248,  0.374, 4.081;
      B_ << 141.8474, -5.12, -0.7644;
      Const_ = 224.1938;
      volt_normed_ = [](const double &v){ return (v-1486)/113.3; };
      throttle_normed_ = [](const double &t){ return (t-0.14)/0.1207; };
      throttle_unnormed_ = [](const double &t){ return t*0.1207 + 0.14; };
      
      Bprime_ = [this](const double &v)
                { return B_ + Eigen::Matrix<double,1,3>(18.109*v +0.5573*v*v, -1.207,0); };
      getExpectedThrust_ = [this](const double &v, const double &t)
                          { 
                            double thr1 = A_*Eigen::Vector3d(v, v*v, v*v*v);
                            double thr2 = Bprime_(v)*Eigen::Vector3d(t, t*t, t*t*t);
                            return thr1 + thr2 + Const_;
                          };
    }

    double calibThrustFromNewtons( const double &req_thrust )
    {
      static Eigen::Matrix<double,4,1> coeffs;
      
      const double &v = batt_volt_normed_;
      double C = req_thrust - Const_ - A_*Eigen::Matrix<double,3,1>(v, v*v, v*v*v);
      coeffs << -C, Bprime_(v).transpose();

      // smallest root of this polyn is the required throttle (pre-normalised)
      bool root_found = false;
      rootSolver_.compute( coeffs );
      double thr = rootSolver_.absSmallestRealRoot(root_found, 0.1);
      // do sanity check: what thrust do we expect to get from this throttle?
      double exp_thrust = getExpectedThrust_( v, thr );

      if( root_found && std::fabs( exp_thrust - req_thrust ) < THRUST_TOL )
      {
        // undo normalisation
        calib_throttle_ = 4.0*throttle_unnormed_( thr );
      }
      else
      {
        // our solution is not acceptable, stick to something basic
        calib_throttle_ = req_thrust/THRUST_SCALER;
      }
      return calib_throttle_;
    }

    void setBatteryVoltage100( const double &v )
    { batt_volt_normed_ = volt_normed_(v); }
};


class MavrosHandler : public rclcpp::Node
{
  bool vehicle_armed_;
  bool in_computer_mode_;
  std::string computer_mode_name_;

  FreyjaIfaceStatus fstatus_;

  bool use_thrust_calib_;
  ThrustCalibration thrust_calib_;

  GeomVec3 baseline_msg_;

  public:
    MavrosHandler();

    rclcpp::Publisher<AttiTarget>::SharedPtr atti_pub_;

    rclcpp::Publisher<FreyjaIfaceStatus>::SharedPtr fstatus_pub_;
    void status_publisher();
    rclcpp::TimerBase::SharedPtr status_timer_;

    rclcpp::Subscription<CtrlCommand>::SharedPtr ctrlCmd_sub_;
    void rpytCommandCallback( const CtrlCommand::ConstSharedPtr );

    rclcpp::Subscription<MavState>::SharedPtr mavState_sub_;
    void mavrosStateCallback( const MavState::ConstSharedPtr );

    rclcpp::Subscription<RCInput>::SharedPtr rcInput_sub_;
    void mavrosRCCallback( const RCInput::ConstSharedPtr );

    rclcpp::Subscription<RTKBaseline>::SharedPtr rtk_sub_;
    rclcpp::Publisher<GeomVec3>::SharedPtr baseline_pub_;

    rclcpp::Subscription<Battery>::SharedPtr battery_sub_;

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
  computer_mode_name_ = "GUIDED_NOGPS";

  declare_parameter<double>( "thrust_scaler", 200.0 );
  declare_parameter<bool>( "use_thrust_calib", false );

  get_parameter( "thrust_scaler", THRUST_SCALER );
  get_parameter( "use_thrust_calib", use_thrust_calib_ );

  // declare publishers and clients before subscribers
  map_lock_ = create_client <BoolServ> ("lock_arming_mapframe" );
  bias_comp_ = create_client <BoolServ> ( "set_auto_biascomp" );

  atti_pub_ = create_publisher <AttiTarget> ( "mavros/setpoint_raw/attitude", 1 );
  fstatus_pub_ = create_publisher <FreyjaIfaceStatus> ( "freyja_interface_status", 1 );
  baseline_pub_ = create_publisher <GeomVec3> ( "ublox_f9p_rtkbaseline", 1 );

  // create subscribers
  ctrlCmd_sub_  = create_subscription <CtrlCommand> ( "rpyt_command", 1,
                          std::bind( &MavrosHandler::rpytCommandCallback, this, _1 ) );
  mavState_sub_ = create_subscription <MavState> ( "mavros/state", qos,
                          std::bind( &MavrosHandler::mavrosStateCallback, this, _1 ) );
  rcInput_sub_  = create_subscription <RCInput> ( "mavros/rc/in", 1, 
                          std::bind( &MavrosHandler::mavrosRCCallback, this, _1 ) );
  battery_sub_  = create_subscription <Battery> ( "mavros/battery", 1, 
                          [this](const Battery::ConstSharedPtr msg)
                          { thrust_calib_.setBatteryVoltage100( msg->voltage ); } );
  rtk_sub_      = create_subscription <RTKBaseline> ( "mavros/rtk_baseline", 1, 
                          [this](const RTKBaseline::ConstSharedPtr msg)
                          { 
                            baseline_msg_.x = msg->baseline_a_mm/1000.0;
                            baseline_msg_.y = msg->baseline_b_mm/1000.0;
                            baseline_msg_.z = msg->baseline_c_mm/1000.0;
                            baseline_pub_ -> publish( baseline_msg_ );
                          } );
  
  // now we can create fixed-rate timers and other processing code
  float status_pubrate = 5.0;     // Hz
  status_timer_ = rclcpp::create_timer( this, get_clock(),
                          std::chrono::duration<float>(1.0/status_pubrate),
                          std::bind(&MavrosHandler::status_publisher, this) );
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

  if( use_thrust_calib_ )
    tgt_thrust = thrust_calib_.calibThrustFromNewtons( tgt_thrust );
  else
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
  auto biasreq = std::make_shared<BoolServ::Request> ();
  if( msg->mode == computer_mode_name_ && !in_computer_mode_ )
  {
    in_computer_mode_ = true;
    biasreq -> data = true;
    bias_comp_ -> async_send_request( biasreq );
  }
  else if( msg->mode != computer_mode_name_ && in_computer_mode_ )
  {
    in_computer_mode_ = false;
    biasreq -> data = false;
    bias_comp_ -> async_send_request( biasreq );
  }
    
  fstatus_.armed = vehicle_armed_;
  fstatus_.computer_ctrl = in_computer_mode_;
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
  static bool extf_state = false;
  if( (msg->channels).size() < 2 )
    return;

  // rc channels 0-3 are usually rpyt sticks;
  // ..  most transmitters will have at least 4 extra channels.
  fstatus_.aux1 = (msg->channels[4]) > 1500;
  fstatus_.aux2 = (msg->channels[5]) > 1500;
  fstatus_.aux3 = (msg->channels[6]) > 1500;
  fstatus_.aux4 = (msg->channels[7]) > 1500;
}

// This function is on a timer, called at fixed (pretty low) rate
void MavrosHandler::status_publisher()
{
  fstatus_pub_ -> publish( fstatus_ );
}

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<MavrosHandler>() );
  rclcpp::shutdown();

  return 0;
}
