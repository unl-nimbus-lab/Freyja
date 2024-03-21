/* Implementation of the LQR-feedback loop.

   Note: at this time, the feedback gain K is computed from Matlab. If somebody
   wants to write a solution to the algebraic Riccati equations that compute the
   optimal feedback gain K, please feel absolutely encouraged to do so in the
   init function :)
   
   -- aj / 17th Nov, 2017.
*/

#include "lqr_ctrl_omni4wheel.h"

#define ROS_NODE_NAME "lqg_control"
#define pi 3.1416

LQRController::LQRController(BiasEstimator &b) : Node( ROS_NODE_NAME ),
                                                 bias_est_( b )
{
  int controller_rate_default = 30;
  
  declare_parameter<int>( "controller_rate", controller_rate_default );
  declare_parameter<bool>( "enable_flatness_ff", true );
  declare_parameter<std::string>( "bias_compensation", "auto" );
  declare_parameter<std::string>( "controller_type", "pos-vel" );
  declare_parameter<std::string>( "refstate_accels_mode", "ignore" );
  declare_parameter<double>( "accel_ff_factor", 0.25 );

  declare_parameter<double>( "chassis_length", 0.4 );
  declare_parameter<double>( "chassis_width", 0.3 );
  declare_parameter<double>( "wheel_radius", 0.10 );

  /* Checks for correctness */
  STATEFB_MISSING_INTRV_ = 0.5;
  have_state_update_ = false;
  have_reference_update_ = false;  
  
  /* Associate a subscriber for the current vehicle state */
  state_sub_ = create_subscription<CurrentState>( "current_state", 1,
                  std::bind(&LQRController::stateCallback, this, _1) );
  /* Associate a subscriber for the current reference state */
  reference_sub_ = create_subscription<TrajRef>( "reference_state", 1,
    std::bind(&LQRController::trajectoryReferenceCallback, this, _1) );
  
  /* Service provider for bias compensation */
  bias_enable_serv_ = create_service <BoolServ> ( "set_bias_compensation",
                  std::bind(&LQRController::biasEnableServer, this, _1, _2 ) );
  
  
  /* Announce publishers for controller output */
  wheel_cmd_pub_ = create_publisher <Wheel_Command>
                    ( "wheel_command", 1 );
  controller_debug_pub_ = create_publisher <CTRL_Debug>
                    ( "controller_debug", 1 );
  // @TODO: replace this with a slip-estimation module
  // est_mass_pub_ = create_publisher <std_msgs::msg::Float32>
  //                   ( "freyja_estimated_mass", 1 );

  

  get_parameter( "controller_rate", controller_rate_ );
  get_parameter( "accel_ff_factor", accel_ff_factor_ );
  
  /* Bias compensation parameters */
  std::string _bcomp = "auto";
  get_parameter( "bias_compensation", _bcomp );
  if( _bcomp == "always-on" )
    bias_compensation_req_ = true;                // always on (be careful!!)
  else if( _bcomp == "auto" || _bcomp == "always-off" )
    bias_compensation_req_ = false;               // off, or on by service call
  
  bias_compensation_off_ = (_bcomp == "always-off")? true : false;   // always off
  f_biases_ << 0.0, 0.0, 0.0;

  /* handle accel data in reference state */
  enable_accel_limiting_ = false;
  enable_accel_feedfwd_ = false;
  std::string _accel_mode;
  get_parameter("refstate_accels_mode", _accel_mode);
  if( _accel_mode == "limits" )             // treat values as scalar limits
    enable_accel_limiting_ = true;
  else if ( _accel_mode == "feedforward" )  // treat values as vector feedfwd elements
    enable_accel_feedfwd_ = true;
  else                                      // default: ignore accels
    RCLCPP_INFO( get_logger(), "LQG: ignoring accels in reference state." );


  /* initialise system, matrices and controller configuration */
  initLqrSystem();

  /* Timer to run the LQR controller perdiodically */
  float controller_period = 1.0/controller_rate_;
  controller_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<float>(controller_period),
                           std::bind( &LQRController::computeFeedback, this ) );

}

void LQRController::initLqrSystem()
{
  /* Implementation in Matlab, the values in lqr_K_ are copy-pasta from there.
    sys_A_ = [..];  (7x7)
    sys_B_ = [..];  (7x4)
    
    lqr_Q_ = [..];  (7x7) Q.diag = [1, 1, 8, 0.01, 0.01, 0.1, 1]
    lqr_R_ = [..];  (4x4) R.diag = [0.8, 0.8, 0.8, 1]
  */
  /* smooth and usually fine:
    lqr_Q_ = [..];  (7x7) Q.diag = [1, 1, 8, 0.01, 0.01, 0.1, 1]
    lqr_R_ = [..];  (4x4) R.diag = [0.8, 0.8, 0.8, 1]
  */
  get_parameter( "controller_type", controller_type_ );
  get_parameter( "enable_flatness_ff", enable_flatness_ff_ );
  if( controller_type_ == "pos-vel" )
  {
    // position, velocity (+ff), and yaw : order-2-ff
    lqr_K_ << 2.1472,   0,        0,        0.1,     0,        0,
              0,        2.1472,   0,        0,       0.1,      0,
              0,        0,        3.4162,   0,       0,        0.12;
  } 
  else if( controller_type_ == "vel-only" )
  {
    // velocity (+ff), and yaw : order-1-ff
    lqr_K_ << 0,       0,      0,        0.172,     0,        0,
              0,       0,      0,        0,         0.172,    0,
              0,       0,      3.4162,   0,         0,        0.18;
  }
  else if( controller_type_ == "open-loop" )
  {
    // ff only, and yaw (no regulation on position or velocities = risk!)
    lqr_K_ << 0,      0,      0,        0,     0,        0,
              0,      0,      0,        0,     0,        0,
              0,      0,      3.1162,   0,     0,        0.13;
    enable_flatness_ff_ = true;
  }

  RCLCPP_INFO( get_logger(), "Controller: %s, using flatness: %d", 
                              controller_type_.c_str(), int(enable_flatness_ff_) );


  double ch_l, ch_w;
  get_parameter( "wheel_radius", wheel_rad_ );
  get_parameter( "chassis_length", ch_l );
  get_parameter( "chassis_width", ch_w );
    
  double chassis_lw = ch_l + ch_w;  
  wheel_geometry_matrix_ << 1.0,   1.15,  chassis_lw,
                            1.0,  -1.15, -chassis_lw,
                            1.0,   1.15, -chassis_lw,
                            1.0,  -1.15,  chassis_lw;
}

void LQRController::biasEnableServer( const BoolServ::Request::SharedPtr rq,
                                      const BoolServ::Response::SharedPtr rp )
{
  if( bias_compensation_off_ )
  {
    RCLCPP_WARN( get_logger(), "LQR: Bias compensation remains off throughout." );
    rp -> success = false;       // service unsuccessful
    return;
  }
  
  bias_compensation_req_ = rq -> data;
  if( bias_compensation_req_ )
  {
    RCLCPP_WARN( get_logger(), "LQR: Bias compensation active!" );
    bias_est_.enable();
  }
  else
  {
    RCLCPP_WARN( get_logger(), "LQR: Bias compensation inactive!" );
    bias_est_.disable();
  }
  rp -> success = true;  // service successful
}

constexpr double LQRController::calcYawError( const double &a, const double &b )
{
  double yd = fmod( a - b + pi, 2*pi );
  yd = yd < 0 ? yd+2*pi : yd;
  return yd-pi; 
}

void LQRController::stateCallback( const CurrentState::ConstSharedPtr msg )
{
  /* Parse message to obtain state and reduced state information */
  static Eigen::Matrix<double, 6, 1> current_state;
  static Eigen::Matrix<double, 6, 1> state_err;

  float yaw = msg->state_vector[8];
  rot_yaw_ << std::cos(yaw), std::sin(yaw), 0,
             -std::sin(yaw), std::cos(yaw), 0,
              0, 0, 1;
             
  /* state  */
  current_state << msg->state_vector[0], msg->state_vector[1], double(yaw),
                   msg->state_vector[3], msg->state_vector[4], msg->state_vector[11];

  /* set measurement for bias estimator */
  if( bias_compensation_req_ )
    bias_est_.setMeasurement( current_state.head<6>() );
  
  /* compute x - xr right here */
  std::unique_lock<std::mutex> rsmtx( reference_state_mutex_, std::defer_lock );
  if( have_reference_update_ )
  {
    rsmtx.lock();
    state_err.head<6>() = current_state.head<6>() - reference_state_.head<6>();
    /* yaw-error is done differently */
    state_err(2) = calcYawError( current_state(2), reference_state_(2) );
    rsmtx.unlock();
    reduced_state_ = std::move( state_err );
    have_state_update_ = true;
  }
  
  last_state_update_t_ = now();
}

void LQRController::trajectoryReferenceCallback( const TrajRef::ConstSharedPtr msg )
{
  /* Simply copy over the reference state. Note that this might happen quite
    frequently (I expect a well-written trajectory provider to be using a nearly
    "continous" time update to the reference state). Any optimizations here are
    greatly welcome.
    Not currently using accelerations for FF.
  */
  reference_state_mutex_.lock();
  reference_state_ << msg->pn, msg->pe, msg->yaw, msg->vn, msg->ve, 0.0;
  reference_state_mutex_.unlock();
  
  reference_ff_ << msg->vn, msg->ve, 0.0;    // feed-forward
  have_reference_update_ = true;

  accel_refs_ << msg->an, msg->ae, 0.0;    // clip vel-out by these 
}



__attribute__((optimize("unroll-loops")))
void LQRController::computeFeedback( )
{
  /* Wait for atleast one update, or architect the code better */
  if( !have_state_update_ )
    return;
  
  static Eigen::Matrix<double, 4, 1> wheel_speeds;
  static Eigen::Matrix<double, 3, 1> control_input, prev_control_input;
  static Eigen::Matrix<double, 6, 1> state_err;
  
  bool state_valid = true;
  auto tnow = now();
  
  /* Check the difference from last state update time */
  float state_age = (tnow - last_state_update_t_).seconds();
  if( state_age > STATEFB_MISSING_INTRV_ )
  {
    /* State feedback missing for more than specified interval. Stop motors */
    control_input.setZero();
    wheel_speeds.setZero();
    state_valid = false;
  }
  else
  {
    /* Compute control inputs (accelerations, in this case) */
    state_err = std::move( reduced_state_ );
    control_input = -1 * lqr_K_ * state_err 
                    + static_cast<double>(enable_flatness_ff_) * reference_ff_
                    + static_cast<double>(enable_accel_feedfwd_) * (accel_refs_ * accel_ff_factor_);
  
    /* Force saturation on acceleration */
    if( enable_accel_limiting_ )
      constrainAccel( prev_control_input, control_input );
    prev_control_input = control_input;
    
    /* See if bias compensation was requested .. */
    if( bias_compensation_req_ )
    {
      bias_est_.getEstimatedBiases( f_biases_ );
      control_input.head<3>() -= f_biases_.head<3>();
    }
  
    /* Decompose accelerations to wheel speeds */
    wheel_speeds = (1.0/wheel_rad_) * wheel_geometry_matrix_ * rot_yaw_ * control_input;
  }
  
  /* Actual commanded input */
  Wheel_Command ctrl_cmd;
  ctrl_cmd.w1 = wheel_speeds(0);
  ctrl_cmd.w2 = wheel_speeds(1);
  ctrl_cmd.w3 = wheel_speeds(2);
  ctrl_cmd.w4 = wheel_speeds(3);
  ctrl_cmd.ctrl_mode = 0b00001111;
  wheel_cmd_pub_ -> publish( ctrl_cmd );
  

  /* Tell bias estimator about new control input */
  static Eigen::Matrix<double,4,1> bias_ctrl_input;
  bias_ctrl_input << control_input, 0.0;
  if( bias_compensation_req_ )
    bias_est_.setControlInput( bias_ctrl_input );
    
  /* Debug information */
  static CTRL_Debug debug_msg;
  debug_msg.header.stamp = tnow;
  for( uint8_t idx=0; idx<3; idx++ )
    debug_msg.lqr_u[idx] = static_cast<float>(control_input(idx));
  for( uint8_t idx=0; idx<3; idx++ )
    debug_msg.biasv[idx] = static_cast<float>(prev_control_input(idx));
  for( uint8_t idx=0; idx<6; idx++ )
    debug_msg.errv[idx] = static_cast<float>(state_err(idx));

  debug_msg.flags = (debug_msg.BIAS_EN * bias_compensation_req_) |
                    (debug_msg.MASS_CR * false ) |                  // not applicable here
                    (debug_msg.FLAT_FF * enable_flatness_ff_ ) |
                    (debug_msg.CTRL_OK * state_valid );
  controller_debug_pub_ -> publish( debug_msg ); 
}

void LQRController::constrainAccel( const Eigen::Matrix<double,3,1> &prev, 
                                    Eigen::Matrix<double,3,1> &cur )
{
  Eigen::Vector3d req_accel = (cur-prev)*controller_rate_;                        // from controller
  accel_refs_ = (accel_refs_.array().abs() < 1e-2).select(1000.0, accel_refs_);   // if near-zero, set to "inf"
  req_accel = req_accel.cwiseMin( accel_refs_ ).cwiseMax( -accel_refs_ );         // limit to range
  cur = prev + req_accel*(1.0/controller_rate_);                                  // shaped
}

int main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  BiasEstimator estimator;
  rclcpp::spin( std::make_shared<LQRController>(estimator) );
  rclcpp::shutdown();
  return 0;
}
