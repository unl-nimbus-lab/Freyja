/* Implementation of the LQR-feedback loop.

   Note: at this time, the feedback gain K is computed from Matlab. If somebody
   wants to write a solution to the algebraic Riccati equations that compute the
   optimal feedback gain K, please feel absolutely encouraged to do so in the
   init function :)
   
   -- aj / 17th Nov, 2017.
*/

#include "lqr_ctrl_flight.h"

#define ROS_NODE_NAME "lqg_control"
#define pi 3.1416

LQRController::LQRController(BiasEstimator &b) : Node( ROS_NODE_NAME ),
                                                 bias_est_( b )
{
  int controller_rate_default = 30;
  float mass_default = 0.85;
  
  declare_parameter<int>( "controller_rate", controller_rate_default );
  declare_parameter<float>( "total_mass", mass_default );
  declare_parameter<bool>( "use_stricter_gains", false );
  declare_parameter<bool>( "enable_flatness_ff", false );
  declare_parameter<bool>( "mass_correction", false );
  declare_parameter<bool>( "mass_estimation", true );
  declare_parameter<std::string>( "bias_compensation", "auto" );
  declare_parameter<bool>( "apply_est_biases", false );

  declare_parameter<std::string>( "controller_type", "pos-vel" );
  
  get_parameter( "controller_rate", controller_rate_ );
  get_parameter( "total_mass", total_mass_ );
  get_parameter( "use_stricter_gains", use_stricter_gains_ );
  
  
  /* initialise system, matrices and controller configuration */
  initLqrSystem();
  
  
  /* Associate a subscriber for the current vehicle state */
  state_sub_ = create_subscription<CurrentState>( "current_state", 1,
                  std::bind(&LQRController::stateCallback, this, _1) );
  /* Associate a subscriber for the current reference state */
  reference_sub_ = create_subscription<TrajRef>( "reference_state", 1,
                  std::bind(&LQRController::trajectoryReferenceCallback, this, _1) );
  /* Associate a subscriber for measured external forces */
  extf_sub_ = create_subscription<GeomVec3Stamped>( "pred_ext_forces", 1,
                  std::bind(&LQRController::externalForceCallback, this, _1) );
  
  /* Service provider for bias compensation */
  bias_enable_serv_ = create_service <BoolServ> ( "set_bias_compensation",
                  std::bind(&LQRController::biasEnableServer, this, _1, _2 ) );
  
  
  /* Announce publishers for controller output */
  atti_cmd_pub_ = create_publisher <RPYT_Command>
                    ( "rpyt_command", 1 );
  controller_debug_pub_ = create_publisher <CTRL_Debug>
                    ( "controller_debug", 1 );
  est_mass_pub_ = create_publisher <std_msgs::msg::Float32>
                    ( "freyja_estimated_mass", 1 );

  /* Timer to run the LQR controller perdiodically */
  float controller_period = 1.0/controller_rate_;
  controller_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<float>(controller_period),
                           std::bind( &LQRController::computeFeedback, this ) );
  
  /* Checks for correctness */
  STATEFB_MISSING_INTRV_ = 0.5;
  have_state_update_ = false;
  have_reference_update_ = false;  
  
  /* Bias compensation parameters */
  std::string _bcomp = "auto";
  get_parameter( "bias_compensation", _bcomp );
  if( _bcomp == "always-on" )
    bias_compensation_req_ = true;                // always on (be careful!!)
  else if( _bcomp == "auto" || _bcomp == "always-off" )
    bias_compensation_req_ = false;               // off, or on by service call
  
  bias_compensation_off_ = (_bcomp == "always-off")? true : false;   // always off
  
  f_biases_ << 0.0, 0.0, 0.0;
  get_parameter( "apply_est_biases", apply_bias_corr_ );
  
  /* Differential flatness feed-forward accelerations */
  get_parameter( "enable_flatness_ff", enable_flatness_ff_ );
  reference_ff_ << 0.0, 0.0, 0.0, 0.0;
  
  
  /* Mass estimation */
  enable_dyn_mass_correction_ = false;
  get_parameter( "mass_correction", enable_dyn_mass_correction_ );
  get_parameter( "mass_estimation", enable_dyn_mass_estimation_ );
  if( enable_dyn_mass_correction_ )
  {
    enable_dyn_mass_estimation_ = true;
    RCLCPP_WARN( get_logger(), "LQR: Mass correction active at init! This is discouraged." );
  }
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
  std::string controller_type;
  get_parameter( "controller_type", controller_type );

  if( controller_type == "pos-vel" )
  {
    lqr_K_ << 1.118, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
            0.0, 1.118, 0.0, 0.00, 1.4995, 0.0, 0.0,
            0.0, 0.0, 3.1623, 0.0, 0.0, 2.5347, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            
    /* little more aggresive:
      lqr_Q_ = [..];  (7x7) Q.diag = [1.2, 1.2, 8.2, 0.02, 0.02, 0.1, 1]
      lqr_R_ = [..];  (4x4) R.diag = [0.8, 0.8, 0.8, 1]
    */
    if( use_stricter_gains_ )
    {
      lqr_K_ << 1.225, 0.0, 0.0, 1.5731, 0.0, 0.0, 0.0,
                0.0, 1.225, 0.0, 0.00, 1.5731, 0.0, 0.0,
                0.0, 0.0, 3.2016, 0.0, 0.0, 2.550, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
      RCLCPP_WARN( get_logger(), "LQR: stricter gains requested!" );
    }
  }
  else if( controller_type == "vel-only" )
  {
    lqr_K_ << 0.0, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.00, 1.4995, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 2.5347, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    RCLCPP_WARN( get_logger(), "LQR: controlling only 3D velocities." );
  }
  else if( controller_type == "NEvel-Dpos" )
  {
    lqr_K_ << 0.0, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.00, 1.4995, 0.0, 0.0,
            0.0, 0.0, 3.1623, 0.0, 0.0, 2.5347, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    RCLCPP_WARN( get_logger(), "LQR: controlling 2D velocity and vertical position." );
  }
  else
    RCLCPP_ERROR( get_logger(), "LQR: Controller type unknown!" );
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
  static Eigen::Matrix<double, 7, 1> current_state;
  static Eigen::Matrix<double, 7, 1> state_err;

  float yaw = msg->state_vector[8];
  rot_yaw_ << std::cos(yaw), std::sin(yaw), 0,
             -std::sin(yaw), std::cos(yaw), 0,
              0, 0, 1;
             
  /* state is the first 6 elements, and yaw */
  current_state << Eigen::Map<const PosVelNED>( msg->state_vector.data() ),
                   double(yaw);

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
    state_err(6) = calcYawError( current_state(6), reference_state_(6) );
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
  reference_state_ << msg->pn, msg->pe, msg->pd, msg->vn, msg->ve, msg->vd, msg->yaw;
  reference_state_mutex_.unlock();
  
  reference_ff_ << msg->an, msg->ae, msg->ad, 0.0;    // only NED accelerations
  have_reference_update_ = true;
}

void LQRController::externalForceCallback( const GeomVec3Stamped::ConstSharedPtr msg )
{
  f_ext_ << msg->vector.x, msg->vector.y, msg->vector.z;
}

__attribute__((optimize("unroll-loops")))
void LQRController::computeFeedback( )
{
  /* Wait for atleast one update, or architect the code better */
  if( !have_state_update_ )
    return;
  
  float roll, pitch, yaw;
  double T;
  static Eigen::Matrix<double, 4, 1> control_input;
  static Eigen::Matrix<double, 7, 1> state_err;
  
  bool state_valid = true;
  auto tnow = now();
  
  /* Check the difference from last state update time */
  float state_age = (tnow - last_state_update_t_).seconds();
  if( state_age > STATEFB_MISSING_INTRV_ )
  {
    /* State feedback missing for more than specified interval. Try descend */
    roll = pitch = yaw = 0.0;
    T = (total_mass_ * 9.81) - 0.5;
    control_input << 0.0, 0.0, 0.0, 0.0;
    state_valid = false;
  }
  else
  {
    /* Compute control inputs (accelerations, in this case) */
    state_err = std::move( reduced_state_ );
    control_input = -1 * lqr_K_ * state_err 
                    + static_cast<double>(enable_flatness_ff_) * reference_ff_;
  
    /* Force saturation on downward acceleration */
    control_input(2) = std::min( control_input(2), 8.0 );
    control_input(2) -= 9.81;
    
    /* See if bias compensation was requested .. */
    if( bias_compensation_req_ )
    {
      bias_est_.getEstimatedBiases( f_biases_ );
      control_input.head<3>() -= (apply_bias_corr_ * f_biases_.head<3>() );
    }
  
    /* Thrust */
    T = total_mass_ * control_input.head<3>().norm();
  
    /* Roll, pitch and yawrate */
    Eigen::Matrix<double, 3, 1> Z = rot_yaw_ * control_input.head<3>() * (-total_mass_/T);
    roll = std::asin( -Z(1) );
    pitch = std::atan( Z(0)/Z(2) );
    yaw = control_input(3);
  }
  
  /* Actual commanded input */
  RPYT_Command ctrl_cmd;
  ctrl_cmd.roll = roll;
  ctrl_cmd.pitch = pitch;
  ctrl_cmd.yaw = yaw;
  ctrl_cmd.thrust = T;
  ctrl_cmd.ctrl_mode = 0b00001111;
  atti_cmd_pub_ -> publish( ctrl_cmd );
  

  /* Tell bias estimator about new control input */
  if( bias_compensation_req_ )
    bias_est_.setControlInput( control_input );
    
  /* Update the total flying mass if requested */
  if( enable_dyn_mass_estimation_ )
    estimateMass( control_input, tnow );
    
  /* Debug information */
  static CTRL_Debug debug_msg;
  debug_msg.header.stamp = tnow;
  for( uint8_t idx=0; idx<4; idx++ )
    debug_msg.lqr_u[idx] = static_cast<float>(control_input.coeff(idx));
  for( uint8_t idx=0; idx<3; idx++ )
    debug_msg.biasv[idx] = static_cast<float>(f_biases_.coeff(idx));
  for( uint8_t idx=0; idx<3; idx++ )
    debug_msg.ext_f[idx] = static_cast<float>(f_ext_.coeff(idx));
  for( uint8_t idx=0; idx<7; idx++ )
    debug_msg.errv[idx] = static_cast<float>(state_err.coeff(idx));

  debug_msg.flags = (debug_msg.BIAS_EN * bias_compensation_req_) |
                    (debug_msg.MASS_CR * enable_dyn_mass_correction_ ) |
                    (debug_msg.FLAT_FF * enable_flatness_ff_ ) |
                    (debug_msg.CTRL_OK * state_valid );
  controller_debug_pub_ -> publish( debug_msg ); 
}

void LQRController::estimateMass( const Vector4d &c, rclcpp::Time &t )
{
  /* Experimental module that estimates the true mass of the system in air.
    It uses the provided mass and observes the deviation from expected output
    of the controller - and attributes *all* of that error to an incorrect
    mass parameter. This is recommended only if mass may be off by ~200-300g.
    Errors larger than that can induce major second-order oscillations, and are
    usually better addressed elsewhere in the architecture (or get a better scale).
  */
  static float prev_estimated_mass = total_mass_;
  static std_msgs::msg::Float32 estmass;
  static rclcpp::Time t_last = now();

  if( (t-t_last).seconds() < 3.0 )
    return;

  float ctrl_effort = c(2) + 9.81;
  float estimated_mass = total_mass_*(9.81 - ctrl_effort)/9.81;
  // basic low-pass filter to prevent wild jitter
  estimated_mass = (prev_estimated_mass + estimated_mass)/2.0;
  
  estmass.data = estimated_mass;
  est_mass_pub_ -> publish( estmass );
  
  /* if correction is allowed- clip between min and max */
  if( enable_dyn_mass_correction_ )
    total_mass_ = std::min( 2.0f, std::max( 0.8f, estimated_mass ) );
  
  // book-keeping
  t_last = t;
  prev_estimated_mass = estimated_mass;
}

int main( int argc, char** argv )
{
  /*ros::init( argc, argv, ROS_NODE_NAME );
  BiasEstimator estimator;
  LQRController lqr( estimator );
  
  ros::MultiThreadedSpinner cbspinner(4);
  cbspinner.spin();
  */
  rclcpp::init( argc, argv );
  BiasEstimator estimator;
  rclcpp::spin( std::make_shared<LQRController>(estimator) );
  rclcpp::shutdown();
  return 0;
}
