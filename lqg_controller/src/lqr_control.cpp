/* Implementation of the LQR-feedback loop.

   Note: at this time, the feedback gain K is computed from Matlab. If somebody
   wants to write a solution to the algebraic Riccati equations that compute the
   optimal feedback gain K, please feel absolutely encouraged to do so in the
   init function :)
   
   -- aj / 17th Nov, 2017.
*/

#include "lqr_control.h"

#define ROS_NODE_NAME "lqr_control"
#define pi 3.1416

LQRController::LQRController() : nh_(), priv_nh_("~")
{
  int controller_rate_default = 30;
  priv_nh_.param( "controller_rate", controller_rate_, controller_rate_default );
  
  float mass_default = 0.85;
  priv_nh_.param( "total_mass", total_mass_, mass_default );
  
  /* initialise system params, matrices and controller configuration */
  initLqrSystem();
  
  /* Associate a subscriber for the current vehicle state */
  state_sub_ = nh_.subscribe( "/current_state", 1,
                              &LQRController::stateCallback, this );
  /* Associate a subscriber for the current reference state */
  reference_sub_ = nh_.subscribe( "/reference_state", 1, 
                              &LQRController::trajectoryReferenceCallback, this );
                              
  /* Announce publisher for controller output */
  atti_cmd_pub_ = nh_.advertise <freyja_msgs::CtrlCommand>
                    ( "/rpyt_command", 1, true );
  controller_debug_pub_ = nh_.advertise <freyja_msgs::ControllerDebug>
                    ( "controller_debug", 1, true );
                    
  /* Timer to run the LQR controller perdiodically */
  float controller_period = 1.0/controller_rate_;
  controller_timer_ = nh_.createTimer( ros::Duration(controller_period),
                                      &LQRController::computeFeedback, this );
  
  /* Checks for correctness */
  STATEFB_MISSING_INTRV_ = 0.5;
  have_state_update_ = false;
  have_reference_update_ = false;  
}

void LQRController::initLqrSystem()
{
  /* Implementation in Matlab, the values in lqr_K_ are copy-pasta from there.
    sys_A_ = [..];  (7x7)
    sys_B_ = [..];  (7x4)
  
    lqr_Q_ = [..];  (7x7)
    lqr_R_ = [..];  (4x4)
  */
  lqr_K_ << 1.118, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
            0.0, 1.118, 0.0, 0.00, 1.4995, 0.0, 0.0,
            0.0, 0.0, 3.1623, 0.0, 0.0, 2.5347, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
}

double LQRController::calcYawError( const double &a, const double &b )
{
  double yd = fmod( a - b + pi, 2*pi );
  yd = yd < 0 ? yd+=2*pi : yd;
  return yd-pi; 
}

void LQRController::stateCallback( const freyja_msgs::CurrentState::ConstPtr &msg )
{
  /* Parse message to obtain state and reduced state information */
  //std::vector<double> sv(13);
  //for( int i=0; i<13; i++ )
  //  sv[i] = msg->state_vector[i];
  const double *msgptr = msg -> state_vector.data();
  std::vector<double> sv( msgptr, msgptr+13  );

  float yaw = sv[8];
  rot_yaw_ << std::cos(yaw), std::sin(yaw), 0,
            -std::sin(yaw), std::cos(yaw), 0,
             0, 0, 1;
             
  /* reduced state is the first 6 elements, and yaw */
  Eigen::Matrix<double, 13,1 >temp( sv.data() );
  reduced_state_ << temp.head<6>() , double(yaw);
  
  /* compute x - xr right here */
  std::unique_lock<std::mutex> rsmtx( reference_state_mutex_, std::defer_lock );
  if( have_reference_update_ && rsmtx.try_lock() )
  {
    reduced_state_.head<6>() -= reference_state_.head<6>();
    /* yaw-error is done differently */
    reduced_state_(6) = calcYawError( reduced_state_(6), reference_state_(6) );
  }
  
  have_state_update_ = true;
  last_state_update_t_ = ros::Time::now();
}

void LQRController::trajectoryReferenceCallback( const TrajRef::ConstPtr &msg )
{
  /* Simply copy over the reference state. Note that this might happen quite
    frequently (I expect a well-written trajectory provider to be using a nearly
    "continous" time update to the reference state). Any optimizations here are
    greatly welcome.
  */
  reference_state_mutex_.lock();
  reference_state_ << msg->pn, msg->pe, msg->pd, msg->vn, msg->ve, msg->vd, msg->yaw;
  reference_state_mutex_.unlock();
  
  have_reference_update_ = true;
}

void LQRController::computeFeedback( const ros::TimerEvent &event )
{
  /* Wait for atleast one update, or architect the code better */
  if( !have_state_update_ )
    return;
  
  float roll, pitch, yaw;
  double T;
  Eigen::Matrix<double, 4, 1> control_input;
  
  bool state_valid = true;
  
  /* Check the difference from last state update time */
  if( (ros::Time::now() - last_state_update_t_).toSec() > STATEFB_MISSING_INTRV_ )
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
    control_input = -1 * lqr_K_ * reduced_state_;
  
    /* Force saturation on downward acceleration */
    control_input(2) = std::min( control_input(2), 8.0 );
    control_input(2) -= 9.81;
  
    /* Thrust */
    T = total_mass_ * control_input.head<3>().norm();
  
    /* Roll, pitch and yawrate */
    Eigen::Matrix<double, 3, 1> Z = rot_yaw_ * control_input.head<3>() * (-total_mass_/T);
    roll = std::asin( -Z(1) );
    pitch = std::atan( Z(0)/Z(2) );
    yaw = control_input(3);
  }
  
  /* Debug information */
  freyja_msgs::ControllerDebug debug_msg;
  debug_msg.header.stamp = ros::Time::now();
  debug_msg.lqr_u[0] = control_input(0);
  debug_msg.lqr_u[1] = control_input(1);
  debug_msg.lqr_u[2] = control_input(2);
  debug_msg.lqr_u[3] = control_input(3);
  debug_msg.thrust = T;
  debug_msg.roll = roll;
  debug_msg.pitch = pitch;
  debug_msg.yaw = yaw;
  debug_msg.state_valid = state_valid;
  controller_debug_pub_.publish( debug_msg );
  
  /* Actual commanded input */
  freyja_msgs::CtrlCommand ctrl_cmd;
  ctrl_cmd.roll = roll;
  ctrl_cmd.pitch = pitch;
  ctrl_cmd.yaw = yaw;
  ctrl_cmd.thrust = T;
  ctrl_cmd.ctrl_mode = 0b00001111;
  atti_cmd_pub_.publish( ctrl_cmd );
}

int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  LQRController lqr;
  ros::spin();
  
  return 0;
}
