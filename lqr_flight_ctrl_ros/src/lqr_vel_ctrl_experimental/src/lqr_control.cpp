/* Implementation of the LQR-feedback loop.

   Note: at this time, the feedback gain K is computed from Matlab. If somebody
   wants to write a solution to the algebraic Riccati equations that compute the
   optimal feedback gain K, please feel absolutely encouraged to do so in the
   init function :)
   
   -- aj / 17th Nov, 2017.
*/

#include "lqr_control.h"

#define ROS_NODE_NAME "lqr_vel_ctrl"
LQRController::LQRController() : nh_("~")
{
  int controller_rate_default = 150;
  nh_.param( "controller_rate", controller_rate_, controller_rate_default );
  
  float mass_default = 0.55;
  nh_.param( "total_mass", total_mass_, mass_default );
  
  /* initialise system params, matrices and controller configuration */
  initLqrSystem();
  
  /* Associate a subscriber for the current vehicle state */
  state_sub_ = nh_.subscribe( "/current_state", 1,
                              &LQRController::stateCallback, this );
  /* Associate a subscriber for the current reference state */
  reference_sub_ = nh_.subscribe( "/reference_state", 1, 
                              &LQRController::trajectoryReferenceCallback, this );
                              
  /* Announce publisher for controller output */
  atti_cmd_pub_ = nh_.advertise <lqr_control::CtrlCommand>
                    ( "/rpyt_command", 1, true );
  controller_debug_pub_ = nh_.advertise <lqr_control::ControllerDebug>
                    ( "controller_debug", 1, true );
                    
  /* Timer to run the LQR controller perdiodically */
  float controller_period = 1.0/controller_rate_;
  controller_timer_ = nh_.createTimer( ros::Duration(controller_period),
                                      &LQRController::computeFeedback, this );
  have_state_update_ = false;
  have_reference_update_ = false;                                   
}

void LQRController::initLqrSystem()
{
  /* Implementation in Matlab, the values in lqr_K_ are copy-pasta from there.
    sys_A_ = [..];  (4x4)
    sys_B_ = [..];  (4x4)
  
    lqr_Q_ = [..];  (4x4)
    lqr_R_ = [..];  (4x4)
  */
  lqr_K_ << 3.1623, 0.0, 0.0, 0.0,
            0.0, 3.1623, 0.0, 0.00,
            0.0, 0.0, 14.1421, 0.0,
            0.0, 0.0, 0.0, 1.0;
}

void LQRController::stateCallback( const state_manager::CurrentState::ConstPtr &msg )
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
  reduced_state_ << temp.segment<3>(3) , double(yaw);
  have_state_update_ = true;
  
  std::unique_lock<std::mutex> rsmtx( reference_state_mutex_, std::defer_lock );
  if( have_reference_update_ && rsmtx.try_lock() )
    reduced_state_ -= reference_state_.tail<4>();
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
    
  /* Compute control inputs (accelerations, in this case) */
  Eigen::Matrix<double, 4, 1> control_input = -1 * lqr_K_ * reduced_state_;
  
  /* Force saturation on downward acceleration */
  control_input(2) = std::min( control_input(2), 8.0 );
  control_input(2) -= 9.81;
  
  /* Thrust */
  double T = total_mass_ * control_input.head<3>().norm();
  
  /* Roll, pitch and yaw */
  Eigen::Matrix<double, 3, 1> Z = rot_yaw_ * control_input.head<3>() * (-total_mass_/T);
  float roll = std::asin( -Z(1) );
  float pitch = std::atan( Z(0)/Z(2) );
  float yaw = control_input(3);
  
  /* Debug information */
  lqr_control::ControllerDebug debug_msg;
  debug_msg.header.stamp = ros::Time::now();
  debug_msg.lqr_u[0] = control_input(0);
  debug_msg.lqr_u[1] = control_input(1);
  debug_msg.lqr_u[2] = control_input(2);
  debug_msg.lqr_u[3] = control_input(3);
  debug_msg.thrust = T;
  debug_msg.roll = roll;
  debug_msg.pitch = pitch;
  debug_msg.yaw = yaw;
  controller_debug_pub_.publish( debug_msg );
  
  /* Actual commanded input */
  lqr_control::CtrlCommand ctrl_cmd;
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
