/* Linear quadratic estimator (LQE) implementation.

  State propagation happens at a rate higher than the controller. External
  sources can trigger an update function at sporadic times.

  -- aj / Apr 18, 2019
*/
#include "state_estimator.h"

#define ROS_NODE_NAME "state_estimator_kf"
#define ZERO3x3 Eigen::MatrixXd::Zero(3,3)
#define IDEN3x3 Eigen::MatrixXd::Identity(3,3)
#define STPROP_SLEEP_US_ 14285

StateEstimator::StateEstimator() : nh_(), priv_nh_("~")
{
  // experimental:
  n_stprops_since_update_ = 0;
  // listen to camera
  state_sub_ = nh_.subscribe( "/current_state", 1,
                                &StateEstimator::stateUpdatesCallback, this );
  std::string output_topic;
  priv_nh_.param( "output_topic", output_topic, std::string("/current_state_estimate") );
  // for when all is said and done
  state_pub_ = nh_.advertise <freyja_msgs::CurrentStateBiasFree>
                  ( output_topic, 1, true );
 
  int estimator_rate_default = 70;
  priv_nh_.param( "estimator_rate", estimator_rate_, estimator_rate_default );
  float estimator_period = 1.0/estimator_rate_;
  n_stprops_allowed_ = 2.5;
  
  // constants for the lq-estimator
  initLqeSystem( );
 
  // listen to controller 
  ctrl_input_sub_ = nh_.subscribe( "/controller_debug", 1, 
                                &StateEstimator::controlInputCallback, this );

  /* ROS timers are pretty much unreliable for "high" rates:
    estimator_timer_ = nh_.createTimer( ros::Duration(estimator_period),
                                        &StateEstimator::state_propagation, this );
                                        
    Use thread libraries instead:
  */

  state_propagation_alive_ = true;
  last_prop_t_ = std::chrono::high_resolution_clock::now();
  state_prop_thread_ = std::thread( &StateEstimator::state_propagation, this );

}

StateEstimator::~StateEstimator()
{
  state_prop_thread_.join();
  std::cout << "StateEstimator: propagation thread stopped!" << std::endl;
}

void StateEstimator::initLqeSystem( )
{
  /* LQE model is a single/double integrator with downscaled ctrl inputs:
      x(k+1) = A.x(k) + B.u(k)
      z(k) = C.x(k) + Q
      
     Note that the identity terms in the B matrix are scaled down to `dt` so as
     to keep the influence of control input somewhat limited and also not strong
     enough to cause feedback oscillations (initiated by offsets). The reality
     is that I'm not smart enough to get it absolutely correct.
     Control input is a 3D vector of accelerations commanded by the control node.
     LQE_INTEGRATOR_ORDER is a compile-time flag from gcc. Check CMakeLists. This
     allows Eigen to statically optimize its matrices. *This* I am smart about :)
  */
  
  float dt = 1.0/estimator_rate_;
  float dt2 = dt*dt/2.0;

  #if LQE_INTEGRATOR_ORDER_ == 2
    sys_A_ << 1.0, 0.0, 0.0, dt, 0.0, 0.0, -dt2, 0.0, 0.0, 
              0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, -dt2, 0.0,
              0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, -dt2,
              0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -dt, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -dt, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -dt,
              Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3);
    sys_A_t_ = sys_A_.transpose();

    sys_B_ << 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0,
              Eigen::MatrixXd::Zero(3,3);

    proc_noise_Q_ << 0.1*IDEN3x3, ZERO3x3, ZERO3x3,
                     ZERO3x3, 0.25*IDEN3x3, ZERO3x3,
                     ZERO3x3, ZERO3x3, 1*IDEN3x3;
    meas_noise_R_ << 0.2*IDEN3x3, ZERO3x3,
                     ZERO3x3, 2.5*IDEN3x3;

    meas_matrix_C_ << IDEN3x3, ZERO3x3, ZERO3x3,
                      ZERO3x3, IDEN3x3, ZERO3x3;

    meas_matrix_C_t = meas_matrix_C_.transpose();
  
    state_cov_P_ = 1*Eigen::MatrixXd::Identity(9,9);
    
    // guess some initial state 
    best_estimate_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    prev_vel1_ << 0.0, 0.0, 0.0;
    prev_vel2_ << 0.0, 0.0, 0.0;

  #else
    sys_A_ << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 
              0.0, 1.0, 0.0, 0.0, dt, 0.0, 
              0.0, 0.0, 1.0, 0.0, 0.0, dt,
              0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    sys_A_t_ = sys_A_.transpose();

    sys_B_ << 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;

    proc_noise_Q_ << 0.1*IDEN3x3, ZERO3x3, 
                     ZERO3x3, 0.25*IDEN3x3;
    meas_noise_R_ << 0.2*IDEN3x3, ZERO3x3,
                     ZERO3x3, 2.5*IDEN3x3;

    meas_matrix_C_ << IDEN3x3, ZERO3x3,
                      ZERO3x3, IDEN3x3;
    meas_matrix_C_t = meas_matrix_C_.transpose();

    state_cov_P_ = 1*Eigen::MatrixXd::Identity(6,6);
    
    // guess some initial state 
    best_estimate_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    prev_vel1_ << 0.0, 0.0, 0.0;
    prev_vel2_ << 0.0, 0.0, 0.0;
  #endif
  // convenience
  I9x9 = Eigen::MatrixXd::Identity(9,9); 
  I6x6 = Eigen::MatrixXd::Identity(6,6);
  I3x3 = Eigen::MatrixXd::Identity(3,3); 
  timing_matrix_ = Eigen::MatrixXd::Zero(6,3);
  
}

__attribute__((optimize("unroll-loops")))
void StateEstimator::state_propagation( )
{
  while( ros::ok() )
  {
    // alternatives to prevent jitter: use cpp-std-thread methods, Orocos, rtt ..
    ts = std::chrono::high_resolution_clock::now();
    prop_interval_ = ts - last_prop_t_;
    double delta_t = prop_interval_.count();
    
    #if LQE_INTEGRATOR_ORDER_ == 2
      timing_matrix_ << 0.5*delta_t*delta_t * I3x3,
                            delta_t * I3x3;
      sys_A_.block<3,3>(0, 3) = delta_t * I3x3;
      sys_A_.topRightCorner<6,3>() = timing_matrix_;
      sys_B_.block<6,3>(0, 0) =  timing_matrix_;
      
      sys_A_t_ = sys_A_.transpose();
    #else
      sys_A_.block(0, 3, 3, 3) = delta_t * I3x3;
      sys_B_.block(3, 0, 3, 3) = delta_t * I3x3;
      sys_A_t_.block(3, 0, 3, 3) = delta_t * I3x3;
    #endif

    std::unique_lock<std::mutex> spmtx( state_prop_mutex_, std::defer_lock );
    if( spmtx.try_lock() )
    {
      // prevent too much artifical feedback
      n_stprops_since_update_++;
      double input_shaping_ratio = ((double)n_stprops_since_update_)/n_stprops_allowed_;
      if( n_stprops_since_update_ > 2 )
        ctrl_input_u_ << 0.0, 0.0, 0.0;

      // propagate state forward
      best_estimate_ = sys_A_ * best_estimate_ +  sys_B_ * ctrl_input_u_ * input_shaping_ratio;
      state_cov_P_ = sys_A_*state_cov_P_*sys_A_t_ + proc_noise_Q_;

      //last_prop_t_ = std::chrono::high_resolution_clock::now();

      // this will be unrolled by gcc
      for( int idx=0; idx < nStates; idx++ )
        state_msg_.state_vector[idx] = best_estimate_(idx);
      spmtx.unlock();

      // publish
      state_msg_.header.stamp = ros::Time::now();

      last_prop_t_ = te = std::chrono::high_resolution_clock::now();
      int dt = std::chrono::duration_cast<std::chrono::microseconds>(te-ts).count();
      state_msg_.state_vector[9] = n_stprops_since_update_;
      state_msg_.state_vector[10] = delta_t;
      state_msg_.state_vector[11] = dt;
      state_pub_.publish( state_msg_ );

      std::this_thread::sleep_for( std::chrono::microseconds(STPROP_SLEEP_US_ - dt) );
    }
    else
      std::this_thread::sleep_for( std::chrono::microseconds(STPROP_SLEEP_US_) );
  }
}

void StateEstimator::state_updation()
{
  /* State update equations:
        G = P_k * C' * (C*P_k*C' + R)'
        x = x + G*(Z - Cx)
        P = (I-GC)*P
     If C = I, speed-up/de-clutter the code here.
  */
  Eigen::MatrixXd innov = ( meas_matrix_C_ * state_cov_P_ * meas_matrix_C_t + meas_noise_R_ ).inverse();
  state_prop_mutex_.lock();
    kalman_gain_G_ = state_cov_P_ * meas_matrix_C_t * innov;
    best_estimate_ = best_estimate_ + kalman_gain_G_*(measurement_z_ - meas_matrix_C_*best_estimate_);
    #if LQE_INTEGRATOR_ORDER_ == 2
      state_cov_P_ = (I9x9 - kalman_gain_G_*meas_matrix_C_)*state_cov_P_;
    #else
      state_cov_P_ = (I6x6 - kalman_gain_G_*meas_matrix_C_)*state_cov_P_;
    #endif
  state_prop_mutex_.unlock();
}

void StateEstimator::controlInputCallback( const LqrInputAccel & msg )
{
  /* Listening to debug from controller is neat because it contains the raw
     accelerations commanded, and also the feedback linearized attitudes that
     were computed. At the moment, we rely on accelerations, but in the future,
     one could incorporate attitude commands to do more non-linear stuff here.
  */

  // note: X = [pn, pe, pd ..]
  state_prop_mutex_.lock();
    ctrl_input_u_[0] = msg -> lqr_u[0];
    ctrl_input_u_[1] = msg -> lqr_u[1];
    ctrl_input_u_[2] = msg -> lqr_u[2] + 9.81;
    n_stprops_since_update_ = 0;
  state_prop_mutex_.unlock();
}

void StateEstimator::medianFilter()
{
  /*
  // three element sort
  measurement_z_(3) = std::max( std::min( prev_vel1_(0), prev_vel2_(0)),
                      std::min( std::max( prev_vel1_(0), prev_vel2_(0) ), measurement_z_(3) ) );
  measurement_z_(4) = std::max( std::min( prev_vel1_(1), prev_vel2_(1)),
                      std::min( std::max( prev_vel1_(1), prev_vel2_(1) ), measurement_z_(4) ) );
  measurement_z_(5) = std::max( std::min( prev_vel1_(2), prev_vel2_(2)),
                      std::min( std::max( prev_vel1_(2), prev_vel2_(2) ), measurement_z_(5) ) );
  */
}

void StateEstimator::stateUpdatesCallback( const freyja_msgs::CurrentState::ConstPtr & msg )
{

  // no need to mutexify this, since measurement_z_ is only used in update function
  const double *msgptr = msg->state_vector.data();
  std::vector<double> msv( msgptr, msgptr+6  );
  Eigen::Matrix<double, nMeas, 1> tmp(msv.data());
  measurement_z_ = tmp;

  // call state update, reset propagation counter

    state_updation();

  //n_stprops_since_update_ = 0;
}


int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  StateEstimator kfilter;
  
  ros::MultiThreadedSpinner cbspinner(3);
  cbspinner.spin();
  
  return 0;
}

