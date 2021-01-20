/* Optimal full-state 3-axis bias estimator.
   State vector being observed is:
    [pn, pe, pd, vn, ve, vd, bn, be, bd]^T
    
   Closed-loop gains for the observer are adjusted by Kalman filter. One could
   also place the eigenvalues using matlab's place command and use this as
   a standard Luenberger observer.
   
  -- aj / 27th Sept, 2019.
*/

#include "bias_estimator.h"

#define ROS_NODE_NAME "state_estimator_kf"
#define ZERO3x3 Eigen::MatrixXd::Zero(3,3)
#define IDEN3x3 Eigen::MatrixXd::Identity(3,3)

BiasEstimator::BiasEstimator() : nh_(), priv_nh_("~")
{
  /* Used for debug and such */
  std::string output_topic;
  priv_nh_.param( "estimator_debug_topic", output_topic,
                  std::string("/bias_estimates") );
  bias_pub_ = nh_.advertise <freyja_msgs::CurrentStateBiasEst>
                              ( output_topic, 1, true );

  /* Parameters for thread */
  int estimator_rate_default = 10;
  priv_nh_.param( "estimator_rate", estimator_rate_, estimator_rate_default );
  estimator_period_us_ = std::round( (1.0/estimator_rate_)*1000*1000 );
  
  /* Time constant  factor */
  estimator_tc_ = 1.5;
  estimator_output_shaping_ = 0.0;
  
  /* init estimator matrices */
  initEstimatorSystem();
  
  /* ROS timers are pretty much unreliable for "high" rates. Don't use:
    estimator_timer_ = nh_.createTimer( ros::Duration(estimator_period),
                                        &BiasEstimator::state_propagation, this );
  
    Use thread libraries instead:
  */
  estimator_off_ = false;
  state_propagation_alive_ = true;
  n_stprops_since_update_ = 0;
  last_prop_t_ = std::chrono::high_resolution_clock::now();
  state_prop_thread_ = std::thread( &BiasEstimator::state_propagation, this );
}

BiasEstimator::~BiasEstimator()
{
  state_prop_thread_.join();
  state_propagation_alive_ = false;
  std::cout << "BiasEstimator: stopped!" << std::endl;
}

void BiasEstimator::initEstimatorSystem()
{
  float dt = 1.0/estimator_rate_;
  float dt2 = dt*dt/2.0;
  
  sys_A_ << 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2, 0.0, 0.0, 
            0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
            Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3);
  sys_A_t_ = sys_A_.transpose();

  sys_B_ << dt2, 0.0, 0.0,
            0.0, dt2, 0.0,
            0.0, 0.0, dt2,
             dt, 0.0, 0.0,
            0.0,  dt, 0.0,
            0.0, 0.0,  dt,
            Eigen::MatrixXd::Zero(3,3);

  /* Numbers pulled out of a magical hat only available to the deserving few */
  proc_noise_Q_ << 0.1*IDEN3x3, ZERO3x3, ZERO3x3,
                   ZERO3x3, 0.2*IDEN3x3, ZERO3x3,
                   ZERO3x3, ZERO3x3, 0.8*IDEN3x3;
  meas_noise_R_ << 0.2*IDEN3x3, ZERO3x3,
                   ZERO3x3, 0.3*IDEN3x3;

  meas_matrix_C_ << IDEN3x3, ZERO3x3, ZERO3x3,
                    ZERO3x3, IDEN3x3, ZERO3x3;

  meas_matrix_C_t = meas_matrix_C_.transpose();
  
  state_cov_P_ = 1*Eigen::MatrixXd::Identity(9,9);
    
  // guess some initial state 
  best_estimate_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ctrl_input_u_ << 0.0, 0.0, 0.0;
  
  // convenience
  I9x9 = Eigen::MatrixXd::Identity(9,9); 
  I6x6 = Eigen::MatrixXd::Identity(6,6);
  I3x3 = Eigen::MatrixXd::Identity(3,3); 
  timing_matrix_ = Eigen::MatrixXd::Zero(6,3);
}

__attribute__((optimize("unroll-loops")))
void BiasEstimator::state_propagation( )
{
  Eigen::Matrix<double, 3, 1> input_shaping;  // % of input to consider
  input_shaping << 0.95, 0.95, 0.95;
  Eigen::Matrix<double, 3, 1> BIAS_LIM_ABS;   // clip limits (acceleration)
  BIAS_LIM_ABS << 2.0, 2.0, 3.0;
  while( ros::ok() )
  {
    /* Mark start(ts) and end(te) times for precise timing */
    ts = std::chrono::high_resolution_clock::now();
    prop_interval_ = ts - last_prop_t_;
    double delta_t = prop_interval_.count();
    
    /* Fill in actual dt, don't assume fixed intervals */
    timing_matrix_ << 0.5*delta_t*delta_t * I3x3,
                      delta_t * I3x3;
    sys_A_.block<3,3>(0, 3) = delta_t * I3x3;
    sys_A_.topRightCorner<6,3>() = timing_matrix_;
    sys_B_.block<6,3>(0, 0) =  timing_matrix_;
      
    sys_A_t_ = sys_A_.transpose();

    std::unique_lock<std::mutex> spmtx( state_prop_mutex_, std::defer_lock );
    if( spmtx.try_lock() )
    {
      /* don't use stale ctrl_input_u_ for some reason */
      n_stprops_since_update_++;
      if( estimator_off_ || n_stprops_since_update_ > 3 )
      {
        ctrl_input_u_ << 0.0, 0.0, 0.0;   // clear ctrl until further update
        best_estimate_ = best_estimate_;  // do not propagate
      }
      else
      {
        /* propagate state forward */
        best_estimate_ = sys_A_ * best_estimate_ +
                         sys_B_ * (ctrl_input_u_.cwiseProduct(input_shaping));
        state_cov_P_ = sys_A_*state_cov_P_*sys_A_t_ + proc_noise_Q_;

        /* only clip estimated bias accelerations */
        best_estimate_.block<3,1>(6,0) = BIAS_LIM_ABS.cwiseMin(
                      best_estimate_.block<3,1>(6,0).cwiseMax(-BIAS_LIM_ABS) );
      }
      
      /* fill state information: this will be unrolled by gcc */
      for( int idx=0; idx < nStates; idx++ )
        state_msg_.state_vector[idx] = best_estimate_(idx);
      spmtx.unlock();

      // publish
      state_msg_.header.stamp = ros::Time::now();

      last_prop_t_ = te = std::chrono::high_resolution_clock::now();
      int dt = std::chrono::duration_cast<uSeconds>(te-ts).count();
      
      /* Fill in extra debug information: subject to change */
      state_msg_.state_vector[9] = static_cast<double>(n_stprops_since_update_);
      state_msg_.state_vector[10] = delta_t;
      //state_msg_.state_vector[10] = state_cov_P_.diagonal().norm();
      state_msg_.state_vector[11] = ctrl_input_u_.norm();
      bias_pub_.publish( state_msg_ );

      std::this_thread::sleep_for( uSeconds(estimator_period_us_ - dt) );
    }
    else
      std::this_thread::sleep_for( uSeconds(estimator_period_us_) );
  }
}

void BiasEstimator::state_updation()
{
  /* State update equations:
        G = P_k * C' * (C*P_k*C' + R)'
        x = x + G*(Z - Cx)
        P = (I-GC)*P
     If C = I, speed-up/de-clutter the code here.
  */
  Eigen::MatrixXd innov = ( meas_matrix_C_ * state_cov_P_ * meas_matrix_C_t +
                            meas_noise_R_ ).inverse();
  state_prop_mutex_.lock();
    kalman_gain_G_ = state_cov_P_ * meas_matrix_C_t * innov;
    best_estimate_ = best_estimate_ +
                kalman_gain_G_*(measurement_z_ - meas_matrix_C_*best_estimate_);
    state_cov_P_ = (I9x9 - kalman_gain_G_*meas_matrix_C_)*state_cov_P_;
  state_prop_mutex_.unlock();
}

void BiasEstimator::setMeasurement( const Eigen::Matrix<double, 6, 1> &m )
{
  measurement_z_ = m;
  n_stprops_since_update_ = 0;
  state_updation();
}

void BiasEstimator::setControlInput( const Eigen::Matrix<double, 4, 1> &c )
{
  state_prop_mutex_.lock();
    ctrl_input_u_[0] = c[0];
    ctrl_input_u_[1] = c[1];
    ctrl_input_u_[2] = c[2] + 9.81;
  state_prop_mutex_.unlock();
}
void BiasEstimator::getEstimatedBiases( Eigen::Matrix<double, 3, 1> &eb )
{
  // return 3 elements starting at 6: bn, be, bd.
  if( estimator_output_shaping_ > 0.95 )
    estimator_output_shaping_ = 1.0;
  else
    updateOutputShapingFactor();
  eb = estimator_output_shaping_ * best_estimate_.tail<3>();
}

