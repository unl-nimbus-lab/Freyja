/* Linear quadratic estimator (LQE) implementation.

  State propagation happens at a rate higher than the controller. External
  sources can trigger an update function at sporadic times.

  -- aj / Apr 18, 2019
*/
#include "state_estimator.hpp"

#define ROS_NODE_NAME "state_estimator_kf"
#define ZERO3x3 Eigen::MatrixXd::Zero(3,3)
#define ZERO6x6 Eigen::MatrixXd::Zero(6,6)
#define ZERO9x9 Eigen::MatrixXd::Zero(9,9)
#define IDEN3x3 Eigen::MatrixXd::Identity(3,3)
#define IDEN6x6 Eigen::MatrixXd::Identity(6,6)
#define IDEN9x9 Eigen::MatrixXd::Identity(9,9)

typedef std::chrono::microseconds uSeconds;

StateEstimator::StateEstimator( )
{
  // experimental:
  n_stprops_since_update_ = 0;
  n_stprops_allowed_ = 2.5;
  state_propagation_alive_ = false;
}

void StateEstimator::init( int e_rate, float mn, float np, float nv, float na )
{
  estimator_rate_ = std::move( e_rate );
  estimator_period_us_ = std::round( (1.0/estimator_rate_)*1000*1000 );

  // constants for the lq-estimator
  initLqeSystem( mn, np, nv, na );
                           
  // separate thread for state prop
  state_propagation_alive_ = true;
  last_prop_t_ = std::chrono::high_resolution_clock::now();
  state_prop_thread_ = std::thread( &StateEstimator::state_propagation, this );
}

StateEstimator::~StateEstimator()
{
  state_propagation_alive_ = false;
  state_prop_thread_.join();
  std::cout << "StateEstimator: propagation thread stopped!" << std::endl;
}

void StateEstimator::initLqeSystem(float m_noise, float np, float nv, float na)
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
  
  float dt = 1.0/float(estimator_rate_);
  float dt2 = dt*dt/2.0;

  sys_A_ << 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2, 0.0, 0.0, 
            0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
            Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3);
  sys_A_t_ = sys_A_.transpose();

  sys_B_ << ZERO3x3,
            IDEN3x3,
            ZERO3x3;

  proc_noise_Q_ <<  np*IDEN3x3, ZERO3x3, ZERO3x3,
                    ZERO3x3, nv*IDEN3x3, ZERO3x3,
                    ZERO3x3, ZERO3x3, na*IDEN3x3;
  meas_noise_R_ << m_noise*IDEN3x3;

  meas_matrix_C_ << IDEN3x3, ZERO3x3, ZERO3x3;

  meas_matrix_C_t_ = meas_matrix_C_.transpose();

  state_cov_P_ = 10.0*IDEN9x9;
  
  // guess some initial state 
  best_estimate_.setZero();
  ctrl_input_u_.setZero();
  prev_vel1_.setZero();
  prev_vel2_.setZero();

  timing_matrix_ = Eigen::MatrixXd::Zero(6,3);
  printf( "LQE init with params: %0.2f, %0.2f, %0.2f, %0.2f -- dt %0.3f: ", m_noise, np, nv, na, dt );
  std::cout << "done!" << std::endl;
}

/* State propagation, done at a fixed rate on a separate thread.
On smaller/less-powerful machines, the thread::sleep_for() methods may not
be highly accurate, and thus, values of "dt" in A and B matrices can't be 
assumed to be constant (=1.0/estimator_rate). The function measures the time
since the propagation uses that for "dt". The time it sleeps for is also
stabilized by accounting for the time it took to run state propagation.
*/
__attribute__((optimize("unroll-loops")))
void StateEstimator::state_propagation( )
{
  while( state_propagation_alive_ )
  {
    ts = std::chrono::high_resolution_clock::now();
    prop_interval_ = ts - last_prop_t_;
    double delta_t = prop_interval_.count();
    
    /*
    timing_matrix_ << 0.5*delta_t*delta_t * IDEN3x3,
                          delta_t * IDEN3x3;
    sys_A_.block<3,3>(0, 3) = delta_t * IDEN3x3;
    sys_A_.topRightCorner<6,3>() = timing_matrix_;
    sys_B_.block<6,3>(0, 0) =  timing_matrix_;
    */
    sys_A_.topRightCorner<6,6>().diagonal().setConstant(delta_t);
    sys_A_.topRightCorner<3,3>().diagonal().setConstant(0.5*delta_t*delta_t);
    
    sys_A_t_ = sys_A_.transpose();

    std::unique_lock<std::mutex> spmtx( state_prop_mutex_, std::defer_lock );
    if( spmtx.try_lock() )
    {
      // propagate state forward
      best_estimate_ = sys_A_ * best_estimate_ +  sys_B_ * ctrl_input_u_;
      state_cov_P_ = sys_A_*state_cov_P_*sys_A_t_ + proc_noise_Q_;

      spmtx.unlock();

      last_prop_t_ = te = std::chrono::high_resolution_clock::now();
      int dt = std::chrono::duration_cast<uSeconds>(te-ts).count();
      std::this_thread::sleep_for( uSeconds(estimator_period_us_ - dt) );
    }
    else
      std::this_thread::sleep_for( uSeconds(estimator_period_us_) );
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
  Eigen::MatrixXd innov = ( meas_matrix_C_ * state_cov_P_ * meas_matrix_C_t_ + meas_noise_R_ ).inverse();
  state_prop_mutex_.lock();
    kalman_gain_G_ = state_cov_P_ * meas_matrix_C_t_ * innov;
    best_estimate_ = best_estimate_ + kalman_gain_G_*(measurement_z_ - meas_matrix_C_*best_estimate_);
    state_cov_P_ = (IDEN9x9 - kalman_gain_G_*meas_matrix_C_)*state_cov_P_;
  state_prop_mutex_.unlock();
}

void StateEstimator::setControlInput( const Eigen::Matrix<double, nCtrl, 1> &ctrl_u )
{
  /* Listening to debug from controller is neat because it contains the raw
     accelerations commanded, and also the feedback linearized attitudes that
     were computed. At the moment, we rely on accelerations, but in the future,
     one could incorporate attitude commands to do more non-linear stuff here.
  */
  // note: X = [pn, pe, pd ..]
  state_prop_mutex_.lock();
    ctrl_input_u_ = std::move(ctrl_u);
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

void StateEstimator::setMeasurementInput( const Eigen::Matrix<double, nMeas, 1> &meas )
{

  // no need to mutexify this, since measurement_z_ is only used in update function
  measurement_z_ = std::move(meas);

  // call state update, reset propagation counter
  state_updation();
  n_stprops_since_update_ = 0;
}

/* Return estimated state 6x1 [posNED, velNED] */
void StateEstimator::getStateEstimate( Eigen::Matrix<double, 9, 1> &x_est )
{
  state_prop_mutex_.lock();
    x_est = best_estimate_.head<9>();
  state_prop_mutex_.unlock();
}

