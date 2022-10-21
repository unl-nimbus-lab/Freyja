/*
  ** State model and the choice of variable names **
  State propagation model:
    x(k+1) = A.x(k) + B.u(k)
    z(k) = C.x(k) + Q
    Process noise: Q
    State covariance: P
    
  Variable names have both a colloquial part and the mathematical-term part.
  >> `state_cov_P_` is the state covariance matrix, P.
  >> `proc_noise_Q_` is noise in the state propagation (aka process noise).
  >> `meas_matrix_C_` is the state observation matrix
  >> `measurement_z_` is the z(k) observation vector (and not altitude).
  >> `best_estimate_` represents the x(k) vector.
  
  State updation model:
    G = P * C' * (C*P*C' + R)'
    x = x + G*(Z - Cx)
    P = (I-GC)*P
    Measurement noise: R
  
*/
#include <thread>
#include <mutex>

namespace FreyjaUtils
{
  const int nStates = 9;
  const int nCtrl = 3;
  const int nMeas = 3;
  class KalmanFilter : public Filter
  {
    // state matrices
    Eigen::Matrix<double, nStates, nStates> sys_A_, sys_A_t_;
    Eigen::Matrix<double, nStates, nCtrl> sys_B_;
    Eigen::Matrix<double, nStates, nStates> state_cov_P_;
    Eigen::Matrix<double, nStates, nStates> proc_noise_Q_;
    Eigen::Matrix<double, nMeas, nMeas> meas_noise_R_;
    Eigen::Matrix<double, nStates, nMeas> kalman_gain_G_;
    
    Eigen::Matrix<double, nStates, 1> best_estimate_;
    Eigen::Matrix<double, nCtrl, 1> ctrl_input_u_;
    Eigen::Matrix<double, nMeas, 1> measurement_z_;
    Eigen::Matrix<double, nMeas, nStates> meas_matrix_C_;
    Eigen::Matrix<double, nStates, nMeas> meas_matrix_C_t_;

    Eigen::Matrix<double, 6, 3> timing_matrix_;
    
    // more parameters
    int estimator_rate_;
    int estimator_period_us_;

    // prevent state propagation when other callbacks are happening
    std::mutex state_prop_mutex_;
    std::thread state_prop_thread_;
    volatile bool state_propagation_alive_;
    volatile int n_stprops_since_update_;
    
    // timing related objects
    std::chrono::time_point<std::chrono::high_resolution_clock> ts, te, last_prop_t_;
    std::chrono::duration<double> prop_interval_;
    
    public:
      KalmanFilter();
      ~KalmanFilter();
      
      void init();
      void init( int estimator_rate );

      // measurement input: z
      void setMeasurementInput( const Eigen::Matrix<double, nMeas, 1> & );

      // control update
      void setControlInput( const Eigen::Matrix<double, nCtrl, 1> & );

      // run state propagation : separate thread
      void state_propagation( );

      // run state updation when required
      void state_updation();

      // init values
      void initLqeSystem();

      // accessor for state estimate
      void getStateEstimate( Eigen::Matrix<double, 6, 1> & );
      void getStateEstimate( Eigen::VectorXd& );
  };


  /* Linear quadratic estimator (LQE) implementation.

  State propagation happens at a rate higher than the controller. External
  sources can trigger an update function at sporadic times.

  -- aj / Apr 18, 2019
  */

  #define ZERO3x3 Eigen::MatrixXd::Zero(3,3)
  #define ZERO6x6 Eigen::MatrixXd::Zero(6,6)
  #define ZERO9x9 Eigen::MatrixXd::Zero(9,9)
  #define IDEN3x3 Eigen::MatrixXd::Identity(3,3)
  #define IDEN6x6 Eigen::MatrixXd::Identity(6,6)
  #define IDEN9x9 Eigen::MatrixXd::Identity(9,9)

  typedef std::chrono::microseconds uSeconds;

  KalmanFilter::KalmanFilter( )
  {
    n_stprops_since_update_ = 0;
    state_propagation_alive_ = false;
  }

  void KalmanFilter::init()
  {
    // constants for the lq-estimator
    initLqeSystem( );
                            
    // separate thread for state prop
    state_propagation_alive_ = true;
    last_prop_t_ = std::chrono::high_resolution_clock::now();
    state_prop_thread_ = std::thread( &KalmanFilter::state_propagation, this );

    std::cout << "KalmanFilter init!" << std::endl;
  }

  void KalmanFilter::init( int estimator_rate )
  {
    estimator_rate_ = std::move( estimator_rate );
    estimator_period_us_ = std::round( (1.0/estimator_rate_)*1000*1000 );

    init();
  }

  KalmanFilter::~KalmanFilter()
  {
    state_propagation_alive_ = false;
    state_prop_thread_.join();
    std::cout << "KalmanFilter: propagation thread stopped!" << std::endl;
  }

  void KalmanFilter::initLqeSystem( )
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

    proc_noise_Q_ << 0.1*IDEN3x3, ZERO3x3, ZERO3x3,
                      ZERO3x3, 0.25*IDEN3x3, ZERO3x3,
                      ZERO3x3, ZERO3x3, 1*IDEN3x3;
    meas_noise_R_ << 0.2*IDEN3x3;

    // only positions are measured (e.g., mocap)
    meas_matrix_C_ << IDEN3x3, ZERO3x3, ZERO3x3;
    meas_matrix_C_t_ = meas_matrix_C_.transpose();
    
    // guess some initial state, cov 
    best_estimate_.setZero();
    state_cov_P_ = 1*IDEN9x9;

    timing_matrix_ = Eigen::MatrixXd::Zero(6,3);
  }

  /* State propagation, done at a fixed rate on a separate thread.
  On smaller/less-powerful machines, the thread::sleep_for() methods may not
  be highly accurate, and thus, values of "dt" in A and B matrices can't be 
  assumed to be constant (=1.0/estimator_rate). The function measures the time
  since the propagation uses that for "dt". The time it sleeps for is also
  stabilized by accounting for the time it took to run state propagation.
  */
  void KalmanFilter::state_propagation( )
  {
    while( state_propagation_alive_ )
    {
      ts = std::chrono::high_resolution_clock::now();
      prop_interval_ = ts - last_prop_t_;
      double delta_t = prop_interval_.count();
      
      timing_matrix_ << 0.5*delta_t*delta_t * IDEN3x3,
                            delta_t * IDEN3x3;
      sys_A_.block<3,3>(0, 3) = delta_t * IDEN3x3;
      sys_A_.topRightCorner<6,3>() = timing_matrix_;
      sys_B_.block<6,3>(0, 0) =  timing_matrix_;
      
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

  void KalmanFilter::state_updation()
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

  void KalmanFilter::setControlInput( const Eigen::Matrix<double, nCtrl, 1> &ctrl_u )
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

  void KalmanFilter::setMeasurementInput( const Eigen::Matrix<double, nMeas, 1> &meas )
  {

    // no need to mutexify this, since measurement_z_ is only used in update function
    measurement_z_ = std::move(meas);

    // call state update, reset propagation counter
    state_updation();
    n_stprops_since_update_ = 0;
  }

  /* Return estimated state 6x1 [posNED, velNED] */
  void KalmanFilter::getStateEstimate( Eigen::Matrix<double, 6, 1> &x_est )
  {
    state_prop_mutex_.lock();
      x_est = best_estimate_.head<6>();
    state_prop_mutex_.unlock();
  }

  void KalmanFilter::getStateEstimate( Eigen::VectorXd &x_est )
  {
    static Eigen::Matrix<double, 6, 1> _x;
    getStateEstimate( _x );
    x_est = _x;
  }


}  // namespace