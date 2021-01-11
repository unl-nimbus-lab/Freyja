/*
  Optimal full state 3-axis state observer: Luenberger/Kalman fashion.
  BiasEstimator is befriended by LQRController class, and it therefore has
  direct unfettered access to its members. This prevents memcpy overheads, and
  keeps only one copy of variables such as current_state and control_input.
  Race conditions are handled here. LQRController will contain a reference to an
  object of this class.
  
  -- aj / 27th Sept, 2019.
*/

#include <mutex>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <freyja_msgs/CurrentStateBiasEst.h>

typedef std::chrono::microseconds uSeconds;
typedef std::chrono::high_resolution_clock ClockTime;
typedef std::chrono::time_point<ClockTime> ClockTimePoint;

const int nStates = 9;
const int nCtrl = 3;
const int nMeas = 6;

class BiasEstimator
{
  friend class LQRController;
  
  ros::NodeHandle nh_, priv_nh_;
  Eigen::Matrix<double, nStates, nStates> sys_A_, sys_A_t_;
  Eigen::Matrix<double, nStates, nCtrl> sys_B_;
  
  Eigen::Matrix<double, nStates, 1> best_estimate_;
  Eigen::Matrix<double, nCtrl, 1> ctrl_input_u_;
  Eigen::Matrix<double, nMeas, 1> measurement_z_;

  /* KF matrices */
  Eigen::Matrix<double, nStates, nStates> proc_noise_Q_;
  Eigen::Matrix<double, nMeas, nMeas> meas_noise_R_;
  Eigen::Matrix<double, nStates, nMeas> kalman_gain_G_;
  Eigen::Matrix<double, nMeas, nStates> meas_matrix_C_;
  Eigen::Matrix<double, nStates, nMeas> meas_matrix_C_t;
  Eigen::Matrix<double, nStates, nStates> state_cov_P_;
  
  /* Convenience matrices */
  Eigen::Matrix<double, 6, 6> I6x6;
  Eigen::Matrix<double, 3, 3> I3x3;
  Eigen::Matrix<double, 9, 9> I9x9;
  Eigen::Matrix<double, 6, 3> timing_matrix_;
  
  /* parameters */
  int estimator_rate_, estimator_period_us_;
  bool estimator_off_;

  /* prevent state propagation when other callbacks are happening */
  std::mutex state_prop_mutex_;
  std::thread state_prop_thread_;
  volatile bool state_propagation_alive_;
  volatile int n_stprops_since_update_;
  
  std::thread st_upd_thread_;
  
  /* timing related objects */
  ClockTimePoint ts, te, last_prop_t_;
  std::chrono::duration<double> prop_interval_;
  
  /* smooth-in the estimator using a time-constant factor */
  double estimator_tc_;
  double estimator_output_shaping_;
  ClockTimePoint t_estimator_on_;
  std::chrono::duration<double> tc_interval_;

  /* final ros data published */
  freyja_msgs::CurrentStateBiasEst state_msg_;
  public:
    BiasEstimator();
    ~BiasEstimator();
    
    ros::Publisher bias_pub_;
    
    /* State propagation - runs at a fixed rate */
    __attribute__((optimize("unroll-loops")))
    void state_propagation();
    
    /* State updation - gets called from lqr class */
    void state_updation();
    
    /* initialise system */
    void initEstimatorSystem();
    
    void setMeasurement( const Eigen::Matrix<double, 6, 1> & );
    void setControlInput( const Eigen::Matrix<double, 4, 1> & );
    void getEstimatedBiases( Eigen::Matrix<double, 3, 1> & );
    
    /* set and reset accessors */
    inline void enable()
    {
      t_estimator_on_ = ClockTime::now();
      estimator_off_ = false;
    }
    inline void disable()
    {
      best_estimate_.tail<3>() << 0.0, 0.0, 0.0;
      estimator_output_shaping_ = 0.0;
      estimator_off_ = true;
    }
    
    /* Time constant calc */
    inline void updateOutputShapingFactor()
    {
      /* Output shaping as a cubic polynomial of time since ON */
      auto tnow = ClockTime::now();
      tc_interval_ = tnow - t_estimator_on_;
      double t_ratio = tc_interval_.count() / estimator_tc_;
      estimator_output_shaping_ = std::min( 1.0, t_ratio * t_ratio * t_ratio );
    }
};

/*
  ** State model and the choice of variable names **
  x(k) = [pn, pe, pd, vn, ve, vd, bn, be, bd]^T
  u(k) = [an, ae, ad]^T
  
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
