/* Maintain and publish the best estimate of the state using an LQE system.
  State model is shared with the controller module. See the bottom of this file
  for a definition of the model used.
  
  > This can/will replace the state_manager node for several applications.
  > Will see if this can be extended to powerful non-linear models (UKF/PF).
  > LQE_INTEGRATOR_ORDER is gotten from a compiler flag, check CMakeLists.
  
  
  -- aj / Apr 18, 2019.
*/

#include <mutex>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <freyja_msgs/CurrentState.h>
#include <freyja_msgs/CurrentStateEst.h>
#include <freyja_msgs/AsctecData.h>
#include <freyja_msgs/ControllerDebug.h>


#include "aj_filter_collection.cpp"

typedef freyja_msgs::ControllerDebug::ConstPtr LqrInputAccel;

#if LQE_INTEGRATOR_ORDER_ == 2
  const int nStates = 9;
#else
  const int nStates = 6;
#endif
const int nCtrl = 3;
const int nMeas = 6;

class StateEstimator
{
  // ros
  ros::NodeHandle nh_, priv_nh_;
  
  // consider state vector of position + velocity
  Eigen::Matrix<double, nStates, nStates> sys_A_, sys_A_t_;
  Eigen::Matrix<double, nStates, nCtrl> sys_B_;
  Eigen::Matrix<double, nStates, nStates> proc_noise_Q_;
  Eigen::Matrix<double, nMeas, nMeas> meas_noise_R_;
  Eigen::Matrix<double, nStates, nMeas> kalman_gain_G_;
  
  Eigen::Matrix<double, nStates, 1> best_estimate_;
  Eigen::Matrix<double, nCtrl, 1> ctrl_input_u_;
  Eigen::Matrix<double, nMeas, 1> measurement_z_;
  Eigen::Matrix<double, 3, 1> prev_vel1_, prev_vel2_;
  Eigen::Matrix<double, nMeas, nStates> meas_matrix_C_;
  Eigen::Matrix<double, nStates, nMeas> meas_matrix_C_t;
  Eigen::Matrix<double, nStates, nStates> state_cov_P_;
  Eigen::Matrix<double, 6, 6> I6x6;
  Eigen::Matrix<double, 3, 3> I3x3;
  Eigen::Matrix<double, 9, 9> I9x9;
  Eigen::Matrix<double, 6, 3> timing_matrix_;
  
  // more parameters
  int estimator_rate_;

  // prevent state propagation when other callbacks are happening
  std::mutex state_prop_mutex_;
  std::thread state_prop_thread_;
  volatile bool state_propagation_alive_;
  volatile int n_stprops_since_update_;
  int n_stprops_allowed_;
  
  // timing related objects
  std::chrono::time_point<std::chrono::high_resolution_clock> ts, te, last_prop_t_;
  std::chrono::duration<double> prop_interval_;

  // final ros data published
  freyja_msgs::CurrentStateBiasFree state_msg_;
  
  public:
    StateEstimator();
    ~StateEstimator();

    ros::Publisher state_pub_;

    // listen to state information from an external source
    ros::Subscriber state_sub_;
    void stateUpdatesCallback( const freyja_msgs::CurrentState::ConstPtr & );

    // listen to full control commands (atti + accel) from the controller
    ros::Subscriber ctrl_input_sub_;
    void controlInputCallback( const LqrInputAccel & );

    // run state propagation at a constant rate
    void state_propagation( );
    ros::Timer estimator_timer_;

    // run state updation when required
    void state_updation();

    // init values
    void initLqeSystem();

    // can be useful for small filters, unused at the moment
    inline void medianFilter();

};

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
