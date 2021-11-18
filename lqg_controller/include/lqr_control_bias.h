/* This is the big star of the show.
   Node implementing the LQR feedback loop, with all the parameters and 
   model-configuration stored here. Now, ideally I'd have designed it in a
   way that enables others (or myself) to write their own variants of the
   actual control loop without having to rewrite the whole thing. But clearly
   I am not that smart. But I use Eigen to pretend that I am.
   
   You can replace this node if you can write something that subscribes to
   full-state information provided by "state_manager", and outputs something
   that can be consumed by an interface/comm node that wants "thrust, roll,
   pitch and yaw" as inputs.
   
   -- aj / 17th Nov, 2017.
*/
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>

#include <freyja_msgs/CurrentState.h>
#include <freyja_msgs/CtrlCommand.h>
#include <freyja_msgs/ControllerDebug.h>
#include <freyja_msgs/ReferenceState.h>

#include <eigen3/Eigen/Dense>

#include "bias_estimator.h"

typedef freyja_msgs::ReferenceState TrajRef;
typedef std_srvs::SetBool::Request BoolServReq;
typedef std_srvs::SetBool::Response BoolServRsp;
typedef Eigen::Matrix<double, 6, 1> PosVelNED;

class LQRController
{
  ros::NodeHandle nh_, priv_nh_;
  freyja_msgs::CurrentState state_vector_;
  Eigen::Matrix<double, 7, 1> reduced_state_;
  
  /* Reference state vector */
  Eigen::Matrix<double, 7, 1> reference_state_;
  Eigen::Matrix<double, 4, 1> reference_ff_;
  bool enable_flatness_ff_;
  
  /* Rate of execution for LQR's feedback */
  int controller_rate_;
  
  /* System matrices */
  Eigen::MatrixXf sys_A_, sys_B_;
  Eigen::MatrixXf lqr_Q_, lqr_R_;
  Eigen::Matrix<double, 4, 7> lqr_K_;
  bool use_stricter_gains_;
  
  /* Vehicle properties */
  float total_mass_;

  
  /* Rotation matrices */
  Eigen::Matrix<double, 3, 3> rot_yaw_;
  
  float STATEFB_MISSING_INTRV_;
  ros::Time last_state_update_t_;
  bool have_state_update_;
  bool have_reference_update_;
  
  std::mutex reference_state_mutex_;
  
  /* Bias estimator reference object */
  BiasEstimator &bias_est_;
  bool bias_compensation_req_;
  bool bias_compensation_off_;
  Eigen::Matrix<double, 3, 1> f_biases_;
  std::thread bias_handler_thread_;
  
  /*Dynamic mass estimation and compensation */
  bool enable_dyn_mass_correction_;
  bool enable_dyn_mass_estimation_;
  
  public:
    LQRController( BiasEstimator & );
    void initLqrSystem();
    
    ros::Subscriber state_sub_;
    void stateCallback( const freyja_msgs::CurrentState::ConstPtr & ) __attribute__((hot));
    
    ros::ServiceServer bias_enable_serv_;
    bool biasEnableServer( BoolServReq&, BoolServRsp& );
    
    ros::Publisher atti_cmd_pub_;
    ros::Publisher controller_debug_pub_;
    
    ros::Timer controller_timer_;
    void computeFeedback( const ros::TimerEvent & ) __attribute__((optimize("unroll-loops")));
    
    ros::Subscriber reference_sub_;
    void trajectoryReferenceCallback( const TrajRef::ConstPtr & );

    /* helper function to calculate yaw error */
    static constexpr inline double calcYawError( const double&, const double& ) __attribute__((always_inline));
    
    /* estimate actual mass in flight */
    void estimateMass( const Eigen::Matrix<double, 4, 1> &, ros::Time & );
    ros::Publisher est_mass_pub_;
};
