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
#include <state_manager/CurrentState.h>
#include <lqr_control/CtrlCommand.h>
#include <lqr_control/ControllerDebug.h>
#include <trajectory_provider/ReferenceState.h>
#include <eigen3/Eigen/Dense>

typedef trajectory_provider::ReferenceState TrajRef;


class LQRController
{
  ros::NodeHandle nh_;
  state_manager::CurrentState state_vector_;
  Eigen::Matrix<double, 7, 1> reduced_state_;
  
  /* Reference state vector */
  Eigen::Matrix<double, 7, 1> reference_state_;
  
  /* Rate of execution for LQR's feedback */
  int controller_rate_;
  
  /* System matrices */
  Eigen::MatrixXf sys_A_, sys_B_;
  Eigen::MatrixXf lqr_Q_, lqr_R_;
  Eigen::Matrix<double, 4, 7> lqr_K_;
  
  /* Vehicle properties */
  float total_mass_;
  
  /* Rotation matrices */
  Eigen::Matrix<double, 3, 3> rot_yaw_;
  
  bool have_state_update_;
  bool have_reference_update_;
  
  std::mutex reference_state_mutex_;
  
  public:
    LQRController();
    void initLqrSystem();
    
    ros::Subscriber state_sub_;
    void stateCallback( const state_manager::CurrentState::ConstPtr & );
    
    ros::Publisher atti_cmd_pub_;
    ros::Publisher controller_debug_pub_;
    
    ros::Timer controller_timer_;
    void computeFeedback( const ros::TimerEvent & );
    
    ros::Subscriber reference_sub_;
    void trajectoryReferenceCallback( const TrajRef::ConstPtr & );
};