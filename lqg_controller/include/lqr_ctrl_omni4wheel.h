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
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/TransformStamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "freyja_msgs/msg/current_state.hpp"
#include "freyja_msgs/msg/wheel_command.hpp"
#include <freyja_msgs/msg/controller_debug.hpp>
#include <freyja_msgs/msg/reference_state.hpp>

#include <eigen3/Eigen/Dense>

#include "bias_estimator.h"

typedef freyja_msgs::msg::ReferenceState TrajRef;
typedef freyja_msgs::msg::CurrentState CurrentState;
typedef std_srvs::srv::SetBool BoolServ;
typedef freyja_msgs::msg::WheelCommand Wheel_Command;
typedef freyja_msgs::msg::ControllerDebug CTRL_Debug;

typedef Eigen::Matrix<double, 6, 1> PosVelNED;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

using std::placeholders::_1;
using std::placeholders::_2;

class LQRController : public rclcpp::Node
{
  CurrentState state_vector_;
  Eigen::Matrix<double, 6, 1> reduced_state_;
  
  /* Reference state vector */
  Eigen::Matrix<double, 6, 1> reference_state_;
  Eigen::Matrix<double, 3, 1> reference_ff_;
  Eigen::Matrix<double, 3, 1> accel_refs_;
  
  /* Controller properties */
  int controller_rate_;
  bool use_stricter_gains_;
  std::string controller_type_;
  bool enable_flatness_ff_;
  bool enable_accel_limiting_;
  bool enable_accel_feedfwd_;
  double accel_ff_factor_;
  
  /* System matrices */
  Eigen::MatrixXf sys_A_, sys_B_;
  Eigen::MatrixXf lqr_Q_, lqr_R_;
  Eigen::Matrix<double, 3, 6> lqr_K_;
  Eigen::Matrix<double, 4, 3> wheel_geometry_matrix_;
  double wheel_rad_;
  
  /* Rotation matrices */
  Eigen::Matrix<double, 3, 3> rot_yaw_;
  
  float STATEFB_MISSING_INTRV_;
  rclcpp::Time last_state_update_t_;
  bool have_state_update_;
  bool have_reference_update_;
  
  std::mutex reference_state_mutex_;
  
  /* Bias estimator reference object */
  BiasEstimator &bias_est_;
  bool bias_compensation_req_;
  bool bias_compensation_off_;
  Eigen::Matrix<double, 3, 1> f_biases_;
  std::thread bias_handler_thread_;
  
  
  public:
    LQRController( BiasEstimator & );
    void initLqrSystem();
    
    rclcpp::Subscription<CurrentState>::SharedPtr state_sub_;
    void stateCallback( const CurrentState::ConstSharedPtr ) __attribute__((hot));
    
    rclcpp::Service<BoolServ>::SharedPtr bias_enable_serv_;
    void biasEnableServer( const BoolServ::Request::SharedPtr,
                           const BoolServ::Response::SharedPtr );
    
    rclcpp::Publisher<Wheel_Command>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<CTRL_Debug>::SharedPtr controller_debug_pub_;
    
    rclcpp::TimerBase::SharedPtr controller_timer_;
    void computeFeedback( ) __attribute__((optimize("unroll-loops")));
    
    rclcpp::Subscription<TrajRef>::SharedPtr reference_sub_;
    void trajectoryReferenceCallback( const TrajRef::ConstSharedPtr );

    /* helper function to calculate yaw error */
    static constexpr inline double calcYawError( const double&, const double& ) __attribute__((always_inline));

    void constrainAccel( const Eigen::Matrix<double,3,1> &prev, Eigen::Matrix<double,3,1> &cur );
};
