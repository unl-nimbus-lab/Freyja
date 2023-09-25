#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/quaternion.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "freyja_msgs/msg/ctrl_command.hpp"
#include "freyja_msgs/msg/freyja_interface_status.hpp"
#include "freyja_msgs/msg/current_state.hpp"
#include "freyja_msgs/msg/reference_state.hpp"

#include "eigen3/Eigen/Dense"

typedef freyja_msgs::msg::FreyjaInterfaceStatus FreyjaIfaceStatus;
typedef freyja_msgs::msg::CurrentState          CurrentState;
typedef freyja_msgs::msg::ReferenceState        ReferenceState;
typedef mavros_msgs::srv::CommandBool           MavrosArming;
typedef std_srvs::srv::Trigger                  Trigger;
typedef std_srvs::srv::SetBool                  BoolServ;

using std::placeholders::_1;
using std::placeholders::_2;

#define ROS_NODE_NAME "apm_arbitration"

/* *************************
        RC MAPPING
  aux1 ---> pilot has given computer full authority
  aux2 ---> pilot has requested a software landing
  aux3 ---> pilot has put vehicle into LAND mode (hardware/AP)
  aux4 ---> 
* **************************
*/

#define vModes(OP) \
  OP(NO_CONNECT) \
  OP(ARMED_COMP) \
  OP(ARMED_NOCOMP) \
  OP(DISARMED_COMP) \
  OP(DISARMED_NOCOMP) \
  OP(COMPUTER_CTRL) \
  OP(ARMED) \
  OP(TAKEOFF) \
  OP(HOVER_READY) \
  OP(LANDING) \
  OP(DISARMED)

#define mModes(OP) \
  OP(NOT_INIT) \
  OP(PENDING_PILOT) \
  OP(PENDING_CMD) \
  OP(TAKING_OFF) \
  OP(HOVERING) \
  OP(E_LANDING) \
  OP(MISSION_EXEC) \
  OP(MISSION_END)

#define MAKE_ENUM(OP) OP,
#define MAKE_ENUM_STRS(OP) #OP,

namespace VehicleMode {
  enum VehicleMode { vModes(MAKE_ENUM) };
}

namespace MissionMode {
  enum MissionMode { mModes(MAKE_ENUM) };
}

const char* const VehicleModeName[] = { vModes(MAKE_ENUM_STRS) };
const char* const MissionModeName[] = { mModes(MAKE_ENUM_STRS) };


class ApmModeArbitrator : public rclcpp::Node
{
  VehicleMode::VehicleMode vehicle_mode_;
  MissionMode::MissionMode mission_mode_;

  Eigen::Vector4d pos_ned_;         // current state 
  Eigen::Vector4d arming_ned_;
  ReferenceState incoming_ref_;     // policy's references
  ReferenceState manager_refstate_; // manager's reference state
  double vel_d_;

  float init_hover_pd_;
  float ARM_TAKEOFF_DELAY;          // wait after arming
  float MISSION_WDG_TIMEOUT;        // wdog timeout to switch back to hover
  float HOVER_WDG_TIMEOUT;          // wdog timeout to switch to landing
  float takeoff_spd_;               // rate of ascent (scalar)

  bool e_landing_;                  // in emergency landing [Freyja-controlled]
  bool arm_req_sent_;               // arming sent
  bool forward_ref_state_;          // should we simply fwd incoming reference (policy mode)
  bool target_state_avail_;         // has ext. policy/mission provided a reference yet
  bool software_trigger_recv;       // has user code triggered arm+takeoff request
  bool await_cmd_after_switch_;     // should we wait for user even after an RC switch?
  bool land_from_hovertimeout_;     // should we land if auto-hovering for some duration?

  float t_clock_;                   // Node time-keeping (zero at constructor)
  float t_comp_armed_;
  float t_tgtstate_update_;
  float t_curstate_update_;
  float t_enter_auto_hover_;
  rclcpp::Time t_manager_init_;

  bool rc_aux_switches_[4];         // state of rc aux switches
  
  float arbitrator_rate_;
  rclcpp::TimerBase::SharedPtr manager_timer_;

  protected:
    void sendMavrosArmCommand( const bool );
    void loadParameters();
  
  public:
    ApmModeArbitrator();
    void manager();

    rclcpp::Subscription<FreyjaIfaceStatus>::SharedPtr fstatus_sub_;
    void freyjaStatusCallback( const FreyjaIfaceStatus::ConstSharedPtr );

    rclcpp::Subscription<CurrentState>::SharedPtr curstate_sub_;
    inline void currentStateCallback( const CurrentState::ConstSharedPtr msg )
    { 
      pos_ned_ << Eigen::Map<const Eigen::Vector3d>( msg->state_vector.data() ),
                  double(msg->state_vector[8]);
      vel_d_ = msg->state_vector[5];   
      t_curstate_update_ = t_clock_;           
    }

    rclcpp::Subscription<ReferenceState>::SharedPtr tgtstate_sub_;
    void targetStateCallback( const ReferenceState::ConstSharedPtr );

    rclcpp::Client<MavrosArming>::SharedPtr arming_client_;
    rclcpp::Client<BoolServ>::SharedPtr biasreq_client_;
    rclcpp::Client<BoolServ>::SharedPtr extfcorr_client_;
    rclcpp::Client<BoolServ>::SharedPtr groundidle_client_;

    rclcpp::Publisher<ReferenceState>::SharedPtr refstate_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr manager_mode_pub_;

    rclcpp::Service<Trigger>::SharedPtr elanding_serv_;
    void eLandingServiceHandler( const Trigger::Request::SharedPtr,
                                 const Trigger::Response::SharedPtr );
    rclcpp::Service<Trigger>::SharedPtr start_manager_serv_;

    inline void updateManagerRefNED( const Eigen::Vector4d &ned )
    {
      manager_refstate_.pn = ned(0);
      manager_refstate_.pe = ned(1);
      manager_refstate_.pd = ned(2);
      manager_refstate_.yaw = ned(3);
    }

    void processRCEvents();
    bool landingInProgress( bool _init = false );
    void keepDisarmedAndHappy();

    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr csrs_cb_group_;
};