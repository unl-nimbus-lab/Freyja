/* Node header definitions for converting Vicon + vehicle information
  from different formats to one coherent state-vector format.
  
  This node also computes velocity from Vicon.
  
  -- aj / 17th Nov, 2017
*/

#ifndef __STATE_INFO_GENERATOR_H__
#define __STATE_INFO_GENERATOR_H__

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#ifdef ROSVER_FOXY_OR_GALAC
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include "freyja_msgs/msg/current_state.hpp"
#include "freyja_msgs/msg/asctec_data.hpp"

#include "freyja_filters.cpp"
#include "state_estimator.hpp"

#include <eigen3/Eigen/Dense>

typedef geometry_msgs::msg::TransformStamped TFStamped;
typedef geometry_msgs::msg::TwistStamped TwStamped;
typedef nav_msgs::msg::Odometry CameraOdom;
typedef std_srvs::srv::SetBool BoolServ;

typedef freyja_msgs::msg::CurrentState CurrentState;

using std::placeholders::_1;
using std::placeholders::_2;

#define DEG2RAD(D) ((D)*3.1415326/180.0)
#define F_PI 3.14153

/* The full state vector is defined as:
  [ pn, pe, pd, vn, ve, vd, roll, pitch, yaw, roll_rate_, pitch_rate_, yaw_rate_ ]
*/
const int STATE_VECTOR_LEN = 13;

class StateManager: public rclcpp::Node
{
  std::vector<double> state_vector_;

  /* Store last n positions to smoothe it out */
  std::vector<double> prev_pn_;
  std::vector<double> prev_pe_;
  std::vector<double> prev_pd_;
  
  /* Store last n velocities to smoothe it out */
  std::vector<double> prev_vn_;
  std::vector<double> prev_ve_;
  std::vector<double> prev_vd_;
  
  /* Store last n angles to smoothe it out */
  std::vector<double> prev_roll_;
  std::vector<double> prev_pitch_;
  std::vector<double> prev_yaw;
  
  /* Book-keeping for velocities and rates */
  float last_pn_, last_pe_, last_pd_;
  float last_roll_, last_pitch_, last_yaw_;
  
  rclcpp::Time lastUpdateTime_;
  
  /* Pick the source of information at launch time */
  std::string state_source_;
  
  /* Filter-specific details for computing velocity */
  int filter_len_;
  std::string filter_type_;
  FreyjaFilters pose_filter_;
  FreyjaFilters rate_filter_;
  StateEstimator estimator_;
  bool use_kf_ = false;

  /* tf related */
  std::string tf_base_frame_;
  std::string tf_my_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* containers for handling gps data */
  double home_lat_, home_lon_;
  bool have_location_fix_;
  double compass_yaw_;
  
  bool have_arming_origin_;
  bool use_rtkbaseframe_;
  double map_rtk_pn_, map_rtk_pe_, map_rtk_pd_;
  double arming_gps_pn_, arming_gps_pe_, arming_gps_pd_;
  double gps_odom_pn_, gps_odom_pe_, gps_odom_pd_;
  double rtk_baseoffset_pn_, rtk_baseoffset_pe_, rtk_baseoffset_pd_;
  
  public:
    StateManager();
    /* launch-time parameter specifies which one to pick */
    void initPixhawkManager();
    void initTfManager();
    void initAsctecManager();
    void initMocapManager();
    void initCameraManager();

    /* Callback handler for Vicon */
    rclcpp::Subscription<TFStamped>::SharedPtr mocap_data_sub_;
    void mocapCallback( const TFStamped::ConstSharedPtr ) __attribute__((hot));

    rclcpp::TimerBase::SharedPtr tf_timer_;
    void timerTfCallback() __attribute__((hot));

    // /* Callback handler for asctec_onboard_data */
    // rclcpp::Subscription asctec_data_sub_;
    // void asctecDataCallback( const freyja_msgs::msg::AsctecData::ConstSharedPtr );
    
    // /* Callback handler for camera updates */
    // rclcpp::Subscription camera_estimate_sub_;
    // void cameraUpdatesCallback( const CameraOdom::ConstSharedPtr & );
    
    /* Callback handlers for mavros data */
    rclcpp::Subscription <std_msgs::msg::Float64>::SharedPtr compass_sub_;
    rclcpp::Subscription <nav_msgs::msg::Odometry>::SharedPtr mavros_gpsodom_sub_;
    rclcpp::Subscription <geometry_msgs::msg::Vector3>::SharedPtr mavros_rtk_sub_;
    void mavrosGpsOdomCallback( const nav_msgs::msg::Odometry::ConstSharedPtr );
    void mavrosCompassCallback( const std_msgs::msg::Float64::ConstSharedPtr );
    void mavrosRtkBaselineCallback( const geometry_msgs::msg::Vector3::ConstSharedPtr );
    
    /* handlers for locking map frame origins */
    inline void lockArmingGps( bool _lock = true )
    {
      arming_gps_pn_ = _lock? gps_odom_pn_ : 0.0;
      arming_gps_pe_ = _lock? gps_odom_pe_ : 0.0;
      arming_gps_pd_ = _lock? gps_odom_pd_ : 0.0;
    }
    inline void lockMapRTK( bool _lock = true )
    {
      map_rtk_pn_ = _lock && use_rtkbaseframe_? rtk_baseoffset_pn_ : 0.0;
      map_rtk_pe_ = _lock && use_rtkbaseframe_? rtk_baseoffset_pe_ : 0.0;
      map_rtk_pd_ = _lock && use_rtkbaseframe_? rtk_baseoffset_pd_ : 0.0;
    }
    
    rclcpp::Service<BoolServ>::SharedPtr maplock_srv_;
    void maplockArmingHandler( const BoolServ::Request::SharedPtr,
                               const BoolServ::Response::SharedPtr );

    
    /* Publisher for state information */
    rclcpp::Publisher<CurrentState>::SharedPtr state_pub_;
    

};
#endif
