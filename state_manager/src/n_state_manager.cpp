#include <iostream>
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

#include <eigen3/Eigen/Dense>

#include <freyja_utils/filters/filters.hpp>
#include <freyja_utils/fastmath/fast_approximate_math.hpp>

using TFStamped     = geometry_msgs::msg::TransformStamped;
using CurrentState  = freyja_msgs::msg::CurrentState;

class NStateManager : public rclcpp::Node
{
  int n_objects_;
  std::vector<bool> tf_avail_;
  std::vector<double> yaw_;

  std::string tf_base_frame_;
  std::vector<std::string> tf_objects_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<std::shared_ptr<freyja_utils::Filter>> state_filters_;

  std::vector<rclcpp::Publisher<CurrentState>::SharedPtr> state_pubs_;

  rclcpp::TimerBase::SharedPtr tf_listen_timer_;
  rclcpp::TimerBase::SharedPtr tf_pubs_timer_;
  void timerTfCallback() __attribute__((hot));
  void timerTFPublishers() __attribute__((hot));
  rclcpp::CallbackGroup::SharedPtr cb_grp1_, cb_grp2_;
  void initTfManager();
  public:
    NStateManager();
};


NStateManager::NStateManager() : Node("NStateManagerNode")
{
  declare_parameter<int>( "tf_rate", 180 );
  declare_parameter<int>( "tf_buffer_time", 1 );
  declare_parameter<std::string>( "tf_baseframe", "map" );
  declare_parameter<std::vector<std::string>>( "tf_objects", std::vector<std::string>({"a_1"}) );
  declare_parameter<std::vector<double>>( "kf_params", std::vector<double>({0.00005, 5.5}) );
  initTfManager();

  tf_avail_.resize(n_objects_);
  yaw_.resize(n_objects_);
  state_filters_.resize(n_objects_);
  state_pubs_.resize(n_objects_);

  std::vector<double> kparams;
  get_parameter( "kf_params", kparams );
  for(int oidx=0; oidx < n_objects_; oidx++)
  {
    std::string oname = tf_objects_[oidx];
    state_filters_[oidx] = std::make_shared<freyja_utils::KalmanFilter>(200, kparams);
    state_pubs_[oidx] = create_publisher <CurrentState> ( "/"+oname+"/current_state", 1 ); 
  }

  cb_grp1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_grp2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  int tf_lookup_rate;
  get_parameter( "tf_rate", tf_lookup_rate );
  // create fixed-rate timer
  tf_listen_timer_  = rclcpp::create_timer(
                                            #ifdef ROSVER_FOXY_OR_GALAC
                                              this, get_clock(),
                                            #endif
                                            std::chrono::duration<float>(1.0/tf_lookup_rate),
                                            std::bind(&NStateManager::timerTfCallback, this),
                                            cb_grp1_ );
  // create fixed-rate timer
  tf_pubs_timer_    = rclcpp::create_timer( 
                                            #ifdef ROSVER_FOXY_OR_GALAC
                                              this, get_clock(),
                                            #endif
                                            std::chrono::duration<float>(1.0/100.0),
                                            std::bind(&NStateManager::timerTFPublishers, this),
                                            cb_grp2_ );
}

void NStateManager::initTfManager()
{  
  int tf_buffer_time;
  float tf_timer_freq;
  auto tf_qos = rclcpp::QoS(2).best_effort().durability_volatile();
  
  
  get_parameter( "tf_buffer_time", tf_buffer_time );
  get_parameter( "tf_baseframe", tf_base_frame_ );
  get_parameter( "tf_objects", tf_objects_ );

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>( get_clock(), std::chrono::seconds(tf_buffer_time) );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_, rclcpp::Node::make_shared("_"), true, tf_qos );

  n_objects_ = tf_objects_.size();

  std::ostringstream s;
  s << "Tracking following " << n_objects_ << " objects in TF tree:\n";
  s << "\t\t\";
  for(const auto &o : tf_objects_)
    s << o << " ";
  s << std::endl;
  RCLCPP_INFO( get_logger(), "%s", s.str().c_str() );
}

void NStateManager::timerTfCallback()
{
  /* This is a timer callback */
  static tf2::Quaternion q;
  static double r, p, y;
  static TFStamped tform;
  static std::string err_msg;
  static Eigen::Matrix<double, 3, 1> meas_z_;
 
  for(int oidx=0; oidx < n_objects_; oidx++)
  {
    if( tf_buffer_->canTransform( tf_objects_[oidx], tf_base_frame_, tf2::TimePointZero, tf2::Duration(0), &err_msg ) )
    {
      tform = tf_buffer_->lookupTransform( tf_base_frame_, tf_objects_[oidx], tf2::TimePointZero );
      meas_z_ <<  tform.transform.translation.y,
                  tform.transform.translation.x,
                  -tform.transform.translation.z;        // setting this order is currently important
      state_filters_[oidx]->setMeasurementInput(meas_z_);

      tf2::convert( tform.transform.rotation, q );
      tf2::impl::getEulerYPR( q, y, p, r );
      y = freyja_utils::fast_approx::pi/2.0 - y;         // converting ENU yaw to NED yaw is manual
      yaw_[oidx] = y;
      tf_avail_[oidx] = true;
    }
    else
      RCLCPP_WARN_THROTTLE( get_logger(), *(get_clock()), 500, "Freyja: TF warn: %s", err_msg.c_str() );
  }
}

void NStateManager::timerTFPublishers()
{
  /* This is a timer callback */
  static CurrentState cs_msg;
  static Eigen::VectorXd best_estimate;
  for(int oidx=0; oidx < n_objects_; oidx++)
  {
    if( tf_avail_[oidx] )
    {
      state_filters_[oidx]->getStateEstimate( best_estimate, 9 );
      std::copy(best_estimate.data(), best_estimate.data()+6, cs_msg.state_vector.begin());
      cs_msg.state_vector[8] = yaw_[oidx];
      cs_msg.state_vector[9] = best_estimate.coeff(6);
      cs_msg.state_vector[10] = best_estimate.coeff(7);
      cs_msg.state_vector[11] = best_estimate.coeff(8);

      cs_msg.state_valid = true;
      state_pubs_[oidx] -> publish(cs_msg);
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init( argc, argv );
  auto nstate_mgr_node = std::make_shared<NStateManager>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(nstate_mgr_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
