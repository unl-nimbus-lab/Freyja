/* Provides a trajectory for the vehicle to follow.
  EXAMPLE FILE; ONLY FOR SUPPORT.
  Whatever you do here, output a time-based continuous function to follow.
  This node should generate a 7 vector: [pn pe pd vn ve vd yaw]' for the vehicle
  to follow. The controller currently listens to this reference trajectory
  and updates its knowledge of the "latest" reference point.
  
  -- aj / 23rd Nov, 2017.
*/

#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "freyja_msgs/msg/reference_state.hpp"

typedef freyja_msgs::msg::ReferenceState TrajRef;

#define DEG2RAD(D) ((D)*3.1415326/180.0)

using std::placeholders::_1;
using namespace std::chrono_literals;

rclcpp::Time init_time;
std::string traj_type;
int agg_level;

class Temporal_Traj : public rclcpp::Node
{
	TrajRef ref_state;
	rclcpp::Time init_time;

	std::string traj_type;
	int agg_level;

	public:
		Temporal_Traj();

		rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr time_reset_sub_;
		void timer_reset_cb( const std_msgs::msg::UInt8::SharedPtr msg );

		rclcpp::Publisher<TrajRef>::SharedPtr traj_pub_;
		
		rclcpp::TimerBase::SharedPtr traj_update_timer_;
		void traj_update_cb();

		TrajRef getHoverReference( rclcpp::Duration cur_time );
		TrajRef getCircleReference( rclcpp::Duration cur_time, const int agg_level);
		TrajRef getDefaultReference( rclcpp::Duration cur_time );
		TrajRef getLemiscateReference( rclcpp::Duration cur_time, const int agg_level);

};

