/**
* Outputs a time-based continuous function to follow.
* Generates a reference vector: '[pn pe pd vn ve vd yaw]' for the vehicle to follow
*/

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "freyja_msgs/msg/ReferenceState.hpp"

#define ROS_NODE_NAME "trajectory_provider"
#define UPDATE_RATE 100															// update rate of the trajectory (Hz)
typedef freyja_msgs::ReferenceState TrajRef;

#define DEG2RAD(D) ((D)*3.1415326/180.0)

using std::placeholders::_1;

rclcpp::Time init_time;

class Temporal_Traj : public rclcpp::Node
{
	public:
		TrajRef()
		: Node(ROS_NODE_NAME)
		{
			this->declare_parameter("traj_type", traj_type);
			this->get_parameter("traj_type", traj_type);

			this->declare_parameter("agg_level", agg_level);
  		this->get_parameter("agg_level", agg_level);

			traj_pub = this->create_publisher<TrajRef>("/reference_state", 1);

			time_reset_sub = this->create_subscription<std_msgs::msg::UInt8>(
			"/reset_trajectory_time", 1, std::bind(&Temporal_Traj::timer_reset_cb, this, _1));

			traj_update_timer = this->create_wall_timer(
      1/UPDATE_RATE, std::bind(&SbusComm::traj_update_cb, this));

		}

	void timer_reset_cb( const std_msgs::msg::UInt8::SharedPtr msg )
	{
		if (msg -> data == 1)
		{
			init_time = now();
			RCLCPP_WARN(this->get_logger(), "%s: Time reset requested!", ROS_NODE_NAME);
		}
	}

	void traj_update_cb()
	{
		if (traj_type == "hover")
		{
			ref_state = getHoverReference(now() - init_time);
		}
		else if (traj_type == "circle")
		{
			ref_state = getCircleReference(now() - init_time, agg_level);
		}
		else if (traj_type == "lemiscate")
		{
			ref_state = getLemiscateReference(now() - init_time, agg_level);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Invalid trajectory parameters given.")
		}

		traj_pub->publish(ref_state);
	}

	rclcpp::Publisher<TrajRef>::SharedPtr traj_pub;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr time_reset_sub;
	rclcpp::TimerBase::SharedPtr traj_update_timer;
};

int main(int argc, char * argv[])
{
	init_time = now();

	rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Temporal_Traj>());  
  rclcpp::shutdown();
  return 0;

}

