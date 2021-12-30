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

	// HOVER AT A POINT
	TrajRef getHoverReference( const  rclcpp::Time &cur_time )
	{
	  TrajRef ref_state;
	  ref_state.pn = 0.0;
	  ref_state.pe = 0.0;
	  ref_state.pd = -0.75;
	  ref_state.vn = 0.0;
	  ref_state.ve = 0.0;
	  ref_state.vd = 0.0;
	  ref_state.yaw = DEG2RAD(0.0);
	  ref_state.an = 0.0;
	  ref_state.ae = 0.0;
	  ref_state.ad = 0.0;
	  ref_state.header.stamp = now();
	  //ref_state.header.stamp = this->now();
	  //ref_state.header.stamp = this->get_clock().now().to_msg();
	  return ref_state;
	}

	// CIRCLE: pn = A*sin(wt), pe = A*cos(wt), vn = A*w*cos(wt) ..
	TrajRef getCircleReference( const rclcpp::Time &cur_time, const int agg_level)
	{
	  // A is amplitude (radius); w angular rate such that 2pi/w = (seconds for one rev)
	  float A = 0.5;
	  float w = 0.5;

	  // Set A and w based on agg_level
	  switch(agg_level)
		{
	    case 1 :
	      break;
	    case 2 :
	      A = 0.5;
	      w = 1;
	      break;
	    case 3 :
	      A = 1;
	      w = 3;
	      break;
	    default :
	      RCLCPP_WARN("Circle aggression " << agg_level << " not supported, defaulting to agg_level 1");
		}

		float t = cur_time.seconds();

  	// Create reference state
  	TrajRef ref_state;
  	ref_state.header.stamp = now();

  	ref_state.pn = A*std::sin( w*t );
  	ref_state.pe = A*std::cos( w*t );
  	ref_state.pd = -4.0;

  	ref_state.vn = A*w*std::cos( w*t );
  	ref_state.ve = -A*w*std::sin( w*t );
  	ref_state.vd = 0.0;

  	ref_state.yaw = 0.0;

  	// set an, ae, ad to second derivatives if needed for FF..
  	return ref_state;

	}

	TrajRef getDefaultReference( const ros::Duration &cur_time )
	{
	  TrajRef ref_state;
	  ref_state.pn = 0.5;
	  ref_state.pe = 0.5;
	  ref_state.pd = -1.0;
	  ref_state.vn = 0.0;
	  ref_state.ve = 0.0;
	  ref_state.vd = 0.0;
	  ref_state.yaw = DEG2RAD(0.0);
	  ref_state.an = 0.0;
	  ref_state.ae = 0.0;
	  ref_state.ad = 0.0;
	  ref_state.header.stamp = now();

		return ref_state;
	}

	/** LEMISCATE OF BERNOULLI / Figure-8:
	* pn = A*cos(wt)/(1+sin^2(wt))
	* pe = A*sin(wt*)cos(wt)/(1+sin^2(wt))
	* vn = {A*w*sin(wt)(sin^2(wt)−3)}/{(sin^2(wt)+1)^2}
	* ve = {2*A*w*(3*cos(2*wt)−1)}/{(cos(2*wt)−3)^2}
	*/
	TrajRef getLemmiscateReference( const rclcpp::Time &cur_time, const int agg_level)
	{
	  // A is amplitude (radius); w angular rate such that 2pi/w = (seconds for one rev)
	  float A = 0.5;
	  float w = 0.5;

	  // Set A and w based on agg_level
	  switch(agg_level)
		{
	    case 1 :
	      break;
	    case 2 :
	      A = 0.5;
	      w = 1;
	      break;
	    case 3 :
	      A = 1;
	      w = 3;
	      break;
	    default :
	      RCLCPP_WARN("Circle aggression " << agg_level << " not supported, defaulting to agg_level 1");
		}

		float t = cur_time.seconds();

		// Create reference state
  	TrajRef ref_state;
  	ref_state.header.stamp = now();

  	ref_state.pn = A * std::cos(w*t)/(1+pow(std::sin(w*t),2));
  	ref_state.pe = A * std::sin(w*t)*std::cos(w*t)/(1+pow(std::sin(w*t),2));
  	ref_state.pd = -4.0;

  	ref_state.vn = ( A * w * std::sin(w*t)*( pow(std::sin(w*t),2) − 3)) / ( pow( pow( std::sin(w*t),2)+1 ,2));
  	ref_state.ve = ( 2 * A * w * (3*std::cos(2*wt)−1)) / ( pow( std::cos(2*wt)−3 , 2) )
  	ref_state.vd = 0.0;

  	ref_state.yaw = 0.0;

  	// set an, ae, ad to second derivatives if needed for FF..
  	return ref_state;

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

