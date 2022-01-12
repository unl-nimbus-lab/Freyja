/* Provides a trajectory for the vehicle to follow.

  EXAMPLE FILE; ONLY FOR SUPPORT.

  Whatever you do here, output a time-based continuous function to follow.
  This node should generate a 7 vector: [pn pe pd vn ve vd yaw]' for the vehicle
  to follow. The controller currently listens to this reference trajectory
  and updates its knowledge of the "latest" reference point.
  
  -- aj / 23rd Nov, 2017.
*/

#include "temporal_provider.h"

#define ROS_NODE_NAME "trajectory_provider"
#define UPDATE_RATE 50														// update rate of the trajectory (Hz)

Temporal_Traj::Temporal_Traj() : Node( ROS_NODE_NAME )
{
	init_time = now();

	declare_parameter<std::string>("traj_type", traj_type);
	declare_parameter<int>("agg_level", agg_level);

	get_parameter("traj_type", traj_type);
	get_parameter("agg_level", agg_level);

	time_reset_sub_ = create_subscription<std_msgs::msg::UInt8>(
	"/reset_trajectory_time", 1, std::bind(&Temporal_Traj::timer_reset_cb, this, _1));

	traj_pub_ = create_publisher<TrajRef>("/reference_state", 1);

	traj_update_timer_ = create_wall_timer(
	20ms, std::bind(&Temporal_Traj::traj_update_cb, this));
}

void Temporal_Traj::timer_reset_cb( std_msgs::msg::UInt8::SharedPtr msg )
{
	if (msg->data == 1)
	{
		init_time = now();
		RCLCPP_WARN(get_logger(), "%s: Time reset requested!", ROS_NODE_NAME);
	}
}

void Temporal_Traj::traj_update_cb()
{
	get_parameter("traj_type", traj_type);

	if (traj_type == "hover")
	{
		ref_state = Temporal_Traj::getHoverReference(now() - init_time);
		RCLCPP_INFO(get_logger(), "Publishing Hover Reference States");
	}
	else if (traj_type == "circle")
	{
		ref_state = Temporal_Traj::getCircleReference(now() - init_time, agg_level);
		RCLCPP_INFO(get_logger(), "Publishing Circle Reference States");
	}
	else if (traj_type == "lemiscate")
	{
		ref_state = Temporal_Traj::getLemiscateReference(now() - init_time, agg_level);
		RCLCPP_INFO(get_logger(), "Publishing Lemiscate Reference States");
	}
	else
	{
		RCLCPP_ERROR(get_logger(), "Invalid trajectory parameters given.");
	}

	traj_pub_->publish(ref_state);
}

// HOVER AT A POINT
TrajRef Temporal_Traj::getHoverReference( rclcpp::Duration cur_time )
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
  return ref_state;
}

/** CIRCLE:
* pn = A*sin(wt)
* pe = A*cos(wt)
* vn = A*w*cos(wt)
* vn = -A*w*sin(wt)
*/
TrajRef Temporal_Traj::getCircleReference( const rclcpp::Duration cur_time, const int agg_level)
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
      RCLCPP_WARN(get_logger(), "Circle aggression %d not supported, defaulting to agg_level 1", agg_level);
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

TrajRef Temporal_Traj::getDefaultReference( rclcpp::Duration cur_time )
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

{



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Temporal_Traj>());
  rclcpp::shutdown();
  return 0;
}

