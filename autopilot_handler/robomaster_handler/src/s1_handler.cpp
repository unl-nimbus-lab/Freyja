#include "rclcpp/rclcpp.hpp"

#include "robomaster_interfaces/msg/wheel_speed.hpp"
#include <freyja_msgs/msg/wheel_command.hpp>

typedef freyja_msgs::msg::WheelCommand WheelCommand;          // rad/s from controller
typedef robomaster_interfaces::msg::WheelSpeed WheelSpeed;    // rpm or normalised to vehicle

using std::placeholders::_1;

#define OMEGA2RPM(W) ((W)*9.55)

class S1Handler : public rclcpp::Node
{
  WheelSpeed target_wheel_rpm_;

  public:
    S1Handler();

    rclcpp::Subscription<WheelCommand>::SharedPtr speed_cmd_sub_;
    void wheelSpeedCallback( const WheelCommand::ConstSharedPtr );

    rclcpp::Publisher<WheelSpeed>::SharedPtr speed_rpm_pub_;

};

S1Handler::S1Handler() : rclcpp::Node( "s1_handler" )
{
  // associate callback for controller's commands
  speed_cmd_sub_ = create_subscription<WheelCommand> ( "controller_command", 1,
                                std::bind(&S1Handler::wheelSpeedCallback, this, _1) );

  // declare publisher for sending out RPM commands
  speed_rpm_pub_ = create_publisher<WheelSpeed> ( "target_rpm", 1 );
}

void S1Handler::wheelSpeedCallback( const WheelCommand::ConstSharedPtr msg )
{
  /* The controller outputs target wheel speeds in rad/s. We must convert them into
    RPMs in the range [-1000, 1000] (int16_t) for DJI
  */
  target_wheel_rpm_.fl = std::round( OMEGA2RPM(msg -> w1) );
  target_wheel_rpm_.fr = std::round( OMEGA2RPM(msg -> w2) );
  target_wheel_rpm_.rr = std::round( OMEGA2RPM(msg -> w3) );
  target_wheel_rpm_.rl = std::round( OMEGA2RPM(msg -> w4) );

  speed_rpm_pub_ -> publish( target_wheel_rpm_ );
}


int main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<S1Handler>() );
  rclcpp::shutdown();
  return 0;
}