#include "asctec_handler.h"

#define ROS_NODE_NAME "asctec_handler"
AsctecHandler::AsctecHandler( const std::string &p, const int b )
                    : AjSerialInterface( p, b ), nh_()
{
  signal( SIGINT, AsctecHandler::signalHandler );
  
  vehicle_state_ = VEHICLE_OFF;
  
  /* Set up callbacks */
  std::string command_topic( "/rpyt_command" );
  nh_.param( "control_topic", command_topic, command_topic );
  rpyt_cmd_sub_ = nh_.subscribe( command_topic, 1,
                                 &AsctecHandler::rpytCommandCallback, this );
                                 
  std::string motor_topic( "/motors_event" );
  nh_.param( "motors_topic", motor_topic, motor_topic );
  motor_handler_sub_ = nh_.subscribe( motor_topic, 1,
                                 &AsctecHandler::motorHandlerCallback, this);
                                 
                                 
  /* Serial initialization stuff */
  write_buffer_.clear();
  write_buffer_.resize( PRY_PKT_LEN );
  
  AjSerialInterface::keep_alive_ = false;                          
}

AsctecHandler::~AsctecHandler()
{
  AjSerialInterface::keep_alive_ = false;
  ROS_INFO( "Asctec Handler shutting down!" );
}
void AsctecHandler::signalHandler( int sig )
{
  shutdown_requested_ = true;
  fake_destructor();
}

void AsctecHandler::fake_destructor()
{
  //AjSerialInterface::keep_alive_ = false;
  ROS_INFO( "AsctecHandler shutting down [fake_destructor]!" );
  ros::shutdown();
}

void AsctecHandler::limitLqrCommands( float &r, float &p, float &y, float &t )
{
  /* Remap thrust so that [+5 .. -5] is  [+1 .. -1] */
  t /= 5.0;
}

void AsctecHandler::rpytCommandCallback( const lqr_control::CtrlCommand::ConstPtr &msg )
{
  /* handle the message and reshape it for asctec vehicles and then publish it
      to a topic that is then consumed by the serial interface node. */
      
  /* Some requested to write rpyt commands to the vehicle. We need to convert
    this to a stream (after checking for validity) and the write to serial.
    The packet is being sent to HLP, so note the following:
      pitch,roll,yaw  = -2047 .. +2047
      thrust          = 0 .. 4095
      ctrl            = 6 bits enabling: [x x gps height thrust yaw roll pitch]
    Expect pitch, roll, yaw and thrust to be in [-1..1] doubles.
  */
  
  /* Force-rescale thrust beforehand to -1 .. +1 */
  float f_roll = msg -> roll;
  float f_pitch = msg -> pitch;
  float f_thrust = msg -> thrust;
  float f_yaw = msg -> yaw;
  limitLqrCommands( f_roll, f_pitch, f_yaw, f_thrust );
  
  /* Convert to -2048 .. 2048 */
  int16_t pitch = static_cast<int16_t> ( 2048 * f_pitch );
  int16_t roll = static_cast<int16_t> ( 2048 * f_roll );
  int16_t yaw = static_cast<int16_t> ( 2048 * f_yaw );
  int16_t thrust = static_cast<int16_t> ( 2048 * (1+f_thrust) );
  uint8_t ctrl_bits = msg->ctrl_mode;
  
  /* clip to hard limits */
  pitch = std::max( PITCH_MIN, std::min( PITCH_MAX, pitch ) );
  roll = std::max( ROLL_MIN, std::min( ROLL_MAX, roll ) );
  yaw = std::max( YAW_MIN, std::min( YAW_MAX, yaw ) );
  thrust = std::max( THRUST_MIN, std::min( THRUST_MAX, thrust ) );
  
  /* put into the write buffer */
  write_buffer_.clear();
  LibNimbusSerial::pack8( PRY_HEADER_VAL, PRY_HEADER_IDX, write_buffer_ );
  LibNimbusSerial::pack16( pitch, PRY_PITCH_LOW, write_buffer_ );
  LibNimbusSerial::pack16( roll, PRY_ROLL_LOW, write_buffer_ );
  LibNimbusSerial::pack16( yaw, PRY_YAW_LOW, write_buffer_ );
  LibNimbusSerial::pack16( thrust, PRY_THRUST_LOW, write_buffer_ );
  LibNimbusSerial::pack8( ctrl_bits, PRY_CTRL, write_buffer_ );
  
  uint16_t crc = LibNimbusSerial::calcCrc16( write_buffer_, PRY_CRC_HIGH );
  LibNimbusSerial::pack16( crc, PRY_CRC_LOW, write_buffer_ );
  
  ROS_ASSERT( write_buffer_.size() < PRY_PKT_LEN );
  ROS_WARN( "writing [rpyt]: %d, %d, %d, %d", roll, pitch, yaw, thrust );
  //writeCommandPacket();
}

void AsctecHandler::motorHandlerCallback( const asctec_handler::MotorCommand::ConstPtr &msg )
{
  /* check what is it that we want to do with the motors, and then send it to a
  topic that the serial interface node can then consume.
  */
  if( msg-> motors_state == 1 )
    turnMotorsOn();
  else if( msg -> motors_state == 0 )
    turnMotorsOff();
}

void AsctecHandler::turnMotorsOn()
{
  /* Prevent from turning motors on when they are on */
  if( vehicle_state_ != VEHICLE_MOTORS_ON )
  {
    ROS_WARN( "Turning motors on!" );
    write_buffer_.clear();
    write_buffer_.resize( MOTORS_ON_CMND_LEN );
    LibNimbusSerial::pack8( MOTORS_ON_HEADER_VAL, MOTORS_ON_HEADER_IDX, write_buffer_ );
  
    uint16_t crc = LibNimbusSerial::calcCrc16( write_buffer_, MOTORS_ON_CRC_HIGH );
    LibNimbusSerial::pack16( crc, MOTORS_ON_CRC_LOW, write_buffer_ );
  
    writeCommandPacket();
    vehicle_state_ = VEHICLE_MOTORS_ON;
    write_buffer_.clear();
  }
}

void AsctecHandler::turnMotorsOff()
{
  /* Only turn off if the motors are on */
  if( vehicle_state_ >= VEHICLE_MOTORS_ON )
  {
    ROS_WARN( "Turning motors off!" );
    write_buffer_.clear();
    write_buffer_.resize( MOTORS_OFF_CMND_LEN );
    LibNimbusSerial::pack8( MOTORS_OFF_HEADER_VAL, MOTORS_OFF_HEADER_IDX, write_buffer_ );
  
    uint16_t crc = LibNimbusSerial::calcCrc16( write_buffer_, MOTORS_OFF_CRC_HIGH );
    LibNimbusSerial::pack16( crc, MOTORS_OFF_CRC_LOW, write_buffer_ );
  
    writeCommandPacket();
    vehicle_state_ = VEHICLE_MOTORS_OFF;
    write_buffer_.clear();
  }
}

void AsctecHandler::sendCtrlCommands()
{

}

volatile bool AsctecHandler::shutdown_requested_ = false;
int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME, ros::init_options::NoSigintHandler );
  AsctecHandler ahandler( "/dev/ttyUSB0", 57600 );
  
  ros::spin();
  return 0;
}
