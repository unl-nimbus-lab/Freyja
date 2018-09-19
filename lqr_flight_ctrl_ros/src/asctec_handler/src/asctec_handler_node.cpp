#include "asctec_handler.h"

#define ROS_NODE_NAME "asctec_handler"
#define __USING_DUAL_RADIO 0

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

  asctec_pub_ = nh_.advertise <common_msgs::AsctecData>
                                          ( "/asctec_onboard_data", 1, true );
  read_decode_timer_ = nh_.createWallTimer( ros::WallDuration(0.0008),
                            &AsctecHandler::readAndDecodePackets, this );
  //pkt_decode_thread_ = std::thread( &AsctecHandler::readAndDecodePacketsThreaded, this );

  AjSerialInterface::keep_alive_ = true;
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
  //pkt_decode_thread_.join();
  ros::shutdown();
}

void AsctecHandler::limitLqrCommands( float &r, float &p, float &y, float &t )
{
  /* Remap thrust so that [+5 .. -5] is  [+1 .. -1] */
  t = t - 6.0;
  t /= 5.0;
}

void AsctecHandler::rpytCommandCallback( const common_msgs::CtrlCommand::ConstPtr &msg )
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
  
  //ROS_WARN( "floats: %0.3f, %0.3f, %0.3f, %0.3f", f_roll, f_pitch, f_yaw, f_thrust );
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
  
  #if __USING_DUAL_RADIO
    /* put into the write buffer */
    write_buffer_.clear();
    write_buffer_.resize(17);
    FormCmdPacketForLLP( pitch, roll, yaw, thrust, ctrl_bits );
  #else
    write_buffer_.resize( PRY_PKT_LEN );
    LibNimbusSerial::pack8( PRY_HEADER_VAL, PRY_HEADER_IDX, write_buffer_ );
    LibNimbusSerial::pack16( pitch, PRY_PITCH_LOW, write_buffer_ );
    LibNimbusSerial::pack16( roll, PRY_ROLL_LOW, write_buffer_ );
    LibNimbusSerial::pack16( yaw, PRY_YAW_LOW, write_buffer_ );
    LibNimbusSerial::pack16( thrust, PRY_THRUST_LOW, write_buffer_ );
    LibNimbusSerial::pack8( ctrl_bits, PRY_CTRL, write_buffer_ );
  
    uint16_t crc = LibNimbusSerial::calcCrc16( write_buffer_, PRY_CRC_HIGH );
    LibNimbusSerial::pack16( crc, PRY_CRC_LOW, write_buffer_ );
  #endif
  //ROS_ASSERT( write_buffer_.size() < PRY_PKT_LEN );
  //ROS_WARN( "writing [rpyt]: %d, %d, %d, %d", roll, pitch, yaw, thrust );
  
  writeCommandPacket();
  
}
void AsctecHandler::FormCmdPacketForLLP( int16_t pitch, int16_t roll, int16_t yaw, int16_t thrust, uint8_t ctrl_bits )
{
  /* { '>', '*', '>', 'd', 'i' }
  Note: asctec words are reversed, low byte first then high byte.
  <sarcasm> Thanks John-Paul. </sarcasm>
  */
  write_buffer_[0] = static_cast<uint8_t>( '>' );
  write_buffer_[1] = static_cast<uint8_t>( '*' );
  write_buffer_[2] = static_cast<uint8_t>( '>' );
  write_buffer_[3] = static_cast<uint8_t>( 'd' );
  write_buffer_[4] = static_cast<uint8_t>( 'i' );

  LibNimbusSerial::packReversed16( pitch, 6, write_buffer_ );
  LibNimbusSerial::packReversed16( roll, 8, write_buffer_ );
  LibNimbusSerial::packReversed16( yaw, 10, write_buffer_ );
  LibNimbusSerial::packReversed16( thrust, 12, write_buffer_ );

  int16_t ctrl_bits_16 = static_cast<int16_t>( ctrl_bits );
  LibNimbusSerial::packReversed16( ctrl_bits_16, 14, write_buffer_ );

  int16_t crc = pitch + roll + yaw + thrust + ctrl_bits_16 + 0xAAAA;
  LibNimbusSerial::packReversed16( crc, 16, write_buffer_ );
}


void AsctecHandler::motorHandlerCallback( const common_msgs::MotorCommand::ConstPtr &msg )
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
    
    //ros::Duration(0.6).sleep();
    /* Write idle packet now */
    //motorsIdle();
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
    
    //ros::Duration(0.6).sleep();
    //motorsIdle();
  }
}

void AsctecHandler::motorsIdle()
{
  if( vehicle_state_ >= VEHICLE_MOTORS_ON )
  {
    write_buffer_.clear();
    write_buffer_.resize( MOTORS_IDLE_CMND_LEN );
    LibNimbusSerial::pack8( MOTORS_IDLE_HEADER_VAL, MOTORS_IDLE_HEADER_IDX, write_buffer_ );
    
    uint16_t crc = LibNimbusSerial::calcCrc16( write_buffer_, MOTORS_IDLE_CRC_HIGH );
    LibNimbusSerial::pack16( crc, MOTORS_IDLE_CRC_LOW, write_buffer_ );
    writeCommandPacket();
    vehicle_state_ = VEHICLE_IDLING;
    write_buffer_.clear();
  }
}

void AsctecHandler::sendCtrlCommands()
{

}

void AsctecHandler::readAndDecodePackets( const ros::WallTimerEvent& event )
{
  /* This runs continouosly on a ROS-timer and inspects the data buffer to try
    and construct a packet out of the front-slice (first PKT_LEN elements). If
    it isn't able to construct a packet then it has to handle the following 
    weird cases:
      1. incorrect header: remove the first element in the buffer
      2. incorrect crc   : remove the first element in the buffer
      3. correct packet  : remove the packet
   */
   double err_now = (event.current_real - event.current_expected).toSec();
   double err_btw = (event.current_real - event.last_real).toSec();
   
   //ROS_INFO( "Cur expectation diff: %0.4f\tTime since last: %0.4f", err_now, err_btw );

  /* get PKT_LEN elements into a local buffer */
  std::vector <uint8_t> local_buffer;
  if( AjSerialInterface::getPossiblePacket( local_buffer, int(PKT_LEN) ) )
  {
    /* check if there is a matching header */
    if( local_buffer.front() == PKT_HEADER_FLAG )
    {
      /* aha! A possible packet, my lord! 
          ..which means it should have a valid checksum, right? */
      uint16_t calc_crc = LibNimbusSerial::calcCrc16( local_buffer, PKT_CHKSUM_IDX_1 );
      uint16_t buf_crc = LibNimbusSerial::unpack16( local_buffer, PKT_CHKSUM_IDX_0 );

      if( calc_crc == buf_crc )
      {
        /* Awesome, we have a totally valid packet now! Let's decode it! */
        decodePacket( local_buffer );
        /* Remove it from the running buffer */
        removeOnePacketLen( PKT_LEN );
      }
      else
      {
        /* crc does not match, which means false alarm. Need to remove front. */
        //AjSerialInterface::removeFrontElement();
        removeOnePacketLen( PKT_LEN );
        ROS_WARN( "crc fail!" );
      }
    }
    else
    {
      /* Not a valid header. Remove front element. */
      //ROS_ERROR( "not header!" );
      removeFrontElement();
    }
  }
}

void AsctecHandler::readAndDecodePacketsThreaded()
{
  /* Thread variant of reading and decoding packets because the other one seemed
  to be dropping a lot of packets for "no reason"
  */
  ROS_WARN( "starting thread" );
  while( AjSerialInterface::showFrontElement(0) != PKT_HEADER_FLAG )
        removeFrontElement();
  while( ros::ok() )
  {
    std::vector <uint8_t> local_buffer;
    
    // maybe a packet.. request that number of bytes
    if( AjSerialInterface::getPossiblePacket( local_buffer, int(PKT_LEN) ) )
    { 
      //    ..which means it should have a valid checksum, right?
      uint16_t calc_crc = LibNimbusSerial::calcCrc16( local_buffer, PKT_CHKSUM_IDX_1 );
      uint16_t buf_crc = LibNimbusSerial::unpack16( local_buffer, PKT_CHKSUM_IDX_0 );
      if( calc_crc == buf_crc )
      {
        decodePacket( local_buffer );
        removeOnePacketLen( PKT_LEN );
      }
      else
      {
        int useless_bytes = 0;
        while( AjSerialInterface::showFrontElement(useless_bytes++) != PKT_HEADER_FLAG );
        removeOnePacketLen( useless_bytes-1 );
      }
    }
    //ros::Duration(0.001).sleep();
  }
}
// PKT_TYPE is defined in aj_packet_format.h that is included for this class.
#if PKT_TYPE == 5
void AsctecHandler::decodePacket( std::vector<uint8_t> &buffer )
{
  /* Arrived here because checksum and header matched. Now we need to 
    sequentially unroll the packet contents and add stuff to the class's local
    variables. Once packet has been unpacked, call publisher.
  */
  vehicle_data_.lat = LibNimbusSerial::unpack32( buffer, PKT_LAT_0_IDX );
  vehicle_data_.lon = LibNimbusSerial::unpack32( buffer, PKT_LON_0_IDX );
  vehicle_data_.hgt = LibNimbusSerial::unpack32( buffer, PKT_FUSION_HEIGHT_0_IDX );
  
  vehicle_data_.sp_x = LibNimbusSerial::unpack32( buffer, PKT_SPEED_X_0_IDX );
  vehicle_data_.sp_y = LibNimbusSerial::unpack32( buffer, PKT_SPEED_Y_0_IDX );
  
  vehicle_data_.best_sp_x = LibNimbusSerial::unpack16( buffer, PKT_BEST_SPEED_X_0_IDX );
  vehicle_data_.best_sp_y = LibNimbusSerial::unpack16( buffer, PKT_BEST_SPEED_Y_0_IDX );
  
  vehicle_data_.heading_angle = LibNimbusSerial::unpack16( buffer, PKT_HEADING_0_IDX );
  
  vehicle_data_.header.stamp = ros::Time::now();
  asctec_pub_.publish( vehicle_data_ );
}
#elif PKT_TYPE == 6
void AsctecHandler::decodePacket( std::vector<uint8_t> &buffer )
{
  /* Arrived here because checksum and header matched. Unpack the packet into 
    respective fields and call publisher.
    Note: this function overload is for the larger packet.
  */
  vehicle_data_.lat = LibNimbusSerial::unpack32( buffer, LAT_0_IDX );
  vehicle_data_.lon = LibNimbusSerial::unpack32( buffer, LON_0_IDX );
  vehicle_data_.best_lat = LibNimbusSerial::unpack32( buffer, BEST_LAT_0_IDX );
  vehicle_data_.best_lon = LibNimbusSerial::unpack32( buffer, BEST_LON_0_IDX );
  vehicle_data_.hgt = LibNimbusSerial::unpack32( buffer, FUSION_HEIGHT_0_IDX );
  
  vehicle_data_.sp_x = LibNimbusSerial::unpack32( buffer, GPSSPEED_X_0_IDX );
  vehicle_data_.sp_y = LibNimbusSerial::unpack32( buffer, GPSSPEED_Y_0_IDX );
  vehicle_data_.best_sp_x = LibNimbusSerial::unpack16( buffer, BEST_SPEED_X_0_IDX );
  vehicle_data_.best_sp_y = LibNimbusSerial::unpack16( buffer, BEST_SPEED_Y_0_IDX );
  vehicle_data_.best_sp_z = LibNimbusSerial::unpack32( buffer, BEST_SPEED_Z_0_IDX );
  
  vehicle_data_.heading_angle = LibNimbusSerial::unpack32( buffer, GPS_HEADING_0_IDX );
  
  vehicle_data_.pitch_angle = LibNimbusSerial::unpack32( buffer, ANGLE_PITCH_O_IDX );
  vehicle_data_.roll_angle = LibNimbusSerial::unpack32( buffer, ANGLE_ROLL_0_IDX );
  vehicle_data_.yaw_angle = LibNimbusSerial::unpack32( buffer, ANGLE_YAW_0_IDX );
  
  vehicle_data_.motor1rpm = LibNimbusSerial::unpack8( buffer, RPM_M0_IDX );
  vehicle_data_.motor2rpm = LibNimbusSerial::unpack8( buffer, RPM_M1_IDX );
  vehicle_data_.motor3rpm = LibNimbusSerial::unpack8( buffer, RPM_M2_IDX );
  vehicle_data_.motor4rpm = LibNimbusSerial::unpack8( buffer, RPM_M3_IDX );
  vehicle_data_.motor5rpm = LibNimbusSerial::unpack8( buffer, RPM_M4_IDX );
  vehicle_data_.motor6rpm = LibNimbusSerial::unpack8( buffer, RPM_M5_IDX );
  vehicle_data_.header.stamp = ros::Time::now();
  asctec_pub_.publish( vehicle_data_ );
}
#endif
volatile bool AsctecHandler::shutdown_requested_ = false;
int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME, ros::init_options::NoSigintHandler );
  AsctecHandler ahandler( "/dev/ttyUSB0", 57600 );
  
  ros::spin();
  return 0;
}
