/* Node to handle an AscTec vehicle. This is the equivalent of motor_ctrl
from nimbus_asctec. It handles what to send to the serial comms - for example,
turning on/off motors is the responsibitily of this guy.

I'm structuring it this way: this node consumes motor and other commands that I
have not thought of yet, and then outputs one large message for the serial 
interface to listen to. This should contain anything and everything that it needs
to send out, and the serial node itself should be unaware of asctec.
Note that the serial interface does know about asctec in order to construct the 
returned packet.

Screw it, this node handles serial interface as well!
*/

#include <signal.h>
#include <thread>
#include <ros/ros.h>
#include <common_msgs/CtrlCommand.h>
#include <common_msgs/CurrentState.h>

#include <common_msgs/MotorCommand.h>
#include <common_msgs/AsctecData.h>

#include "aj_serial_interface.h"
#include "aj_packet_format.h"

enum ASCTEC_VEHICLE_STATE
{
  VEHICLE_OFF,
  VEHICLE_ON,
  VEHICLE_MOTORS_OFF,
  VEHICLE_MOTORS_ON,
  VEHICLE_IDLING,
  VEHICLE_FLYING
};

class AsctecHandler : public AjSerialInterface
{
  ros::NodeHandle nh_;
  
  int vehicle_state_;
  common_msgs::AsctecData vehicle_data_;
  
  public:
    AsctecHandler( const std::string&, const int );
    ~AsctecHandler();
    static void fake_destructor();
    static void signalHandler( int );
    static volatile bool shutdown_requested_;
    
    void limitLqrCommands( float&, float&, float&, float& );
    
    ros::Publisher asctec_command_pub_;
    
    ros::Subscriber rpyt_cmd_sub_;
    void rpytCommandCallback( const common_msgs::CtrlCommand::ConstPtr & );
    
    ros::Subscriber motor_handler_sub_;
    void motorHandlerCallback( const common_msgs::MotorCommand::ConstPtr & );
    
    void turnMotorsOn();
    void turnMotorsOff();
    void motorsIdle();
    void sendCtrlCommands();
    void FormCmdPacketForLLP( int16_t, int16_t, int16_t, int16_t, uint8_t );
    
    /* Publisher for decoded data packets */
    ros::Publisher asctec_pub_;
    /* Function to decode packets based on the packet structure */
    void decodePacket( std::vector<uint8_t> & );
    
    /* Periodically try to read and call decode function */
    ros::WallTimer read_decode_timer_;
    void readAndDecodePackets( const ros::WallTimerEvent& );
    
    std::thread pkt_decode_thread_;
    void readAndDecodePacketsThreaded();
    
};

