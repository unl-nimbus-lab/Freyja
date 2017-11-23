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
#include <ros/ros.h>
#include <lqr_control/CtrlCommand.h>
#include <state_manager/CurrentState.h>

#include <asctec_handler/MotorCommand.h>
//#include <asctec_handler/AsctecCommand.h>

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
  
  public:
    AsctecHandler( const std::string&, const int );
    ~AsctecHandler();
    static void fake_destructor();
    static void signalHandler( int );
    static volatile bool shutdown_requested_;
    
    void limitLqrCommands( float&, float&, float&, float& );
    
    ros::Publisher asctec_command_pub_;
    
    ros::Subscriber rpyt_cmd_sub_;
    void rpytCommandCallback( const lqr_control::CtrlCommand::ConstPtr & );
    
    ros::Subscriber motor_handler_sub_;
    void motorHandlerCallback( const asctec_handler::MotorCommand::ConstPtr & );
    
    void turnMotorsOn();
    void turnMotorsOff();
    void sendCtrlCommands();
    
    /* Function to decode packets based on the packet structure */
    // void decodePacket( const std::vector<uint8_t> & );
    
    /* Periodically try to read and call decode function */
    // ros::WallTimer read_decode_timer_;
    //void readAndDecodePackets( const ros::WallTimerEvent& );
    
};

