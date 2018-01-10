/*
aj: new serial comm
*/

#include "lib_nimbus_serial.h"
//#include "aj_packet_format.h"

#include <thread>
#include <deque>
#include <mutex>
#include <chrono>


#define MAX_BUFFER_LEN 1000
#define BUFFER_OVF_DROP 100
#define READ_MAX 100

class AjSerialInterface
{
  std::vector <uint8_t> recv_buffer_;

protected:
  std::vector <uint8_t> running_buffer_;
  std::vector <uint8_t> write_buffer_;
  
  std::thread reader_thread_;
  std::mutex running_buffer_mutex_;
  
  /* Port characteristics */
  int32_t serial_fd_;
  int32_t serial_fd_write_;
  
  std::string serial_port_name_;
  std::string serial_port_name_write_;
  int32_t baudrate_;
  
  /* Derived classes can explicitly tell read methods to stop */
  volatile bool keep_alive_;
  
  /* How fast should the reader thread run (ms) */
  uint16_t time_between_reads_;
  
public:
    AjSerialInterface();
    AjSerialInterface( const std::string &port, const int &rate );
    ~AjSerialInterface();
    
    void writeCommandPacket();
    void readPacket();
    void readThread();
    
    /* Provide thread-safe accessing and modifications */
    int getPossiblePacket( std::vector<uint8_t> &buf, int len );
    
    inline void removeFrontElement()
    {
      /* Removing the first element because someone requested so.
        I'm not entirely sure if STL containers are thread-safe in an operation
        at different ends, so I'm going to close this inside mutex access anyway.
      */
      //std::lock_guard <std::mutex> rbmutex( running_buffer_mutex_ );
      running_buffer_.erase( running_buffer_.begin() );
    }
    
    inline void removeOnePacketLen( const uint8_t &len )
    {
      /* Remove one packet length worth of elements from buffer. Most likely
        because a packet was matched
      */
      //std::lock_guard <std::mutex> rbmutex ( running_buffer_mutex_ );
      running_buffer_.erase( running_buffer_.begin(), running_buffer_.begin() + len );
    }
    
    uint8_t showFrontElement( int idx )
    {
      return (running_buffer_.size() > 0) ? running_buffer_[ idx ] : 0;
    }
    
};
