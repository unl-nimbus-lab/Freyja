#include "aj_serial_interface.h"

AjSerialInterface::AjSerialInterface( const std::string &port, const int &rate )
{
  serial_port_name_ = port;
  baudrate_ = rate;
  int ret = LibNimbusSerial::initSerialInterface( port, rate, serial_fd_ );
  if( ret == 0 )
  { 
    ROS_INFO( "Serial interface up! Running on file: %d", serial_fd_ );
    
    /* Start a thread that constantly spins and reads and puts stuff into the
      read buffer. Need mutexes to write to the buffer, maybe.
    */
    reader_thread_ = std::thread( &AjSerialInterface::readThread, this );
    keep_alive_ = true;
    recv_buffer_.resize( READ_MAX );
    time_between_reads_ = 4;
  }
}

AjSerialInterface::~AjSerialInterface()
{
  if( reader_thread_.joinable() )
    reader_thread_.join();
    
  /* Close the serial interface, "gracefully" */
  if( LibNimbusSerial::closeSerialInterface( serial_fd_ ) == 0 )
    ROS_INFO( "Serial interface closed." );
  
}

void AjSerialInterface::readThread()
{
  /* Constantly spin and try reading stuff from the serial port. It will put
    the bytes into the class datastructure buffer.
  */
  while( keep_alive_ )
  {
    readPacket();
    std::this_thread::sleep_for( std::chrono::milliseconds(time_between_reads_) );
  }
}

void AjSerialInterface::writeCommandPacket()
{
  /* Write to serial port now */
  LibNimbusSerial::sendSerial( serial_fd_, write_buffer_ );
}

void AjSerialInterface::readPacket()
{
  int read_bytes;
  read_bytes = LibNimbusSerial::readSerial( serial_fd_, READ_MAX, recv_buffer_ );

  /* If something was read, append it to "our" running copy of the buffer */
  if( read_bytes > 0 )
  {
    running_buffer_mutex_.lock();
    running_buffer_.insert( running_buffer_.end(), recv_buffer_.begin(), recv_buffer_.begin() + read_bytes );
    running_buffer_mutex_.unlock();
  }
  
  /* start dropping messages from the buffer if it grows to crazy sizes */
  if( running_buffer_.size() > MAX_BUFFER_LEN )
  {
    running_buffer_mutex_.lock();
    running_buffer_.erase( running_buffer_.begin(), running_buffer_.begin() + BUFFER_OVF_DROP );
    running_buffer_mutex_.unlock();
    printf( "Serial interface dropping bytes. Consider decoding faster!\n" );
  }
}

int AjSerialInterface::getPossiblePacket( std::vector<uint8_t> &buf, int len )
{
  /* Return the first slice of the current buffer. Thread safe. */
  running_buffer_mutex_.lock();
  int buffer_size = running_buffer_.size();
  running_buffer_mutex_.unlock();
  //std::cout << "running size: " << buffer_size << std::endl;
  
  if( buffer_size <= len )
      return 0;

  std::vector<uint8_t> temp_buf( running_buffer_.begin(), running_buffer_.begin() + len );
  buf = temp_buf ;
  return 1;
}

