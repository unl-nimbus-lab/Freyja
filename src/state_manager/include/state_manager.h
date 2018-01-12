/* Node header definitions for converting Vicon + vehicle information
  from different formats to one coherent state-vector format.
  
  This node also computes velocity from Vicon.
  
  -- aj / 17th Nov, 2017
*/

#ifndef __STATE_INFO_GENERATOR_H__
#define __STATE_INFO_GENERATOR_H__

#define __USE_VICON 1
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <state_manager/CurrentState.h>
#include <asctec_handler/AsctecData.h>
#if __USE_VICON
#include <tf/tf.h>
#endif

typedef geometry_msgs::TransformStamped TFStamped;


/* The full state vector is defined as:
  [ pn, pe, pd, vn, ve, vd, roll, pitch, yaw, roll_rate_, pitch_rate_, yaw_rate_ ]
*/
const int STATE_VECTOR_LEN = 13;
class StateManager
{
  ros::NodeHandle nh_, priv_nh_;
  
  /* Object for actual full-state */
  std::vector<double> state_vector_;
  
  /* Store last n positions to compute velocity */
  std::vector<double> prev_pn_;
  std::vector<double> prev_pe_;
  std::vector<double> prev_pd_;
  
  float last_pn_, last_pe_, last_pd_;
  
  ros::Time lastUpdateTime_;
  
  /* Filter-specific details for computing velocity */
  int filter_len_;
  std::string filter_type_;
  
  double home_lat_, home_lon_;
  bool have_location_fix_;
  
  public:
    StateManager();
    #if __USE_VICON
    /* Callback handler for Vicon */
    ros::Subscriber vicon_data_sub_;
    void viconCallback( const TFStamped::ConstPtr & );
    #endif

    /* Callback handler for asctec_onboard_data */
    ros::Subscriber asctec_data_sub_;
    void asctecDataCallback( const asctec_handler::AsctecData::ConstPtr & );
    
    /* Publisher for state information */
    ros::Publisher state_pub_;
    
    /* Compute velocity */
    void computeVelocity( float&, float&, float&, float& );
};
#endif
