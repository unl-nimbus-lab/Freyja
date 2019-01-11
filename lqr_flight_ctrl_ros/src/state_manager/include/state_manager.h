/* Node header definitions for converting Vicon + vehicle information
  from different formats to one coherent state-vector format.
  
  This node also computes velocity from Vicon.
  
  -- aj / 17th Nov, 2017
*/

#ifndef __STATE_INFO_GENERATOR_H__
#define __STATE_INFO_GENERATOR_H__

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <common_msgs/CurrentState.h>
#include <common_msgs/AsctecData.h>
#include "aj_filter_collection.cpp"
#if USE_VICON_ == 1
  #include <tf/tf.h>
#endif

typedef geometry_msgs::TransformStamped TFStamped;
typedef geometry_msgs::TwistStamped TwStamped;
#define DEG2RAD(D) ((D)*3.1415326/180.0)
#define AJ_PI 3.14153

/* The full state vector is defined as:
  [ pn, pe, pd, vn, ve, vd, roll, pitch, yaw, roll_rate_, pitch_rate_, yaw_rate_ ]
*/
const int STATE_VECTOR_LEN = 13;
class StateManager
{
  ros::NodeHandle nh_, priv_nh_;
  
  /* Object for actual full-state */
  std::vector<double> state_vector_;
  
  /* Store last n positions to smoothe it out */
  std::vector<double> prev_pn_;
  std::vector<double> prev_pe_;
  std::vector<double> prev_pd_;
  
  /* Store last n velocities to smoothe it out */
  std::vector<double> prev_vn_;
  std::vector<double> prev_ve_;
  std::vector<double> prev_vd_;
  
  /* Store last n angles to smoothe it out */
  std::vector<double> prev_roll_;
  std::vector<double> prev_pitch_;
  std::vector<double> prev_yaw;
  
  /* Book-keeping for velocities and rates */
  float last_pn_, last_pe_, last_pd_;
  float last_roll_, last_pitch_, last_yaw_;
  
  ros::Time lastUpdateTime_;
  
  /* Pick the source of information at launch time */
  std::string state_source_;
  
  /* Filter-specific details for computing velocity */
  int filter_len_;
  std::string filter_type_;
  AjFilterCollection pose_filter_;
  AjFilterCollection rate_filter_;
  
  double home_lat_, home_lon_;
  bool have_location_fix_;
  
  public:
    StateManager();
    /* launch-time parameter specifies which one to pick */
    void initApmManager();
    void initAsctecManager();
    void initViconManager();

    /* Callback handler for Vicon */
    ros::Subscriber vicon_data_sub_;
    void viconCallback( const TFStamped::ConstPtr & ) __attribute__((hot));

    /* Callback handler for asctec_onboard_data */
    ros::Subscriber asctec_data_sub_;
    void asctecDataCallback( const common_msgs::AsctecData::ConstPtr & );

    /* Callback handlers for mavros data */
    ros::Subscriber mavros_gps_sub_;
    ros::Subscriber mavros_vel_sub_;
    void mavrosGpsCallback( const sensor_msgs::NavSatFix::ConstPtr & );
    void mavrosGpsVelCallback( const TwStamped::ConstPtr & );
    
    
    /* Publisher for state information */
    ros::Publisher state_pub_;
    

};
#endif
