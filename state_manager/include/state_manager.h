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
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <std_srvs/SetBool.h>

#include <freyja_msgs/CurrentState.h>
#include <freyja_msgs/AsctecData.h>

#include "freyja_filters.cpp"

typedef geometry_msgs::TransformStamped TFStamped;
typedef geometry_msgs::TwistStamped TwStamped;
typedef nav_msgs::Odometry CameraOdom;
typedef std_srvs::SetBool::Request BoolServReq;
typedef std_srvs::SetBool::Response BoolServRsp;

#define DEG2RAD(D) ((D)*3.1415326/180.0)
#define F_PI 3.14153

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
  FreyjaFilters pose_filter_;
  FreyjaFilters rate_filter_;
  
  /* containers for handling gps data */
  double home_lat_, home_lon_;
  bool have_location_fix_;
  double compass_yaw_;
  
  bool have_arming_origin_;
  double map_rtk_pn_, map_rtk_pe_, map_rtk_pd_;
  double arming_gps_pn_, arming_gps_pe_, arming_gps_pd_;
  double gps_odom_pn_, gps_odom_pe_, gps_odom_pd_;
  double rtk_baseoffset_pn_, rtk_baseoffset_pe_, rtk_baseoffset_pd_;
  
  public:
    StateManager();
    /* launch-time parameter specifies which one to pick */
    void initPixhawkManager();
    void initAsctecManager();
    void initViconManager();
    void initCameraManager();

    /* Callback handler for Vicon */
    ros::Subscriber vicon_data_sub_;
    void viconCallback( const TFStamped::ConstPtr & ) __attribute__((hot));

    /* Callback handler for asctec_onboard_data */
    ros::Subscriber asctec_data_sub_;
    void asctecDataCallback( const freyja_msgs::AsctecData::ConstPtr & );
    
    /* Callback handler for camera updates */
    ros::Subscriber camera_estimate_sub_;
    void cameraUpdatesCallback( const CameraOdom::ConstPtr & );
    
    /* Callback handlers for mavros data */
    ros::Subscriber mavros_gpsraw_sub_;
    ros::Subscriber mavros_vel_sub_;
    ros::Subscriber compass_sub_;
    ros::Subscriber mavros_gpsodom_sub_;
    ros::Subscriber mavros_rtk_sub_;
    void mavrosGpsOdomCallback( const nav_msgs::Odometry::ConstPtr & );
    void mavrosCompassCallback( const std_msgs::Float64::ConstPtr & );
    void mavrosRtkBaselineCallback( const geometry_msgs::Vector3::ConstPtr & );
    
    void mavrosGpsRawCallback( const sensor_msgs::NavSatFix::ConstPtr & );
    
    /* handlers for locking map frame origins */
    inline void lockArmingGps( bool _lock = true )
    {
      arming_gps_pn_ = _lock? gps_odom_pn_ : 0.0;
      arming_gps_pe_ = _lock? gps_odom_pe_ : 0.0;
      arming_gps_pd_ = _lock? gps_odom_pd_ : 0.0;
    }
    inline void lockMapRTK( bool _lock = true )
    {
      map_rtk_pn_ = _lock? rtk_baseoffset_pn_ : 0.0;
      map_rtk_pe_ = _lock? rtk_baseoffset_pe_ : 0.0;
      map_rtk_pd_ = _lock? rtk_baseoffset_pd_ : 0.0;
    }
    
    ros::ServiceServer maplock_srv_;
    bool maplockArmingHandler( BoolServReq&, BoolServRsp& );

    
    /* Publisher for state information */
    ros::Publisher state_pub_;
    

};
#endif
