/* An implementation of the paper:
      Mueller, Mark W., Markus Hehn, and Raffaello D'Andrea.
      "A computationally efficient motion primitive for
      quadrocopter trajectory generation."
      IEEE Transactions on Robotics, Vol. 31.6, pp. 1294-1310, 2015.
  for trajectories that have defined position+velocity start and end states.
  Resultant trajectories are energy-optimal and second-order smooth.

  Overview:
    1. Accepts a terminal state [pos3ax,vel3ax,yaw] and an allocated_time;
    2. Snapshot current state
    3. Plan a smooth trajectory that takes allocated_time to finish
    4. Hover in the beginning and end.

  -- aj // May 2020 // Nimbus Lab.
*/

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <freyja_msgs/WaypointTarget.h>
#include <freyja_msgs/CurrentState.h>
#include <freyja_msgs/ReferenceState.h>

#include <eigen3/Eigen/Dense>

#define ROS_NODE_NAME "trajectory_primitives"

#define DEG2RAD(D) ((D)*3.14153/180.0)

const int nAxes = 3;
typedef Eigen::Matrix<double, 1, 3> Pos3ax;
typedef Eigen::Matrix<double, 1, 6> PosVel3ax;
typedef Eigen::Matrix<double, 1, 9> PosVelAcc3ax;

enum modes{time_mode=0, speed_mode=1};

class TrajectoryGenerator
{
  ros::NodeHandle nh_, priv_nh_;
  Eigen::Matrix<double, 3, 3> tf_premult_;
  Eigen::Matrix<double, 3, nAxes> albtgm_;
  Eigen::Matrix<double, 3, nAxes> delta_targets_;
  Eigen::Matrix<double, 1, 9> current_state_;
  Eigen::Matrix<double, 3, 3> planning_cur_state_;
  Eigen::Matrix<double, 1, 6> final_state_;

  Eigen::Matrix<double, 1, 3> segment_gradients_;
  Eigen::Matrix<double, 1, 3> segment_intersection_;

  double yaw_target;
  
  /* timing related containers */
  double traj_alloc_duration_;
  double traj_alloc_speed_;
  Eigen::Matrix<double, 3, 3> tnow_matrix_;
  ros::Time t_traj_init_;
  ros::Timer traj_timer_;
  bool traj_init_;

  modes mode_;
  double segment_percentage_;
  
  float k_thresh_skipreplan_;

  float init_pn_;
  float init_pe_;
  float init_pd_;
  
  int terminal_style_; // 0: posn+vel, 1: posn+vel+acc
  
  inline void update_albtgm( const double& );
  inline void update_premult_matrix( const double& );
  inline void trigger_replan_time( const double& );
  inline void trigger_replan_speed( const double& );
  
  public:
    TrajectoryGenerator();

    // requested final state
    ros::Subscriber waypoint_sub_;
    void waypointCallback( const freyja_msgs::WaypointTarget::ConstPtr & );
    
    // vehicle current state (from state_manager)
    ros::Subscriber current_state_sub_;
    void currentStateCallback( const freyja_msgs::CurrentState::ConstPtr & );
    
    ros::Publisher traj_ref_pub_;
    freyja_msgs::ReferenceState traj_ref_;
    void trajectoryReference( const ros::TimerEvent & );
    
    void publishHoverReference();

};

TrajectoryGenerator::TrajectoryGenerator() : nh_(), priv_nh_("~")
{
  /* Operational constants */
  k_thresh_skipreplan_ = 0.10;       // don't replan if new point within this radius

  /* initial location for hover */
  priv_nh_.param( "init_pn", init_pn_, float(0.0) );
  priv_nh_.param( "init_pe", init_pe_, float(0.0) );
  priv_nh_.param( "init_pd", init_pd_, float(-0.75) );

  /* unused argument for trajectory shaping */
  priv_nh_.param( "term_style", terminal_style_, int(1) );

  /* used for speed and duration constraints*/
  traj_alloc_duration_ = 10.0;
  traj_alloc_speed_ = 1.0;

  traj_init_ = false;

  // default to speed mode (as per waypoint default)
  mode_ = speed_mode;

  // Init eigen matrices 
  current_state_ = Eigen::Matrix<double, 1, 9>::Zero();
  update_premult_matrix( traj_alloc_duration_ );
  
  /* Subscribers */
  current_state_sub_ = nh_.subscribe( "/current_state", 1,
                           &TrajectoryGenerator::currentStateCallback, this );
  waypoint_sub_ = nh_.subscribe( "/waypoint_state", 1, 
                           &TrajectoryGenerator::waypointCallback, this );

  /* Publishers */
  traj_ref_pub_ = nh_.advertise<freyja_msgs::ReferenceState>( "/reference_state", 1, true );

  /* Fixed-rate trajectory provider. Ensure ~40-50hz. */
  float traj_period = 1.0/50.0;
  traj_timer_ = nh_.createTimer( ros::Duration(traj_period),
                            &TrajectoryGenerator::trajectoryReference, this );

  ROS_WARN_STREAM( ros::this_node::getName() << ": Initialized, waiting for waypoint" );
}

void TrajectoryGenerator::currentStateCallback( const freyja_msgs::CurrentState::ConstPtr &msg )
{
  /* make current_state available locally */
  current_state_.head<6>() = Eigen::Map<const PosVel3ax>( msg->state_vector.data() );
}

void TrajectoryGenerator::waypointCallback( const freyja_msgs::WaypointTarget::ConstPtr &msg )
{
  /*
    WaypointTarget:
      [terminal_pn, terminal_pe, terminal_pd, terminal_vn, terminal_ve, terminal_vd, terminal_yaw]
      allocated_time
      translational_speed
      waypoint_mode (WaypointTarget::TIME=0, WaypointTarget::SPEED=1)
  */
  // Check if we are in time or speed mode
  if ( msg->waypoint_mode == msg->TIME ||  msg->waypoint_mode == msg->SPEED )
  {
    // Update yaw
    yaw_target = msg->terminal_yaw;

    // check if the change is big enough to do a replan: final_state_: [1x6]
    Eigen::Matrix<double, 1, 6> updated_final_state;
    updated_final_state << msg->terminal_pn, msg->terminal_pe, msg->terminal_pd,
    msg->terminal_vn, msg->terminal_ve, msg->terminal_vd;

    if( ( updated_final_state - final_state_ ).norm() > k_thresh_skipreplan_ )
    {

      /* accept new waypoint */
      final_state_ = updated_final_state;

      if (msg->waypoint_mode == msg->TIME) // Time mode
      {
        traj_alloc_duration_ = msg->allocated_time;
        trigger_replan_time( traj_alloc_duration_ );
        mode_ = time_mode;
      }
      else // Speed mode 
      {
        traj_alloc_speed_ = msg->translational_speed;
        trigger_replan_speed( traj_alloc_speed_ );
        mode_ = speed_mode;
      }
      ROS_WARN( "Plan generated!" );
      t_traj_init_ = ros::Time::now();

    }
    else
    {
      // Reject waypoint
      ROS_WARN_STREAM( ros::this_node::getName() << ": New WP too close to old WP. Ignoring!" );
    }
  }
  else
  {
    // Reject waypoint
    ROS_WARN_STREAM( ros::this_node::getName() << ": Waypoint mode incorrectly specified. Ignoring!" );
  }

  // this is only handled once - trajectory is never reinit, only updated.
  if( traj_init_ == false )
  {
    traj_init_ = true;
  }
}

void TrajectoryGenerator::trigger_replan_time( const double &dt )
{
  /* TWO STEPS:
     1. Take current snapshot
        Deltas are shaped as: [px py pz; vx vy vz].
  */ 
  auto targetpos = final_state_.head<3>();
  auto targetvel = final_state_.tail<3>();
  auto currentpos = current_state_.head<3>();
  auto currentvel = current_state_.block<1,3>(0,3);
  
  delta_targets_ << targetpos - currentpos - dt*currentvel,
                    targetvel - currentvel,
                    0.0, 0.0, 0.0;
  /* save snapshot of current_state */
  planning_cur_state_ << currentpos,
                         currentvel,
                         0.0, 0.0, 0.0;
  
  
  // 2. update timing parameters
  update_premult_matrix( dt );
}

void TrajectoryGenerator::trigger_replan_speed( const double &speed )
{
  // Get current and target position
  auto targetpos = final_state_.head<3>();
  auto currentpos = current_state_.head<3>();

  // x = x0 + (x1 - x0)t ; y = y0 + (y1 - y0)t ; z = z0 + (z1 - z0)t
  segment_gradients_ = targetpos - currentpos;
  segment_intersection_ = currentpos;

  // Calculate length
  double segment_length = (targetpos - currentpos).norm();

  segment_percentage_ = speed / segment_length;
}

inline void TrajectoryGenerator::update_premult_matrix( const double &tf )
{
  tf_premult_ <<  720.0, -360.0*tf, 60.0*tf*tf,
                 -360*tf, 168.0*tf*tf, -24.0*tf*tf*tf,
                 60.0*tf*tf, -24.0*tf*tf*tf, 3.0*tf*tf*tf*tf;
                 
  // necessarily update alpha, beta and gamma as well
  update_albtgm( tf );
}

inline void TrajectoryGenerator::update_albtgm( const double &dt )
{
  /* albtgm_ is stored as three stacked cols of:
          [alpha; beta; gamma] .. for each axis
  */
  albtgm_ = 1.0/std::pow(dt,5) * tf_premult_ * delta_targets_;
}

void TrajectoryGenerator::trajectoryReference( const ros::TimerEvent &event )
{
  /* This is the trajectory generator - gets called at a fixed rate.
  This must keep track of current time since "go". If go-signal has
  not been recorded yet, we must stay at initial position.
  */
    
  float tnow = (ros::Time::now() - t_traj_init_).toSec();
  
  PosVelAcc3ax tref;

  if ( mode_ == time_mode ) // Time mode
  {
    /* check if we are past the final time */
    if( tnow > traj_alloc_duration_ || !traj_init_ )
    {
      publishHoverReference();
      return;
    }
      
    double t5 = std::pow( tnow, 5 )/120.0;
    double t4 = std::pow( tnow, 4 )/24.0;
    double t3 = std::pow( tnow, 3 )/6.0;
    double t2 = std::pow( tnow, 2 )/2.0;
    
    tnow_matrix_ << t5, t4, t3,
                    t4, t3, t2,
                    t3, t2, tnow;  
    // three stacked cols of [pos; vel; acc] for each axis.
    Eigen::MatrixXd traj_pt = tnow_matrix_ * albtgm_ +
                            (Eigen::Matrix<double, 3, 3>() <<
                                1, tnow, t2,
                                0,  1, tnow,
                                0,  0,   1).finished() * planning_cur_state_;

    // TODO (couldnt get transpose to work)
    // tref << traj_pt.transpose();
    tref = traj_pt.reshaped<Eigen::RowMajor>().transpose();
  }
  else // Speed mode
  {
    /* check if we have arrived at the waypoint */
    if( ( tnow * segment_percentage_ ) >= 1  || !traj_init_ )
    {
      publishHoverReference();
      return;
    }

    // Calculate new position based on line
    auto new_pos = segment_gradients_ * (tnow * segment_percentage_) + 
          segment_intersection_;

    // Todo clean up this operation
    // tref << new_pos(0, 0), segment_gradients_(0), 0,
    //         new_pos(0, 1), segment_gradients_(1), 0,
    //         new_pos(0, 2), segment_gradients_(2), 0;
    tref << new_pos, segment_gradients, 0.0, 0.0, 0.0;

  }
  
  /* fill in and publish */
  traj_ref_.pn = tref(0);
  traj_ref_.pe = tref(1);
  traj_ref_.pd = tref(2);

  traj_ref_.vn = tref(3);
  traj_ref_.ve = tref(4);
  traj_ref_.vd = tref(5);

  traj_ref_.an = tref(6);
  traj_ref_.ae = tref(7);
  traj_ref_.ad = tref(8);

  traj_ref_.yaw = yaw_target;     // nothing special for yaw
  traj_ref_.header.stamp = ros::Time::now();

  traj_ref_pub_.publish( traj_ref_ );
}

void TrajectoryGenerator::publishHoverReference()
{
  if( !traj_init_ )
  {
    // trajectory hasn't been initialsed
    traj_ref_.pn = init_pn_;
    traj_ref_.pe = init_pe_;
    traj_ref_.pd = init_pd_;
  }
  else
  {
    // trajectory segment completed
    traj_ref_.pn = final_state_[0];
    traj_ref_.pe = final_state_[1];
    traj_ref_.pd = final_state_[2];
  }
  
  traj_ref_.vn = 0.0;
  traj_ref_.ve = 0.0;
  traj_ref_.vd = 0.0;
  
  traj_ref_.yaw = yaw_target;

  traj_ref_.an = 0.0;
  traj_ref_.ae = 0.0;
  traj_ref_.ad = 0.0;

  traj_ref_.header.stamp = ros::Time::now();
  traj_ref_pub_.publish( traj_ref_ );
}

int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_NODE_NAME );
  TrajectoryGenerator tgen;
  
  ros::spin();
  return 0;
}


//TODO: Add status waypoint_done