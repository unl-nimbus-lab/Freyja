/* 
Handles user-requested discrete waypoints and produces the appropriate
reference_state for the controller.
Two modes are supported for waypoints: TIME (0) and SPEED (1).
  TIME(0): generates a trajectory that meets the requested waypoint 
  at exactly t seconds in the future. t is user-provided.
  This is an implementation of the paper:
      Mueller, Mark W., Markus Hehn, and Raffaello D'Andrea.
      "A computationally efficient motion primitive for
      quadrocopter trajectory generation."
      IEEE Transactions on Robotics, Vol. 31.6, pp. 1294-1310, 2015.
  for trajectories that have defined position+velocity start and end states.
  Resultant trajectories are energy-optimal and second-order smooth.

  SPEED(1): generates a linear constant-speed path from current location
  to the requested waypoint. Translational speed target is user-provided.
  
  Overview:
    1. Accept a terminal state [pos3ax,vel3ax,yaw], and, allocated_time or translational_speed;
    2. Snapshot current state
    3. Plan a smooth trajectory towards the destination
    4. Hover in the beginning and end.

  -- aj // May 2020 // Nimbus Lab.
*/

#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"

#include "freyja_msgs/msg/waypoint_target.hpp"
#include "freyja_msgs/msg/current_state.hpp"
#include "freyja_msgs/msg/reference_state.hpp"

#include <eigen3/Eigen/Dense>

#define rclcpp_NODE_NAME "waypoint_manager"

#define DEG2RAD(D) ((D)*3.14153/180.0)

const int nAxes = 3;
typedef Eigen::Matrix<double, 1, 3> PosNED;
typedef Eigen::Matrix<double, 1, 6> PosVelNED;
typedef Eigen::Matrix<double, 1, 9> PosVelAccNED;
typedef Eigen::Matrix<double, 3, 3> PosVelAccNED3x3;

typedef freyja_msgs::msg::ReferenceState  ReferenceState;
typedef freyja_msgs::msg::CurrentState    CurrentState;
typedef freyja_msgs::msg::WaypointTarget  WaypointTarget;

using std::placeholders::_1;

namespace WaypointMode {
  enum WaypointMode : int
  {
    TIME = WaypointTarget::MODE_TIME,
    SPEED = WaypointTarget::MODE_SPEED
  };
}

namespace TerminalBehaviour {
  enum TerminalBehaviour : int
  {
    HOVER = 0,
    HOLD = 1,
    HOVER_STOPPUB = 2,
    HOLD_STOPPUB = 3
  };
}

using WaypointMode::WaypointMode;
using TerminalBehaviour::TerminalBehaviour;


class WaypointManager
{
  public:
    // common elements for all waypoint styles
    PosVelNED target_state_;
    PosVelAccNED3x3 planning_cur_state_;
    WaypointMode::WaypointMode wpt_mode_;
    TerminalBehaviour::TerminalBehaviour term_behav_;
    Eigen::Matrix<double, 1, 3> term_behav_vel_;
  
    double eta_;

    bool have_waypoint_;
    bool have_plan_;
    bool have_forced_waypoint_;

    // elements for mode TIME
    Eigen::Matrix<double, 3, 3> tf_premult_;
    Eigen::Matrix<double, 3, nAxes> albtgm_;
    Eigen::Matrix<double, 3, nAxes> delta_targets_;
    double traj_alloc_duration_;

    // elements for mode SPEED
    Eigen::Matrix<double, 1, 3> segment_gradients_;
    Eigen::Matrix<double, 1, 3> segment_origin_;
    Eigen::Matrix<double, 1, 3> segment_vels_;
    double traj_alloc_speed_;
    double segment_timeratio_;

    inline void update_planning_coeffs( const double &tf )
    {
      tf_premult_ <<  720.0,      -360.0*tf,        60.0*tf*tf,
                  -360.0*tf,    168.0*tf*tf,    -24.0*tf*tf*tf,
                  60.0*tf*tf, -24.0*tf*tf*tf,   3.0*tf*tf*tf*tf;
                  
      /* albtgm_ is stored as three stacked cols of:
          [alpha; beta; gamma] .. for each axis
      */
      albtgm_ = 1.0/std::pow(tf,5) * tf_premult_ * delta_targets_;
    }

    WaypointManager(){ have_waypoint_ = have_plan_ = have_forced_waypoint_ = false; }
    WaypointManager( const PosVelNED _ts )
    {
      target_state_ = _ts;
      have_waypoint_ = true;
      have_plan_ = false;
    }

    inline void setTargetState( const PosVelNED _ts ) { target_state_ = std::move(_ts); }
    inline void getTargetState( PosVelNED &ts ) { ts = target_state_; }
    inline void setForcedTargetState( const PosVelNED _ts )
    {
      setTargetState( _ts );
      have_forced_waypoint_ = true;
    }
    inline void setCurrentState( const PosVelAccNED &_cs )
    { // this variable is a 3x3 matrix
      planning_cur_state_ << _cs.head<3>(),
                            _cs.tail<3>(),
                            0.0, 0.0, 0.0;
    }
    inline void setWaypointModeAndBehav( WaypointMode::WaypointMode &m, TerminalBehaviour::TerminalBehaviour &t )
    { // simply copy
      wpt_mode_ = m;
      term_behav_ = t;
    }

    // interface for the outside world
    void setWaypointStates( const PosVelAccNED &_cs, const PosVelNED &_ts )
    {
      setTargetState( _ts );   // set target state as such
      setCurrentState( _cs );  // set current state (will be reshaped a bit differently)
      have_waypoint_ = true;
    }
    
    void triggerReplanning( double param1, double param2, WaypointMode::WaypointMode& , TerminalBehaviour::TerminalBehaviour& );
    void getCurrentReference( const double t_seg, PosVelAccNED &pva, bool &segment_done );
};


void WaypointManager::triggerReplanning( double p1, double p2, WaypointMode::WaypointMode &m, TerminalBehaviour::TerminalBehaviour &b )
{
  /* Handle a request to plan a trajectory. This function is supposed to be called
    immediately after calling `setWaypointStates(..)`, in order to plan a trajectory
    from the current and target states set by that function. A delayed call, or repeated
    call to this function will result in unintended behaviour: a new plan will be created
    from the old current-state to the old target-state, without accounting for where the
    robot is right now.

    The input argument `p1` is either allocated_time or translational_speed, based on the
    WaypointMode argument. TerminalBehaviour defines what happens when the waypoint is
    reached: hover, or hold last velocity.
  */

  wpt_mode_ = m;
  term_behav_ = b;

  if( wpt_mode_ == WaypointMode::TIME )
  {
    traj_alloc_duration_ = std::move(p1);
    /* TWO STEPS:
        1. Take current snapshot
            Deltas are shaped as: [px py pz; vx vy vz].
    */ 
    auto targetpos = target_state_.head<3>();
    auto targetvel = target_state_.tail<3>();
    auto currentpos = planning_cur_state_.row(0);
    auto currentvel = planning_cur_state_.row(1);

    delta_targets_ << targetpos - currentpos - (traj_alloc_duration_*currentvel),
                      targetvel - currentvel,
                      0.0, 0.0, 0.0;

    // 2. update timing parameters
    update_planning_coeffs( traj_alloc_duration_ );

    // also update other containers
    eta_ = traj_alloc_duration_;
  }
  else if( wpt_mode_ == WaypointMode::SPEED )
  {
    traj_alloc_speed_ = std::move(p2);
    // Get current and target position
    auto targetpos = target_state_.head<3>();
    auto currentpos = planning_cur_state_.row(0);

    // x = x0 + (x1 - x0)t ; y = y0 + (y1 - y0)t ; z = z0 + (z1 - z0)t
    segment_gradients_ = targetpos - currentpos;
    segment_origin_ = currentpos;

    // Calculate length
    double segment_length = (targetpos - currentpos).norm();

    segment_timeratio_ = traj_alloc_speed_ / segment_length;
    segment_vels_ = segment_gradients_ * segment_timeratio_;

    // update eta
    eta_ = segment_length / traj_alloc_speed_;
  }

  have_waypoint_ = true;                // this waypoint is the current
  have_plan_ = true;                    // we have a plan to get there
  have_forced_waypoint_ = false;        // clear any forced initialisation waypoint
  if( term_behav_ == TerminalBehaviour::HOVER )
    term_behav_vel_.setZero();
  else if( term_behav_ == TerminalBehaviour::HOLD )
    term_behav_vel_ = target_state_.tail<3>();
  else
    term_behav_vel_.setZero();
}

void WaypointManager::getCurrentReference( const double t_seg, PosVelAccNED &pva, bool &segment_done )
{
  static PosVelAccNED3x3 tref;
  static Eigen::Matrix<double, 3, 3> tnow_matrix;

  segment_done = false;
  if( have_forced_waypoint_ )
  { //. have a forced waypoint target (but usually no plan), just return that waypoint.
    // A replan event clears this behaviour.
    pva << target_state_.head<3>(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    return;
  }

  // check if segment is done (note this variable is returned-by-ref)
  if( wpt_mode_ == WaypointMode::TIME )
    segment_done = (t_seg >= traj_alloc_duration_);
  else if( wpt_mode_ == WaypointMode::SPEED )
    segment_done = (t_seg*segment_timeratio_ >= 1);

  if( segment_done )
  { //. handle terminal behaviour: pos, acc are fixed, vel is dependent on param during planning
    pva << target_state_.head<3>(),
          term_behav_vel_,
          0.0, 0.0, 0.0;
    have_waypoint_ = false;
    have_plan_ = false;
  }
  else
  { //. segment is ongoing
    if( wpt_mode_ == WaypointMode::TIME )
    {
      double t5 = std::pow( t_seg, 5 )/120.0;
      double t4 = std::pow( t_seg, 4 )/24.0;
      double t3 = std::pow( t_seg, 3 )/6.0;
      double t2 = std::pow( t_seg, 2 )/2.0;
  
      tnow_matrix << t5, t4, t3,
                      t4, t3, t2,
                      t3, t2, t_seg;  
      // three horz-stacked cols, each like [pos;; vel;; acc], one for each axis.
      tref = tnow_matrix * albtgm_ +
                (Eigen::Matrix<double, 3, 3>() <<
                                  1, t_seg, t2,
                                  0,  1, t_seg,
                                  0,  0,   1).finished() * planning_cur_state_;
      // available Eigen3.4: pva = tref.reshaped<Eigen::RowMajor>();
      pva << tref.row(0), tref.row(1), tref.row(2);
    }
    else if( wpt_mode_ == WaypointMode::SPEED )
    {
      // Calculate new position based on line
      auto new_pos = segment_gradients_ * (t_seg*segment_timeratio_) + 
                      segment_origin_;

      pva << new_pos, segment_vels_, 0.0, 0.0, 0.0;
    }
  }
}




class TrajectoryGenerator : public rclcpp::Node
{
  WaypointManager WP;

  Eigen::Matrix<double, 1, 9> current_state_;
  rclcpp::Time t_last_wpt_;
  
  bool traj_init_;
  bool publish_after_seg_;
  
  float k_thresh_skipreplan_;

  std::vector<double> init_NEDy_;
  double yaw_target_;
  
  public:
    TrajectoryGenerator();

    // requested final state
    rclcpp::Subscription<WaypointTarget>::SharedPtr waypoint_sub_;
    void waypointCallback( const WaypointTarget::ConstSharedPtr );
    
    // vehicle current state (from state_manager)
    rclcpp::Subscription<CurrentState>::SharedPtr current_state_sub_;
    void currentStateCallback( const CurrentState::ConstSharedPtr );
    
    rclcpp::Publisher<ReferenceState>::SharedPtr trajref_pub_;
    ReferenceState trajref_;
    
    rclcpp::TimerBase::SharedPtr traj_timer_;
    void trajectoryReferenceManager();
    
    void publishHoverReference();
};

TrajectoryGenerator::TrajectoryGenerator() : rclcpp::Node( rclcpp_NODE_NAME )
{
  /* Operational constants */
  k_thresh_skipreplan_ = 0.10;       // don't replan if new point within this radius

  /* initial location for hover */
  double nan_q = std::numeric_limits<double>::quiet_NaN();
  declare_parameter<std::vector<double>>( "init_NEDy", std::vector<double>({nan_q, nan_q, nan_q, nan_q}) );

  traj_init_ = false;
  publish_after_seg_ = true;

  // Init eigen matrices 
  current_state_.setZero();
  
  /* Subscriptions */
  current_state_sub_ = create_subscription<CurrentState> ( "current_state", 1,
                            [this]( const CurrentState::ConstSharedPtr msg )
                            { current_state_.head<6>() = Eigen::Map<const PosVelNED>( msg->state_vector.data() ); } );
  waypoint_sub_ = create_subscription<WaypointTarget>( "discrete_waypoint_target", 1, 
                           std::bind(&TrajectoryGenerator::waypointCallback, this, _1) );

  /* Publishers */
  trajref_pub_ = create_publisher<ReferenceState>( "reference_state", 1 );

  /* Fixed-rate trajectory provider. Ensure ~40-50hz. */
  float traj_period = 1.0/50.0;
  traj_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<float>(traj_period),
                            std::bind(&TrajectoryGenerator::trajectoryReferenceManager, this) );

  RCLCPP_INFO( get_logger(), "Initialized; waiting for waypoint .." );
  get_parameter( "init_NEDy", init_NEDy_ );
  if( std::none_of( init_NEDy_.cbegin(), init_NEDy_.cend(), [](const double &d){return std::isnan(d);} ) )
  {//. user provided a complete initial waypoint ..
    PosVelNED initial_wpt;
    initial_wpt << init_NEDy_[0], init_NEDy_[1], init_NEDy_[2], 0.0, 0.0, 0.0;
    WP.setForcedTargetState( initial_wpt );
    yaw_target_ = init_NEDy_[3];
    traj_init_ = true;
    t_last_wpt_ = now();
    RCLCPP_WARN( get_logger(), "Using given initial waypoint!" );
  }
}

void TrajectoryGenerator::waypointCallback( const WaypointTarget::ConstSharedPtr msg )
{
  /*
    WaypointTarget:
      [terminal_pn, terminal_pe, terminal_pd, terminal_vn, terminal_ve, terminal_vd, terminal_yaw]
      allocated_time
      translational_speed
      waypoint_mode (WaypointTarget::TIME=0, WaypointTarget::SPEED=1)
  */
  
  // Check if we aren't in a correct mode (time or speed mode)
  if( msg->waypoint_mode != msg->MODE_TIME && msg->waypoint_mode != msg->MODE_SPEED )
  {
    // Reject waypoint
    RCLCPP_WARN( get_logger(), "Waypoint mode incorrect. Ignoring!" );
    return;
  }
  
  // guess if provided waypoint mode was "accidental" or maybe wrong
  if( msg->waypoint_mode == msg->MODE_TIME && msg->allocated_time < 0.01 )
  {
    RCLCPP_WARN( get_logger(), "Allocated time too small (possibly zero). Ignoring!\n\t--- Did you mean to use SPEED mode?" );
    return;
  }
  
  if( msg->waypoint_mode == msg->MODE_SPEED && msg->translational_speed < 0.001 )
  {
    RCLCPP_WARN( get_logger(), "Translational speed too small (possibly zero). Ignoring!\n\t--- Speed must be positive, and > 0.001 m/s." );
    return;
  }


  // check if the change is big enough to do a replan: target_state_: [1x6]
  static PosVelNED candidate_target_state, last_target_state;
  candidate_target_state << msg->terminal_pn, msg->terminal_pe, msg->terminal_pd,
                            msg->terminal_vn, msg->terminal_ve, msg->terminal_vd;

  bool accept_wpt = !traj_init_ ||
                    (candidate_target_state.head<3>() - last_target_state.head<3>()).norm() > k_thresh_skipreplan_;
  if( accept_wpt )
  {
    /* accept new waypoint */
    auto m = static_cast<WaypointMode::WaypointMode>(msg->waypoint_mode);
    auto b = static_cast<TerminalBehaviour::TerminalBehaviour>(msg->after_waypoint_done);
    WP.setWaypointStates(current_state_, candidate_target_state);
    WP.triggerReplanning(msg->allocated_time, msg->translational_speed, m, b);
    yaw_target_ = msg->terminal_yaw;
    if(msg->after_waypoint_done == TerminalBehaviour::HOLD_STOPPUB || msg->after_waypoint_done == TerminalBehaviour::HOVER_STOPPUB)
      publish_after_seg_ = false;
    else
      publish_after_seg_ = true;
    
    RCLCPP_INFO( get_logger(), "Plan generated!" );
    t_last_wpt_ = now();
    last_target_state = candidate_target_state;
    traj_init_ = true;
  }
  else
  {
    // Reject waypoint because it is not distinct enough
    RCLCPP_WARN( get_logger(), "New WP too close to old WP. Ignoring!" );
  }
}

void TrajectoryGenerator::trajectoryReferenceManager( )
{
  /* This is the trajectory generator - gets called at a fixed rate.
  This must keep track of current time since "go". If go-signal has
  not been recorded yet, we must stay at initial position.
  */
  static PosVelAccNED tref; // 1x9: [pn, pe, pd, vn, ve, vd, an, ae, ad]
  static bool segment_done = false;

  if( !traj_init_ )
    return;

  
  float t_seg = (now() - t_last_wpt_).seconds();

  WP.getCurrentReference( t_seg, tref, segment_done );
  
  /* fill in and publish */
  trajref_.pn = tref.coeff(0);
  trajref_.pe = tref.coeff(1);
  trajref_.pd = tref.coeff(2);

  trajref_.vn = tref.coeff(3);
  trajref_.ve = tref.coeff(4);
  trajref_.vd = tref.coeff(5);

  trajref_.an = tref.coeff(6);
  trajref_.ae = tref.coeff(7);
  trajref_.ad = tref.coeff(8);

  trajref_.yaw = yaw_target_;     // nothing special for yaw
  trajref_.header.stamp = now();

  // Publish only if segment is not done, or if the publisher
  // is requested to stay active after a segment
  if( !segment_done || publish_after_seg_ )
    trajref_pub_ -> publish( trajref_ );
}


int main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<TrajectoryGenerator>() );
  rclcpp::shutdown();
  return 0;
}
