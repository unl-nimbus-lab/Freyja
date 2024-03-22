#include "apm_arbitration.hpp"

/* *************************
        RC MAPPING
  aux1 ---> pilot has given computer full authority
  aux2 ---> pilot has requested a software landing
  aux3 ---> pilot has put vehicle into LAND mode (hardware/AP)
  aux4 ---> 
* **************************
*/

ApmModeArbitrator::ApmModeArbitrator() : Node( ROS_NODE_NAME )
{
  declare_parameter<double>( "init_hover_pd", -0.75 );
  declare_parameter<double>( "takeoff_land_spd", 0.2 );
  declare_parameter<double>( "arm_takeoff_delay", 4.0 );
  declare_parameter<double>( "mission_wdg_timeout", 1.0 );
  declare_parameter<double>( "hover_wdg_timeout", -1.0 );
  declare_parameter<bool>( "await_cmd_after_rc", false );
  declare_parameter<int>( "sys_ready_checks", 3 );

  loadParameters();
  
  e_landing_ = false;
  arm_req_sent_ = false;
  target_state_avail_ = false;
  software_trigger_recv = false;
  land_from_hovertimeout_ = HOVER_WDG_TIMEOUT > 0.0;
  t_clock_ = 0.0;

  vehicle_mode_ = VehicleMode::NO_CONNECT;
  mission_mode_ = MissionMode::NOT_INIT;

  /* create at least two groups -- and put the timer in a separate group.
    The clients will default to a separate 'node default' group (MutExcl).
     !! NOTE: The current_state and target_state callbacks are in a reentrant group. !!
  */
  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  csrs_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = csrs_cb_group_;

  curstate_sub_ = create_subscription<CurrentState>( "current_state", 1, 
                            std::bind(&ApmModeArbitrator::currentStateCallback,this,_1), options );
  tgtstate_sub_ = create_subscription<ReferenceState>( "target_state", 1,
                            std::bind(&ApmModeArbitrator::targetStateCallback, this,_1), options );                            
  fstatus_sub_ = create_subscription<FreyjaIfaceStatus>( "freyja_interface_status", 1,
                            std::bind(&ApmModeArbitrator::freyjaStatusCallback, this,_1) );
  refstate_pub_ = create_publisher<ReferenceState>( "reference_state", 1 );    
  manager_mode_pub_ = create_publisher<std_msgs::msg::UInt8> ( "flight_arbitration_mode", 1);                      

  arming_client_ = create_client<MavrosArming> ( "mavros/cmd/arming" );
  biasreq_client_ = create_client<BoolServ> ( "set_bias_compensation" );
  extfcorr_client_ = create_client<BoolServ> ( "set_extf_correction" );
  groundidle_client_ = create_client<BoolServ> ( "set_onground_idle" );

  elanding_serv_ = create_service<Trigger>( "blind_software_landing",
                            std::bind(&ApmModeArbitrator::eLandingServiceHandler, this, _1, _2) );
  start_manager_serv_ = create_service<Trigger>( "start_arm_takeoff",
                            [this](const Trigger::Request::SharedPtr rq, const Trigger::Response::SharedPtr rp)
                            {
                              software_trigger_recv = true;
                              rp->success = true;
                            } );
  arbitrator_rate_ = 10.0;   // mode arbitrator runs at this rate, hz
  manager_timer_ = create_timer( this, get_clock(),
                            std::chrono::duration<float>(1.0/arbitrator_rate_),
                            std::bind(&ApmModeArbitrator::manager, this),
                            timer_cb_group_ );
  t_manager_init_ = now();
}

void ApmModeArbitrator::loadParameters()
{
  get_parameter( "takeoff_land_spd", takeoff_spd_ );    // m/s
  get_parameter( "init_hover_pd", init_hover_pd_ );     // hover here at first takeoff
  get_parameter( "arm_takeoff_delay", ARM_TAKEOFF_DELAY );
  get_parameter( "mission_wdg_timeout", MISSION_WDG_TIMEOUT );
  get_parameter( "hover_wdg_timeout", HOVER_WDG_TIMEOUT );
  get_parameter( "await_cmd_after_rc", await_cmd_after_switch_ );
  int sys_ready_checks;
  get_parameter( "sys_ready_checks", sys_ready_checks );
  sysready_chk_flags_ = 0b11111111 & sys_ready_checks;
}

/*
    "target_state" : Callback for external reference generators.
    This reference is forwarded to the controller only when in MISSION_EXEC mode.
*/
void ApmModeArbitrator::targetStateCallback( const ReferenceState::ConstSharedPtr msg )
{
  target_state_avail_ = true;
  t_tgtstate_update_ = t_clock_;            // for watchdog
  // publish only if we are in MISSION_EXEC mode
  if( forward_ref_state_ )
    refstate_pub_->publish( *msg );
}

void ApmModeArbitrator::sendMavrosArmCommand( const bool req )
{
  auto arm_req = std::make_shared<MavrosArming::Request> ();
  arm_req->value = req;
  arming_client_->async_send_request( arm_req );
  RCLCPP_WARN( get_logger(), "Requesting arming state = %d.", req );
}

void ApmModeArbitrator::eLandingServiceHandler( const Trigger::Request::SharedPtr rq,
                                                const Trigger::Response::SharedPtr rp )
{
  RCLCPP_WARN( get_logger(), "** LAND REQUEST RECV **" );
  mission_mode_ = MissionMode::E_LANDING;
  rp->success = true;
}

void ApmModeArbitrator::freyjaStatusCallback( const FreyjaIfaceStatus::ConstSharedPtr msg )
{
  /* msg contains vehicle mode, armed?, and rcdata */
  /*
    This function handles vehicle state as reported by the vehicle (not our guesses),
    and thus should modify only the vehicle_mode_ enum. The mission_mode_ variable is
    generally not modified here, unless for special cases.
  */
  static int skip_initial_msgs = 0;
  if( skip_initial_msgs < 20 )
  {
    skip_initial_msgs++;
    RCLCPP_WARN( get_logger(), "Intentionally waiting .." );
    return;
  }

  uint8_t sys_readiness = (SysReadyChecks::Connected * msg->connected)  |
                          (SysReadyChecks::CurState * ((t_clock_ - t_curstate_update_) < 0.5)) |
                          (SysReadyChecks::GpsRtk * true);
  bool sys_healthy = sysready_chk_flags_ == (sysready_chk_flags_ & sys_readiness);
                                                 
  if( sys_healthy )
  {
    if( mission_mode_ == MissionMode::NOT_INIT )
    { //. this begins the state machine arbitration
      mission_mode_ = MissionMode::PENDING_PILOT;
      vehicle_mode_ = VehicleMode::DISARMED_NOCOMP;
    }
  }
  else
  { //. we are not connected yet, or have lost connection, or have no state data
    mission_mode_ = MissionMode::NOT_INIT;
    vehicle_mode_ = VehicleMode::NO_CONNECT;
    // we could exit the callback here, but there may be additional work that we
    // could be doing regardless of these conditions, so let's continue.
  }

  if( msg->computer_ctrl == true )
  { //. pilot has given us control authority
    vehicle_mode_ = (msg->armed)? VehicleMode::ARMED_COMP : VehicleMode::DISARMED_COMP;
  }
  else
  { //. pilot has not given us authority
    vehicle_mode_ = (msg->armed)? VehicleMode::ARMED_NOCOMP : VehicleMode::DISARMED_NOCOMP;
  }

  // handle aux data
  static bool extf_state = false;
  if( extf_state != msg->aux4 )
  {
    // rc state has changed
    auto extf_req = std::make_shared<BoolServ::Request> ();
    extf_req -> data = msg->aux4;
    extfcorr_client_ -> async_send_request( extf_req );
    extf_state = msg->aux4;
  }
}

void ApmModeArbitrator::manager()
{
  /* Main mode manager function, called at a fixed rate */
  static std_msgs::msg::UInt8 manager_mode_msg;
  t_clock_ = (now() - t_manager_init_).seconds();

  switch( mission_mode_ )
  {
    case MissionMode::PENDING_PILOT :
      { //. we are not in control at the moment, so keep checking if we are
        RCLCPP_INFO_THROTTLE( get_logger(), *(get_clock()), 2000, "Awaiting Pilot Switch .." );
        
        // there are 4 vehicle modes we can generally be in during this mode; 2 interesting
        if( vehicle_mode_ == VehicleMode::DISARMED_COMP )
        { //. pilot has given control (RC), but arming has not happened.
          mission_mode_ = MissionMode::PENDING_CMD;
        }
        else if( vehicle_mode_ == VehicleMode::ARMED_COMP )
        { //. we went from PENDING_PILOT to being in the air with computer control
          t_comp_armed_ = -100.0;                       // don't know when this arming happened
          mission_mode_ = MissionMode::MISSION_EXEC;    // no other states in between
        }
        forward_ref_state_ = false;
        break;
      }

    case MissionMode::PENDING_CMD :
      { //. pilot has given us authority, but we should probably wait for a software trigger
        if( await_cmd_after_switch_ && !software_trigger_recv )
        { //. can't do anything until a software command is received
          RCLCPP_INFO_THROTTLE( get_logger(), *(get_clock()), 2000, "Pilot ready -- awaiting GO command." );
          keepDisarmedAndHappy();
        }
        else
          software_trigger_recv = true;   // automatically transition to next mode

        if( software_trigger_recv )
        { //. we are clear to go!
          if( vehicle_mode_==VehicleMode::DISARMED_COMP )
          {
            if ( !arm_req_sent_ )
            { //. send the very first arm command
              RCLCPP_WARN( get_logger(), "Sending ARM command!!" );
              auto idlereq = std::make_shared<BoolServ::Request> ();
              idlereq -> data = false;                // unlock ground lock
              groundidle_client_ -> async_send_request( idlereq );
              sendMavrosArmCommand( true );           // send command
              arm_req_sent_ = true;                   // prevent returning here
            }
          }
          else if( vehicle_mode_ == VehicleMode::ARMED_COMP )
          {
            arm_req_sent_ = false;                    // clear this for future
            t_comp_armed_ = t_clock_;                 // note when arming happened
            arming_ned_ = pos_ned_;                   // update where arming happened
            updateManagerRefNED(arming_ned_);         // update our reference state
            mission_mode_ = MissionMode::TAKING_OFF;  // change to takeoff
            RCLCPP_WARN( get_logger(), "-- ARMED BY COMPUTER -- TAKING OFF --" );
          }
        }
        forward_ref_state_ = false;
        break;
      }

    case MissionMode::TAKING_OFF :
      { 
        // wait for a bit before actually taking off
        if( (t_clock_ - t_comp_armed_) < ARM_TAKEOFF_DELAY )
        {
          RCLCPP_WARN_THROTTLE( get_logger(), *(get_clock()), 900, "Prepare for takeoff .." );
          break;
        }
        // push reference state up until takeoff is done
        if( pos_ned_(2) > init_hover_pd_ )
        {
          manager_refstate_.pd -= (takeoff_spd_*1.0/arbitrator_rate_);
          manager_refstate_.vd = -takeoff_spd_;
          RCLCPP_WARN_THROTTLE( get_logger(), *(get_clock()), 500, "TAKEOFF in-prog: %0.1f/%0.1f", pos_ned_(2), init_hover_pd_ );
        }
        else
        {
          // altitude ok, switch to hover mode
          manager_refstate_.vd = 0.0;
          updateManagerRefNED( pos_ned_ );
          RCLCPP_INFO( get_logger(), "Done takeoff." );
          mission_mode_ = MissionMode::HOVERING;
          t_enter_auto_hover_ = t_clock_;
          // turn on bias compensation
          auto biasreq = std::make_shared<BoolServ::Request> ();
          biasreq -> data = true;
          biasreq_client_ -> async_send_request( biasreq );
        }
        refstate_pub_->publish( manager_refstate_ );
        forward_ref_state_ = false;
        break;
      }

    case MissionMode::HOVERING :
      {
        // wait here until a target_state is available (from an external source)
        if( target_state_avail_ )
          mission_mode_ = MissionMode::MISSION_EXEC;
        else if( land_from_hovertimeout_ )
        { //. if landing from hover-timeout is enabled
          if( (t_clock_ - t_enter_auto_hover_) > HOVER_WDG_TIMEOUT )
          { //. switch to landing mode
            RCLCPP_WARN( get_logger(), "Hover timeout. Switching to auto landing!" );
            mission_mode_ = MissionMode::E_LANDING;
          }
        }

        // publish zero velocities; positions are captured at other places
        manager_refstate_.vn = manager_refstate_.ve = manager_refstate_.vd = 0.0;
        refstate_pub_->publish( manager_refstate_ );
        forward_ref_state_ = false;
        break;
      }

    case MissionMode::E_LANDING :
      {
        forward_ref_state_ = false;
        // freeze current position as reference
        if( !e_landing_ )
        {
          updateManagerRefNED( pos_ned_ );
          // zero out velocities
          manager_refstate_.vn = manager_refstate_.ve = manager_refstate_.vd = 0.0;
          refstate_pub_->publish( manager_refstate_ );
          RCLCPP_WARN( get_logger(), " **** FREYJA LANDING! ****\n\tLocking current position .." );
          e_landing_ = true;
          landingInProgress( /*_initialise=*/true );
        }
        
        // push reference state down as long as above arming altitude or reference
        if( landingInProgress() )
        {
          manager_refstate_.pd += (takeoff_spd_*1.1/arbitrator_rate_);
          manager_refstate_.vd = takeoff_spd_;
          refstate_pub_->publish( manager_refstate_ );
        }
        else
        {
          // be extra sure we have pushed reference down enough
          manager_refstate_.pd = arming_ned_(2) + 5.0;
          refstate_pub_->publish( manager_refstate_ );
          
          // send disarm command and freeze arbitration
          auto idlereq = std::make_shared<BoolServ::Request> ();
          idlereq -> data = true;
          groundidle_client_ -> async_send_request( idlereq );
          
          sendMavrosArmCommand( false );
          mission_mode_ = MissionMode::MISSION_END;
        }

        break;
      }

    case MissionMode::MISSION_EXEC :
      {
        // here we simply forward incoming reference state to controller
        forward_ref_state_ = true;

        // also set a timeout for target_state updates
        if( (t_clock_ - t_tgtstate_update_) > MISSION_WDG_TIMEOUT )
        {
          // capture current state, and switch to HOVER
          updateManagerRefNED( pos_ned_ );
          mission_mode_ = MissionMode::HOVERING;
          t_enter_auto_hover_ = t_clock_;
          target_state_avail_ = false;
        }
        
        // handle other custom events from RC switches
        processRCEvents();
        break;
      }
    
    case MissionMode::MISSION_END :
      {
        keepDisarmedAndHappy();
        break;
      }
  }
  manager_mode_msg.data = static_cast<unsigned int>(mission_mode_);
  manager_mode_pub_ -> publish(manager_mode_msg);
  RCLCPP_INFO_THROTTLE( get_logger(), *(get_clock()), 1000,
                        "Mission Mode: %s", MissionModeName[(int)mission_mode_] );
}

void ApmModeArbitrator::processRCEvents()
{
  
}

bool ApmModeArbitrator::landingInProgress( bool _init )
{
  // This function is called repeatedly when in landing mode.
  // Returns true as long as still descending.

  // fill container with default target speed
  static int n_sec = 2;
  static int n_hist = arbitrator_rate_*n_sec;
  static std::vector<double> n_sec_hist(n_hist, takeoff_spd_);
  static int coeff_idx = 0;
  static double avg_desc_spd = 0.0;

  if( _init )
  {
    coeff_idx = 0;
    std::fill( n_sec_hist.begin(), n_sec_hist.end(), takeoff_spd_ );
  }
  else
  {
    // store current velocity
    n_sec_hist[coeff_idx] = vel_d_;
    // calculate avg
    avg_desc_spd = std::accumulate( n_sec_hist.begin(), n_sec_hist.end(), 0.0 )/n_hist;
    // wrap coeff accessor
    coeff_idx = (coeff_idx == n_hist-1)? 0 : coeff_idx+1;
    return avg_desc_spd > 0.02;
  }

  return false;

}

void ApmModeArbitrator::keepDisarmedAndHappy()
{
  // This function may be called quite frequently when the vehicle is
  // set in computer mode but we aren't ready to fly yet. In this state,
  // we send disarm commands and zero thrust periodically.
  static float t_last_disarm = t_clock_;
  if( t_clock_ - t_last_disarm > 2.0 )
  { //. keep sending zero thrust packets and disarm requests
    auto idlereq = std::make_shared<BoolServ::Request> ();
    idlereq -> data = true;
    groundidle_client_ -> async_send_request( idlereq );
    sendMavrosArmCommand( false );
    t_last_disarm = t_clock_;
  }
}

int main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  auto n = std::make_shared<ApmModeArbitrator>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(n);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}