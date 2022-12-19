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
  declare_parameter<double>( "arm_takeoff_delay", 4.0 );
  declare_parameter<double>( "mission_wdg_timeout", 1.0 );

  loadParameters();
  
  e_landing_ = false;
  arm_req_sent_ = false;
  target_state_avail_ = false;
  t_clock_ = 0.0;

  vehicle_mode_ = VehicleMode::NO_CONNECT;
  mission_mode_ = MissionMode::PENDING_PILOT;

  curstate_sub_ = create_subscription<CurrentState>( "current_state", 1, 
                            std::bind(&ApmModeArbitrator::currentStateCallback,this,_1) );
  tgtstate_sub_ = create_subscription<ReferenceState>( "target_state", 1,
                            std::bind(&ApmModeArbitrator::targetStateCallback, this,_1) );                            
  fstatus_sub_ = create_subscription<FreyjaIntStatus>( "freyja_internal_status", 1,
                            std::bind(&ApmModeArbitrator::freyjaStatusCallback, this,_1) );
  refstate_pub_ = create_publisher<ReferenceState>( "reference_state", 1 );                            

  arming_client_ = create_client<MavrosArming> ("mavros/cmd/arming" );
  biasreq_client_ = create_client<BoolServ> ("set_bias_compensation" );

  elanding_serv_ = create_service<Trigger>( "blind_software_landing",
                            std::bind(&ApmModeArbitrator::eLandingServiceHandler, this, _1, _2) );
  arbitrator_rate_ = 10.0;   // mode arbitrator runs at this rate, hz
  manager_timer_ = create_timer( this, get_clock(),
                            std::chrono::duration<float>(1.0/arbitrator_rate_),
                            std::bind(&ApmModeArbitrator::manager, this) );
  t_manager_init_ = now();
}

void ApmModeArbitrator::loadParameters()
{
  takeoff_spd_ = 0.1;                               // m/s
  get_parameter( "init_hover_pd", init_hover_pd_ ); // hover here at first takeoff
  get_parameter( "arm_takeoff_delay", ARM_TAKEOFF_DELAY );
  get_parameter( "mission_wdg_timeout", MISSION_WDG_TIMEOUT );
}

/*
    "target_state" : Callback for external policy controllers.
*/
void ApmModeArbitrator::targetStateCallback( const ReferenceState::ConstSharedPtr msg )
{
  target_state_avail_ = true;
  t_tgtstate_update_ = t_clock_;
  //incoming_ref_ = *msg;
  if( forward_ref_state_ )
    refstate_pub_->publish( *msg );
}

void ApmModeArbitrator::sendMavrosArmCommand( const bool req )
{
  auto arm_req = std::make_shared<MavrosArming::Request> ();
  arm_req->value = req;
  arming_client_->async_send_request( arm_req );
}

void ApmModeArbitrator::eLandingServiceHandler( const Trigger::Request::SharedPtr rq,
                                                const Trigger::Response::SharedPtr rp )
{
  RCLCPP_WARN( get_logger(), "** LAND REQUEST RECV **" );
  mission_mode_ = MissionMode::E_LANDING;
  rp->success = true;
}

void ApmModeArbitrator::freyjaStatusCallback( const FreyjaIntStatus::ConstSharedPtr msg )
{
  /* msg contains vehicle mode, armed?, and rcdata */
  static int skip_initial_msgs = 0;
  if( skip_initial_msgs < 20 )
  {
    skip_initial_msgs++;
    RCLCPP_WARN( get_logger(), "Intentionally waiting .." );
    return;
  }
  // in comp ctrl but not armed
  if( msg->armed == false && msg->computer_ctrl )
  {
    // we are here in the beginning or in the end
    if( mission_mode_ == MissionMode::MISSION_END )
    {
      // DO NOTHING: keep mission mode the same
      mission_mode_ = MissionMode::MISSION_END;
      vehicle_mode_ = VehicleMode::DISARMED_COMP;
    }
    else
    {
      // change to mission arm mode
      mission_mode_ = MissionMode::PENDING_PILOT;
      vehicle_mode_ = VehicleMode::DISARMED_COMP;
      RCLCPP_INFO( get_logger(), "Changing mission-mode to: ARM_NOW." );
    }
  }

  if( msg->armed && msg->computer_ctrl )
  {
    vehicle_mode_ = VehicleMode::ARMED_COMP;
  }

  if( msg->armed && msg->computer_ctrl == false )
  {
    vehicle_mode_ = VehicleMode::ARMED_NOCOMP;
    mission_mode_ = MissionMode::PENDING_PILOT;
  }
}

void ApmModeArbitrator::manager()
{
  /* Main mode manager function, called at a fixed rate */

  t_clock_ = (now() - t_manager_init_).seconds();

  switch( mission_mode_ )
  {
    case MissionMode::PENDING_PILOT :
      {
        RCLCPP_INFO_THROTTLE( get_logger(), *(get_clock()), 2000, "Awaiting Pilot Switch .." );
        // send arming command and don't send again
        if( vehicle_mode_ == VehicleMode::DISARMED_COMP )
        {
          if( !arm_req_sent_ )
          {
            sendMavrosArmCommand( true );
            t_comp_armed_ = t_clock_;
            arm_req_sent_ = true;
          }
        }
        else if( vehicle_mode_ == VehicleMode::ARMED_COMP )
        {
          if( arm_req_sent_ )
          {
            arming_ned_ = pos_ned_;
            updateManagerRefNED( arming_ned_ );
            arm_req_sent_ = false;
            mission_mode_ = MissionMode::TAKING_OFF;
            RCLCPP_WARN( get_logger(), "-- ARMED BY COMPUTER -- TAKING OFF --" );
          }
          else
          {
            // we didn't send arming, but we find ourselves already armed
            t_comp_armed_ = -100.0;             // we don't know when armed            
            updateManagerRefNED( pos_ned_ );
            mission_mode_ = MissionMode::TAKING_OFF;
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
          RCLCPP_WARN_THROTTLE( get_logger(), *(get_clock()), 500, "TAKEOFF ALT: %0.1f/%0.1f", pos_ned_(2), init_hover_pd_ );
        }
        else
        {
          // altitude ok, switch to hover mode
          manager_refstate_.vd = 0.0;
          updateManagerRefNED( pos_ned_ );
          RCLCPP_INFO( get_logger(), "Done takeoff." );
          mission_mode_ = MissionMode::HOVERING;

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
        // wait here until a target_state is available (from policy/rvo3 etc)
        if( target_state_avail_ )
          mission_mode_ = MissionMode::POLICY_EXEC;

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
        }
        
        // push reference state down as long as above arming altitude or reference
        // @TODO: replace with an abstracted "not landingComplete()"
        if( (pos_ned_(2) < arming_ned_(2)) || (manager_refstate_.pd - 3.0 < arming_ned_(2)) )
        {
          manager_refstate_.pd += (takeoff_spd_*1.0/arbitrator_rate_);
          manager_refstate_.vd = takeoff_spd_;
          refstate_pub_->publish( manager_refstate_ );
        }
        else
        {
          // be extra sure we have pushed reference down enough
          manager_refstate_.pd = arming_ned_(2) + 5.0;
          refstate_pub_->publish( manager_refstate_ );
          // send disarm command and freeze arbitration
          sendMavrosArmCommand( false );
          mission_mode_ = MissionMode::MISSION_END;
        }

        break;
      }

    case MissionMode::POLICY_EXEC :
      {
        // here we simply forward incoming reference state to controller
        forward_ref_state_ = true;

        // also set a timeout for target_state updates
        if( (t_clock_ - t_tgtstate_update_) > MISSION_WDG_TIMEOUT )
        {
          // capture current state, and switch to HOVER
          updateManagerRefNED( pos_ned_ );
          mission_mode_ = MissionMode::HOVERING;
          target_state_avail_ = false;
        }
        break;
      }
  }

  RCLCPP_INFO_THROTTLE( get_logger(), *(get_clock()), 1000,
                        "Mission Mode: %s", MissionModeName[(int)mission_mode_] );

}

int main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<ApmModeArbitrator>() );

  rclcpp::shutdown();
  return 0;
}