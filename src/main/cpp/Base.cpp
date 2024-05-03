
#include "Base.h"


// ----------------------------------------------------------------------------
//

Base::Base():logger("Base"), m_metrics("base"){}

// ----------------------------------------------------------------------------
//
void Base::RobotInit()
{
  m_autonomous = false;
  m_driveTrain.init(&m_nt, this);
  handleTopInit();
  m_driveTrain.handleTaskDriveTrainInit();
  m_nt.ntManagerInit();

  setState(AutoState::Idle);
}

// ----------------------------------------------------------------------------
//

void Base::RobotPeriodic()
{
#ifdef PATHPLANNER
  frc2::CommandScheduler::GetInstance().Run();
#endif
  if (collect_periodic_metrics){
    MetricsGetPowerInfo();
  }
  

}

// ----------------------------------------------------------------------------
//

void Base::DisabledInit()
{

  m_nt.autonomous_pub.Set(0);
  m_nt.teleop_mode_pub.Set(0);
  m_nt.handleSubscriberTask();
  relay_activited = false;
  collect_periodic_metrics = false;
  m_metrics.flush();
  m_driveTrain.flushMetrics();
  flushFeederMetrics();
  flushTopMetrics();

}

// ----------------------------------------------------------------------------
//
void Base::AutonomousInit()
{

#ifdef PATHPLANNER
  m_autonomousCommand = m_container.getAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
#endif

  collect_periodic_metrics = true;
  m_autonomous = true;
  logger.log(LL_NOTICE, "enter autonomous");


  m_nt.autonomous_pub.Set(0);
  m_nt.teleop_mode_pub.Set(1);
  
  first_note_shoot_append = false;
  secondNoteAuto = false;

  timer_started_go_forward = false;

  setState(AutoState::FirstNote);
}

// ----------------------------------------------------------------------------
//
void Base::AutonomousPeriodic()
{
  time_t delay_go_forward;
  m_nt.handleSubscriberTask();
  metricsInGame();
  m_driveTrain.wheelEncoder();
  feederHandler();

  switch (m_auto_state)
  {
  case Idle:
    break;
  
  case FirstNote:
    if (m_nt.pos_value_auto == 1 || m_nt.pos_value_auto == 2 || m_nt.pos_value_auto == 3){
      handleTopTaskAuto();
      if (first_note_shoot_append)
      {
        if (m_nt.pos_value_auto == 2)
        {
          setState(AutoState::DropFeeder);
        }
        else
        {
          setState(AutoState::Idle);
        }
      }
    }
    else{
      setState(AutoState::Idle);
    }
    break;

  case DropFeeder:

    if (getFeederState() == FeederState::Idle)
    {
      Feeder::setState(FeederState::GoDown);
      
    }
    else if (getFeederState() == FeederState::Suck)
    {
      setState(AutoState::SuckNote);
    }
    break;

  case SuckNote:
    m_driveTrain.handleTaskDriveTrainAuto();
    std::cout <<"driveauto " << "driveauto" << "\n";

    if (getFeederState() != FeederState::Suck)
    {
      setState(AutoState::GoForward);
    }
    break;

  case GoForward:
    startTimerGoForward();
    m_driveTrain.goForward();
    delay_go_forward = time(NULL) - start_go_forward;

    if (delay_go_forward >= 3)
    {
      m_driveTrain.stop();
      timer_started_propul = false;
      timer_started_fire = false;
      first_note_shoot_append = false;
      setState(AutoState::SecondNote);
    }
    break;

  case SecondNote:
    secondNoteAuto = true;
    handleTopTaskAuto();
    if (first_note_shoot_append)
    {
      setState(AutoState::Idle);
    }
    break;
/*
  case AutoLeave:
    if ()
    
    break;
*/ 
  default:
    break;
  }
}

// ----------------------------------------------------------------------------
//
void Base::TeleopInit()
{
// This makes sure that the autonomous stops running when
// teleop starts running. If you want the autonomous to
// continue until interrupted by another command, remove
// this line or comment it out.
#ifdef PATHPLANNER
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
#endif

  m_nt.teleop_mode_pub.Set(1);
  m_nt.autonomous_pub.Set(0);
  collect_periodic_metrics = true;
}

// ----------------------------------------------------------------------------
//
void Base::TeleopPeriodic()
{
  if (m_autonomous)
  {
    m_autonomous = false;
    logger.log(LL_NOTICE, "exit autonomous");
  }

  metricsInGame();
  m_driveTrain.handelPigeonValue();
  m_driveTrain.wheelEncoder();
  m_nt.handleSubscriberTask();
  m_driveTrain.handleTaskDriveTrainTeleop();
  handleTopTaskTeleop();
}

// ----------------------------------------------------------------------------
//

void Base::TestInit()
{
  m_nt.teleop_mode_pub.Set(0);
  m_nt.autonomous_pub.Set(0);
}

// ----------------------------------------------------------------------------
//

void Base::TestPeriodic()
{
}

// ----------------------------------------------------------------------------
//

void Base::setState(AutoState state)
{
  // std::cout <<"auto_state " << state << "\n";
  frc::SmartDashboard::PutNumber("AutoState", state);
  m_auto_state = state;
}

// ----------------------------------------------------------------------------
//

void Base::startTimerGoForward()
{
  if (!timer_started_go_forward)
  {
    time(&start_go_forward);
    timer_started_go_forward = true;
  }
}

// ----------------------------------------------------------------------------
//

void Base::metricsInGame(){

  m_driveTrain.baseCollectMetrics();
  feederCollectMetrics();
  topCollectMetrics();

}

// ----------------------------------------------------------------------------
//

void Base::MetricsGetPowerInfo(){

  double current = power_distribution.GetTotalCurrent();
  double voltage = power_distribution.GetVoltage();

  m_metrics.publish("current", current);
  m_metrics.publish("voltage", voltage);

}

// ----------------------------------------------------------------------------
//



