#pragma once

#include "Config.h"
#include "Top.h"
#include "Feeder.h"
#include "Nt_manager.h"
#include "DriveTrain.h"
#include <Metrics.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>
#include <frc/Timer.h>

#include <math.h>
#include <time.h>
#include <iostream>

#include <RobotContainer.h>
#include <FileLogger.h>

using namespace std;

class Base : public frc::TimedRobot, Top
{
public:
  Base();
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;  
  
protected:
  tools::FileLogger     logger{"Base"};

  enum AutoState{
    Idle, FirstNote, DropFeeder, SuckNote, GoForward, SecondNote, AutoLeave
  };
  AutoState m_auto_state;

  void setState(AutoState state);
  
private:

  bool timer_started_go_forward;
  time_t start_go_forward;
  
  DriveTrain            m_driveTrain;
  bool                  m_autonomous;
  bool                  collect_periodic_metrics; 
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  RobotContainer        m_container;
  Metrics               m_metrics;

  frc::PowerDistribution power_distribution{POWER_DISTRIBUTION_ID, frc::PowerDistribution::ModuleType::kRev};

  void startTimerGoForward();
  void metricsInGame();
  void MetricsGetPowerInfo();
};
