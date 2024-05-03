#pragma once

#include"Config.h"
#include "Feeder.h"
#include <Metrics.h>
#include <FileLogger.h>

#include <frc/XboxController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/DigitalOutput.h>

#include <rev/CANSparkMax.h>

#include <cmath> 
#include <time.h>
#include <iostream>
//using namespace std;

class Top : protected Feeder{
 public:

 protected:
  enum AmpState{
    Idle, BasculUp, BasculGoingUp, FeederUp, Loaded, Fire, FeederDown, BasculDown, BasculGoingDown
  };
  AmpState amp_state;

  Top();
  void setState(AmpState state);

  void handleTopInit();
  void handleTopAutoInit();
  void handleEncoderValue();
  void handleMotorTemp();
  
  void ampHandler();
  void PositionBascul();
  void basculHandler();
  void handleTopTaskTeleop();
  void handleTopTaskAuto();
  void handlelancerSpeed();
  void lanceurAutoOneNote();
  void lanceurAutoDeuxiemeNote();

  void handleRelaySolo();

  void topCollectMetrics();

  void flushTopMetrics();

  bool relay_activited;
  bool solo_activited;
  double bascul_value;
  bool first_note_shoot_append;
  bool secondNoteAuto;

  bool timer_started_propul;
  time_t start_propul;

  bool timer_started_fire;
  time_t start_fire;

 private:
    int nb_shoot_amp;

    bool bascul_target_position;

    bool connect_encoder_lancer;

    bool y_button_handler;

    double y_button;

    double feeder_speed;
    double intake_speed;
    double bascul_down_speed;
    
    double angle_encoder_lancer;
    bool RB;
    bool LB;
    double RT;
    double LT;
    int pov;

    int pov_second;
    
    double bascul_value_aprox;

    bool set_predefine_loaded;

    bool timer_started_relay;
    time_t start_relay;

    bool securite_relay;
    bool timer_started_relay_securite;
    time_t start_relay_securite;

    tools::FileLogger     logger{"Top"};
    Metrics m_metrics;

    frc::XboxController m_controller{CONTROLLER_PORT_NO};
    frc::XboxController m_second_controller{SECOND_CONTROLLER_PORT_NO};
        
    frc::DutyCycleEncoder m_dutyCycleEncoder_lancer{ENCODER_BASCUL};

    rev::CANSparkMax motor_lancer_an_left{MOTOR_LANCER_AN_LEFT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax motor_lancer_an_right{MOTOR_LANCER_AN_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    
    rev::CANSparkMax motor_bascul_left{MOTOR_BASCUL_LEFT, rev::CANSparkMax::MotorType::kBrushless}; 
    rev::CANSparkMax motor_bascul_right{MOTOR_BASCUL_RIGHT, rev::CANSparkMax::MotorType::kBrushless};

    frc::DigitalOutput relay_solo{RELAY_PIN_DIO};

    void yButtonHandler();

    void basculUp();
    void basculIdle();
    void basculGoingUp();
    void feederUp(bool update_state = true);
    void feederLoaded();
    void feederFire();
    void feederDown();
    void basculDown(bool update_state = true);
    void basculGoingDown();

    void startTimerPropul();
    void startTimerFire();
    void startTimerRelay();
    void startTimerRelaySecurite();

    void lanceurAutoFirstOfTwo();
    void lanceurAutoTwoOfTwo();

};  