#include "Top.h"

// ----------------------------------------------------------------------------
//

Top::Top():logger("Top"), m_metrics("top"){}

// ----------------------------------------------------------------------------
//

void Top:: handleTopInit(){
    feederInit();

    m_dutyCycleEncoder_lancer.SetDistancePerRotation(360);

    motor_lancer_an_left.RestoreFactoryDefaults();
    motor_lancer_an_right.RestoreFactoryDefaults();

    motor_bascul_left.RestoreFactoryDefaults();
    motor_bascul_right.RestoreFactoryDefaults();

    bascul_value = POS_BASE_VALUE_BASCUL;

    lancer_speed_value = 0.7;
   
    set_predefine_loaded = false;
    nb_shoot_amp = 0;

    timer_started_propul = false;
    timer_started_fire = false;
    timer_started_relay = false;
    timer_started_relay_securite = false;

    bascul_target_position = false;

    securite_relay = true;

    relay_activited = false;
    
    y_button_handler = false;

    solo_activited = false;
  
}

// ----------------------------------------------------------------------------
//

void Top::handleTopAutoInit(){

    
}

// ----------------------------------------------------------------------------
//

void Top::topCollectMetrics(){

    double temp_m_lancer_left = motor_lancer_an_left.GetMotorTemperature();
    double temp_m_lancer_right = motor_lancer_an_right.GetMotorTemperature();
    double temp_m_bascul_left =  motor_bascul_left.GetMotorTemperature();
    double temp_m_bascul_right =  motor_bascul_right.GetMotorTemperature();
    double angle_encoder_lancer = m_dutyCycleEncoder_lancer.GetDistance();

    m_metrics.publish("temp_m_lancer_left", temp_m_lancer_left);
    m_metrics.publish("temp_m_lancer_right", temp_m_lancer_right);
    m_metrics.publish("temp_m_bascul_left", temp_m_bascul_left);
    m_metrics.publish("temp_m_bascul_right", temp_m_bascul_right);
    m_metrics.publish("angle_encoder_lancer", angle_encoder_lancer);
    m_metrics.publish("amp_state", amp_state);
    m_metrics.publish("bascul_value", bascul_value);

}

// ----------------------------------------------------------------------------
//

void Top::flushTopMetrics(){

  m_metrics.flush();

}

// ----------------------------------------------------------------------------
//

void Top::handleEncoderValue(){
    
    connect_encoder_lancer = m_dutyCycleEncoder_lancer.IsConnected();
    angle_encoder_lancer = m_dutyCycleEncoder_lancer.GetDistance();
    frc::SmartDashboard::PutNumber("encoLancerVal", angle_encoder_lancer);
    frc::SmartDashboard::PutBoolean("encoder lancer", connect_encoder_lancer);
    
}
// ----------------------------------------------------------------------------
//

void Top::handleMotorTemp(){
    
    double temp_m_lancer_left = motor_lancer_an_left.GetMotorTemperature();
    double temp_m_lancer_right = motor_lancer_an_right.GetMotorTemperature();
    double temp_m_bascul_left =  motor_bascul_left.GetMotorTemperature();
    double temp_m_bascul_right =  motor_bascul_right.GetMotorTemperature();

    frc::SmartDashboard::PutNumber("tempMlancerLeft", temp_m_lancer_left);
    frc::SmartDashboard::PutNumber("tempMlancerRight", temp_m_lancer_right);
    frc::SmartDashboard::PutNumber("tempMbasculLeft", temp_m_bascul_left);
    frc::SmartDashboard::PutNumber("tempMbasculRight", temp_m_bascul_right);

}
// ----------------------------------------------------------------------------
//

void Top::PositionBascul(){
    pov = m_controller.GetPOV();
    LT = m_controller.GetLeftTriggerAxis();

    if (feeder_state != FeederState::Loaded && feeder_state != FeederState::Fire){
        set_predefine_loaded = true;
    }else if (set_predefine_loaded) {
        bascul_value = POS_BASE_VALUE_BASCUL-3; //bascul_value = POS_BASE_VALUE_BASCUL;
        set_predefine_loaded = false;
    }   

    double limit_base =  POS_BASE_VALUE_BASCUL;
    if (securite_relay == false){
        limit_base += 3;
    }

    if (LT == 1 && bascul_value < limit_base){
        bascul_value += 1.5;
    }
    if(pov == 0 && bascul_value > POS_UP_VALUE_BASCUL){
        bascul_value -=0.5;
    }
    basculHandler();

}
// ----------------------------------------------------------------------------
//

void Top::basculHandler(){

    bascul_value_aprox = abs(bascul_value)-abs(angle_encoder_lancer);

    frc::SmartDashboard::PutNumber("basculencoder", angle_encoder_lancer);
    frc::SmartDashboard::PutNumber("basculValue", bascul_value);
    frc::SmartDashboard::PutNumber("valueArox", bascul_value_aprox);  
    
    if(bascul_value > angle_encoder_lancer && bascul_value_aprox > -2){ 
        //bascule monte (pointe vers haut)
        motor_bascul_left.Set(-bascul_down_speed); //-0.08
        motor_bascul_right.Set(bascul_down_speed); //0.08
        
    }
    if(bascul_value < angle_encoder_lancer && bascul_value_aprox < 2){ 
        //bascule descend (point vers bas)
        motor_bascul_left.Set(0.22);
        motor_bascul_right.Set(-0.22);
        
    }

    bascul_target_position = false;

    if (bascul_value_aprox <= 2 && bascul_value_aprox >= 0){
        motor_bascul_left.Set(0.00001);
        motor_bascul_right.Set(-0.00001);
        bascul_target_position = true;
        
    }
    if (bascul_value_aprox >= -2 && bascul_value_aprox < 0 ){
        motor_bascul_left.Set(0.00001);
        motor_bascul_right.Set(-0.00001);
        bascul_target_position = true;
    }
}
// ----------------------------------------------------------------------------
//

void Top::handlelancerSpeed(){

    pov = m_controller.GetPOV();

    if (pov == 90 && lancer_speed_value != 1){

        lancer_speed_value += 0.01;
    
    }
    if (pov == 270 && lancer_speed_value != 0){

        lancer_speed_value -= 0.01;
    
    }

    if (lancer_speed_value <= 0){
        lancer_speed_value = 0;
            
    }else if (lancer_speed_value >= 1){
        lancer_speed_value = 1;
    }

    std::cout <<"lanceur speed " << lancer_speed_value << "\n";

    if(feeder_state == FeederState::GoUp || feeder_state == FeederState::Loaded || feeder_state == FeederState::Fire){
        
        if (amp_state == AmpState::Idle){

            motor_lancer_an_right.Set(-lancer_speed_value);  
            motor_lancer_an_left.Set(lancer_speed_value); 
            
        }else{
            motor_lancer_an_right.Set(0); 
            motor_lancer_an_left.Set(0); 
        }
    
    }
    else{
        motor_lancer_an_right.Set(0); 
        motor_lancer_an_left.Set(0);   
    }
    
}
// ----------------------------------------------------------------------------
//

void Top::yButtonHandler(){
    y_button = m_second_controller.GetYButton();

    if (y_button && !y_button_handler){
        y_button_handler = true;
    }else if (y_button_handler){
        y_button_handler = false;
    }
}
// ----------------------------------------------------------------------------
//

void Top::lanceurAutoOneNote(){

    motor_lancer_an_right.Set(-1);
    motor_lancer_an_left.Set(1);
    bascul_value = POS_BASE_VALUE_BASCUL-3; //bascul_value = POS_BASE_VALUE_BASCUL;
    startTimerPropul();
    time_t delay_propul = time(NULL) - start_propul;

    if( delay_propul >= WAIT_TIME_BEFORE_SHOOTING){
        feederFireAuto();
        startTimerFire();
        time_t delay_fire = time(NULL) - start_fire;
        if (delay_fire >= WAIT_TIME_SHOOTHING){
            Feeder::setState(FeederState::Idle);
            bascul_value = POS_BASE_VALUE_BASCUL;
            first_note_shoot_append = true;
        }
    } 

}

// ----------------------------------------------------------------------------
//

void Top::lanceurAutoDeuxiemeNote(){

    if (feeder_state == Loaded){

        motor_lancer_an_right.Set(-1);
        motor_lancer_an_left.Set(1);
        bascul_value = POS_BASE_VALUE_BASCUL-3; //bascul_value = POS_BASE_VALUE_BASCUL;
        startTimerPropul();
        time_t delay_propul = time(NULL) - start_propul;

        if( delay_propul >= WAIT_TIME_BEFORE_SHOOTING){
            feederFireAuto();
            startTimerFire();
            time_t delay_fire = time(NULL) - start_fire;
            if (delay_fire >= WAIT_TIME_SHOOTHING){
                Feeder::setState(FeederState::Idle);
                bascul_value = POS_BASE_VALUE_BASCUL;
                first_note_shoot_append = true;
            }
        } 
    }     

}

// ----------------------------------------------------------------------------
//

void Top::startTimerPropul(){
    
    if (!timer_started_propul){
        time(&start_propul);
        timer_started_propul = true;
    }
}

// ----------------------------------------------------------------------------
//

void Top::startTimerFire(){
    if (!timer_started_fire){
        time(&start_fire);
        timer_started_fire = true;
    }
}

// ----------------------------------------------------------------------------
//

void Top::handleTopTaskTeleop(){
    feederHandler();
    handleEncoderValue();
    handleMotorTemp();
    handlelancerSpeed();
    PositionBascul();
    ampHandler();
    handleRelaySolo();

}
// ----------------------------------------------------------------------------
//

void Top::handleTopTaskAuto(){
    
    handleEncoderValue();
    handlelancerSpeed();
    handleMotorTemp();
    feederHandler();
    PositionBascul();
    if (secondNoteAuto == false){
       lanceurAutoOneNote();  
    }
    if (secondNoteAuto == true){
       lanceurAutoDeuxiemeNote(); 
    }
    
}

// ----------------------------------------------------------------------------
//

void Top::handleRelaySolo(){

    pov_second = m_second_controller.GetPOV();
    startTimerRelaySecurite();
    time_t delay_relay_securite = time(NULL) - start_relay_securite;

    if (delay_relay_securite > CHRONO_RELAY_SECURITE){
        securite_relay = false;
    }    

    if (pov_second == 0 && securite_relay == false){
        if (!relay_activited){
            std::cout <<"relay " << "on" << "\n";
            relay_solo.Set(1);
            relay_activited = true;
            frc::SmartDashboard::PutNumber("relayState", relay_activited);
            if (!timer_started_relay){
                startTimerRelay();  
            }
        }
        
    }
    time_t delay_relay = time(NULL) - start_relay;
    if (delay_relay >= 5){
        relay_solo.Set(0);
        solo_activited = true;
        std::cout <<"relay " << "off" << "\n";
    }

}

// ----------------------------------------------------------------------------
//

void Top::startTimerRelay(){

    if (!timer_started_relay){
        time(&start_relay);
        timer_started_relay = true;
    }

}

// ----------------------------------------------------------------------------
//

void Top::startTimerRelaySecurite(){

    if (!timer_started_relay_securite){
        time(&start_relay_securite);
        timer_started_relay_securite = true;
    }

}

// ----------------------------------------------------------------------------
//

void Top::ampHandler(){
    frc::SmartDashboard::PutNumber("nb_shoot_amp", nb_shoot_amp);

    switch (amp_state)
    {
    case Idle:
        basculIdle();
        break;
    case BasculUp:
        basculUp();
        break;
    case BasculGoingUp:
        basculGoingUp();
        break;
    case FeederUp:
        feederUp();
        break;
    case Loaded:
        feederLoaded();
        break;
    case Fire:
        feederFire();
        break;
    case FeederDown:
        feederDown();
        break;
    case BasculDown:
        basculDown();
        break;
    case BasculGoingDown:
        basculGoingDown();
        break;

        
    
    default:
        break;
    }
}

// ----------------------------------------------------------------------------
//

void Top::basculUp(){
    bascul_value = BASCUL_VALUE_AMP;
    basculHandler();
    setState(AmpState::BasculGoingUp);

}

// ----------------------------------------------------------------------------
//

void Top::basculIdle(){
    LB = m_controller.GetLeftBumper();
    bascul_down_speed = BASCUL_SPEED_DOWN_IDLE;
    if(LB && feeder_state == FeederState::Loaded){
        setState(AmpState::BasculUp);

    }
}

// ----------------------------------------------------------------------------
//

void Top::basculGoingUp(){
    if (angle_encoder_lancer <= BASCUL_VALUE_DEPLOY_FEEDER_AMP){
        feederUp(false);
    }
    if (bascul_target_position){
        setState(AmpState::FeederUp);
    }
}

// ----------------------------------------------------------------------------
//

void Top::feederUp(bool update_state){
    Feeder::feederEncoderReader();
    if(angle_encoder_feeder < AMP_POSITION_FEEDER_DOWN){ 
        if(feeder_speed != FEEDER_DOWN_SPEED){
            feeder_speed = FEEDER_DOWN_SPEED;
            intake_speed = SHAKE_RING_INTAKE;
            setMotorFeeder(feeder_speed);
            setMotorIntake(intake_speed);

        }
        
    }else{
        feeder_speed = 0;
        //intake_speed = 0;
        setMotorFeeder(feeder_speed);
        //setMotorIntake(intake_speed);
        if (update_state){
          setState(AmpState::Loaded);  
        }
        

    }
}

// ----------------------------------------------------------------------------
//

void Top::feederLoaded(){
    RT = m_controller.GetRightTriggerAxis();

    if(RT != 1){
        return;
    }

    intake_speed = INTAKE_PUSH_SPEED;
    setMotorIntake(intake_speed);
    nb_shoot_amp = nb_shoot_amp + 1;
    setState(AmpState::Fire);
    
}

// ----------------------------------------------------------------------------
//

void Top::feederFire(){
    RT = m_controller.GetRightTriggerAxis();
    
    if(RT != 1){
        intake_speed = 0;
        setMotorIntake(intake_speed);
        setState(AmpState::FeederDown);
        
    }
}

// ----------------------------------------------------------------------------
//

void Top::feederDown(){
    Feeder::feederEncoderReader();
    if (angle_encoder_feeder <= FEEDER_VALUE_DEPLOY_BASCUL_AMP){
        basculDown(false);
    }
    if(angle_encoder_feeder > ENCODER_FEEDER_LANCER_VALUE){ 
        if(feeder_speed != FEEDER_UP_SPEED_AMP){
            feeder_speed = FEEDER_UP_SPEED_AMP;
            setMotorFeeder(feeder_speed);

        }
        
    }else{
        feeder_speed = 0;
        setMotorFeeder(feeder_speed);
        setState(AmpState::BasculDown);

    }
}

// ----------------------------------------------------------------------------
//

void Top::basculDown(bool update_state){
    bascul_value = POS_BASE_VALUE_BASCUL;
    bascul_down_speed = BASCUL_AMP_GOING_DOWN_SPEED;
    basculHandler();
    if (update_state){
      setState(AmpState::BasculGoingDown);  
    }
    
}

// ----------------------------------------------------------------------------
//

void Top::basculGoingDown(){
    if (bascul_target_position){
        setState(AmpState::Idle);
    
    }
}

// ----------------------------------------------------------------------------
//

void Top::setState(AmpState state){
    //std::cout <<"amp_state " << state << "\n";
    frc::SmartDashboard::PutNumber("AmpState", state);
    amp_state = state; 
}

// ----------------------------------------------------------------------------
//