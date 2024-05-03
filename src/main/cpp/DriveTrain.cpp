
#include "DriveTrain.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

using namespace pathplanner;
using namespace tools;

// ----------------------------------------------------------------------------
//

DriveTrain::DriveTrain():logger("DriveTrain"), m_metrics("driveTrain"){}

// ----------------------------------------------------------------------------
//

bool DriveTrain::init(NT_Manager *nt, Feeder *feeder)
{
  m_nt = nt;
  m_feeder = feeder;
  assistance_drive = false;

#ifdef PATHPLANNER
  try
  {
    logger.log(LL_INFO, "AutoBuilder::configureRamsete");

    AutoBuilder::configureRamsete(
        [this]()
        { return getPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose)
        { resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]()
        { return getSpeed(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds)
        { setSpeed(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        ReplanningConfig(),    // Default path replanning config. See the API for the options here
        [this]()
        { return shouldFlipPath(); },
        this // Reference to this subsystem to set requirements
    );
  }
  catch (...)
  {
    // LogError << "exception" << std::endl;
    return false;
  }
#endif

  return true;
}

// ----------------------------------------------------------------------------
//

void DriveTrain::baseCollectMetrics(){
  
  double drive_L_avant = m_left_lead_motor.GetMotorTemperature();
  double drive_L_arriere = m_left_follow_motor.GetMotorTemperature();
  double drive_R_arriere = m_right_lead_motor.GetMotorTemperature();
  double drive_R_avant = m_right_follow_motor.GetMotorTemperature();
  double left_wheel_encoder = m_encoder_left_lead.GetPosition();
  double right_wheel_encoder = m_encoder_right_lead.GetPosition();
  double robot_yaw = m_pigeon.GetYaw();
  double robot_pitch = m_pigeon.GetPitch();
  double robot_roll = m_pigeon.GetRoll();

  m_metrics.publish("temp_m_left_front", drive_L_avant);
  m_metrics.publish("temp_m_left_rear", drive_L_arriere);
  m_metrics.publish("temp_m_right_rear", drive_R_arriere);
  m_metrics.publish("temp_m_right_front", drive_R_avant);
  m_metrics.publish("left_drive_encoder", left_wheel_encoder);
  m_metrics.publish("right_drive_encoder", right_wheel_encoder);
  m_metrics.publish("robot_yaw", robot_yaw);
  m_metrics.publish("robot_pitch", robot_pitch);
  m_metrics.publish("robot_roll", robot_roll);
  m_metrics.publish("left_speed_auto", left_speed_auto);
  m_metrics.publish("right_speed_auto", right_speed_auto);

  //std::cout <<"left_wheel_encoder: " << left_wheel_encoder << "\n";
  //std::cout <<"right_wheel_encoder: " << right_wheel_encoder << "\n";
}
// ----------------------------------------------------------------------------
//

void DriveTrain::flushMetrics(){

  m_metrics.flush();

}
// ----------------------------------------------------------------------------
//

void DriveTrain::handelPigeonValue(){

  double robot_yaw_value = m_pigeon.GetYaw(); 
  double robot_pitch_value = m_pigeon.GetPitch();
  double robot_roll_value = m_pigeon.GetRoll();

  frc::SmartDashboard::PutNumber("yaw", robot_yaw_value);
  frc::SmartDashboard::PutNumber("pitch", robot_pitch_value);
  frc::SmartDashboard::PutNumber("roll", robot_roll_value);
} 

// ----------------------------------------------------------------------------
//
void DriveTrain::wheelEncoder(){

  double left_wheel_encoder = m_encoder_left_lead.GetPosition();
  double right_wheel_encoder = m_encoder_right_lead.GetPosition();
 
  m_nt->left_wheel_encoder_pub.Set(left_wheel_encoder);
  m_nt->right_wheel_encoder_pub.Set(right_wheel_encoder);
}

// ----------------------------------------------------------------------------
//

void DriveTrain::handleTaskDriveTrainAuto()
{
  handleShowPiValue();
  handleMotorBaseTemp();
  if (m_feeder->getFeederState() == Feeder::FeederState::Suck){
    handleDriveAuto();
  }
  
  
}

// ----------------------------------------------------------------------------
//
void DriveTrain::handleTaskDriveTrainTeleop()
{  
  
  handleShowPiValue();
  
  if (!assistance_drive){
    handleTaskBaseTeleop();
    buttonAssistanceDrive();
    //logger.log(LL_INFO, "manuel");
  }else if (m_feeder->getFeederState() == Feeder::FeederState::Suck){
    assistanceDrive();
    buttonAssistanceDriveStop();
    #ifdef DEBUG_DRIVE_TRAIN
      logger.log(LL_INFO, "assist");
    #endif
  }else{
    assistance_drive = false;  
    #ifdef DEBUG_DRIVE_TRAIN  
      logger.log(LL_INFO, "assist off");  
    #endif
  }
  
}

// ----------------------------------------------------------------------------
//
void DriveTrain::handleTaskDriveTrainInit()
{
  baseInit();
}

// ----------------------------------------------------------------------------
//

void DriveTrain::assistanceDrive(){
  if (assistance_drive){
    handleDriveAuto();
  }
}
// ----------------------------------------------------------------------------
//

void DriveTrain::buttonAssistanceDrive(){
  a_second_controller = m_second_controller.GetAButton();
  if (a_second_controller && !assistance_drive){
    assistance_drive = true;
    #ifdef DEBUG_DRIVE_TRAIN
      logger.log(LL_INFO, "assist on");
    #endif
  }
}
// ----------------------------------------------------------------------------
//

void DriveTrain::buttonAssistanceDriveStop(){
  b_second_controller = m_second_controller.GetBButton();
  if (b_second_controller && assistance_drive){
    assistance_drive = false;
    #ifdef DEBUG_DRIVE_TRAIN
      logger.log(LL_INFO, "button stop assit");
    #endif
  }
}
// ----------------------------------------------------------------------------
//

void DriveTrain::handleDriveTeleop()
{
  maxSpeedTele();
  handleSensitivityStickLeftX();
  handleSensitivityStickLeftY();

  //x_final = m_controller.GetLeftX();
  //y_final = m_controller.GetLeftY();

  if (x_final > max_speed_tele)
  {
    x_final = max_speed_tele;
  }
  if (y_final > max_speed_tele)
  {
    y_final = max_speed_tele;
  }
  if (x_final < -max_speed_tele)
  {
    x_final = -max_speed_tele;
  }
  if (y_final < -max_speed_tele)
  {
    y_final = -max_speed_tele;
  }
  frc::SmartDashboard::PutNumber("maxSpeed", max_speed_tele);
  m_robotDrive.ArcadeDrive(x_final, -y_final);

}

// ----------------------------------------------------------------------------
//
void DriveTrain::maxSpeedTele()
{
  pov = m_second_controller.GetPOV();

  if (pov == 90 && max_speed_tele < 1)
  {
    max_speed_tele += 0.01;
  }
  if (pov == 270 && max_speed_tele > 0.1)
  {
    max_speed_tele -= 0.01;
  }
}

// ----------------------------------------------------------------------------
//
void DriveTrain::handleSensitivityStickLeftX()
{
  x = m_controller.GetLeftX();

  if (x <= IX && x >= 0)
  {
    x_final = x - (((IX - x) * FX) * x);
  }
  else if (x > IX && x >= 0)
  {
    x_final = x;
  }
  else if (x >= -IX && x <= 0)
  {
    x_final = -(abs(x) - (((IX - abs(x)) * FX) * abs(x)));
  }
  else if (x < -IX && x <= 0)
  {
    x_final = x;
  }
}

// ----------------------------------------------------------------------------
//

void DriveTrain::handleSensitivityStickLeftY()
{
  y = m_controller.GetLeftY();

  if (y <= IY && y >= 0)
  {
    y_final = y - (((IY - y) * FY) * y);
  }
  else if (y > IY && y >= 0)
  {
    y_final = y;
  }
  else if (y >= -IY && y <= 0)
  {
    y_final = -(abs(y) - (((IY - abs(x)) * FY) * abs(y)));
  }
  else if (y < -IY && y <= 0)
  {
    y_final = y;
  }
}

// ----------------------------------------------------------------------------
//

void DriveTrain::antiPatinage(){
  y = m_controller.GetLeftY();
  
  if (y > 0){
    speed = prev_speed + MAX_ACCELERATION_VALUE;
    prev_speed = speed;
  } 


}

// ----------------------------------------------------------------------------
//
void DriveTrain::handleMotorBaseTemp()
{
  double motor_left_ID10 = m_left_lead_motor.GetMotorTemperature();
  double motor_left_ID18 = m_left_follow_motor.GetMotorTemperature();
  double motor_right_ID1 = m_right_lead_motor.GetMotorTemperature();
  double motor_right_ID6 = m_right_follow_motor.GetMotorTemperature();

  frc::SmartDashboard::PutNumber("mLeftID10", motor_left_ID10);
  frc::SmartDashboard::PutNumber("mLeftID18", motor_left_ID18);
  frc::SmartDashboard::PutNumber("mRightID1", motor_right_ID1);
  frc::SmartDashboard::PutNumber("mRightID6", motor_right_ID6);

}

// ----------------------------------------------------------------------------
//
void DriveTrain::handleTaskBaseTeleop()
{
  handleDriveTeleop();
  handleMotorBaseTemp();
}

// ----------------------------------------------------------------------------
//
void DriveTrain::baseInit()
{
  camera1 = frc::CameraServer::StartAutomaticCapture(0);

  m_left_follow_motor.RestoreFactoryDefaults();
  m_left_lead_motor.RestoreFactoryDefaults();
  m_right_follow_motor.RestoreFactoryDefaults();
  m_right_lead_motor.RestoreFactoryDefaults();

  m_left_follow_motor.Follow(m_left_lead_motor);
  m_right_follow_motor.Follow(m_right_lead_motor);

  max_speed_tele = MAX_SPEED_TELEOP_INIT; 
}

// ----------------------------------------------------------------------------
//
void DriveTrain::handleDriveAuto()
{

  left_speed_auto = m_nt->left_wheel_speed_percent;
  right_speed_auto = m_nt->right_wheel_speed_percent;

  if (left_speed_auto > DRIVE_MAX_SPEED_AUTO)
  {
    left_speed_auto = DRIVE_MAX_SPEED_AUTO;
  }
  if (right_speed_auto > DRIVE_MAX_SPEED_AUTO)
  {
    right_speed_auto = DRIVE_MAX_SPEED_AUTO;
  }
  if (left_speed_auto < -DRIVE_MAX_SPEED_AUTO)
  {
    left_speed_auto = -DRIVE_MAX_SPEED_AUTO;
  }
  if (right_speed_auto < -DRIVE_MAX_SPEED_AUTO)
  {
    right_speed_auto = -DRIVE_MAX_SPEED_AUTO;
  }

  m_left_lead_motor.Set(-left_speed_auto);
  m_right_lead_motor.Set(right_speed_auto);
}

// ----------------------------------------------------------------------------
//

void DriveTrain::goForward(){
  m_left_lead_motor.Set(0.1);
  m_right_lead_motor.Set(-0.1);
}

// ----------------------------------------------------------------------------
//

void DriveTrain::stop(){
  m_left_lead_motor.Set(0);
  m_right_lead_motor.Set(0);
}

// ----------------------------------------------------------------------------
//

void DriveTrain::handleShowPiValue()
{

  frc::SmartDashboard::PutNumber("ringDetected", m_nt->ring_detected);
  frc::SmartDashboard::PutNumber("tagDetected", m_nt->tag_detected);
  frc::SmartDashboard::PutNumber("tagID", m_nt->tag_id);
}

// ----------------------------------------------------------------------------
//
frc::Pose2d DriveTrain::getPose()
{
#ifdef DEBUG
    logger.log(LL_INFO, "");
#endif
    return m_pose2d;
}

// ----------------------------------------------------------------------------
//
void DriveTrain::resetPose(frc::Pose2d pose)
{
#ifdef DEBUG
    logPose(pose);
#endif

    m_pose2d = pose;
}

// ----------------------------------------------------------------------------
//
void DriveTrain::setSpeed(frc::ChassisSpeeds speed)
{
    // speed in mps
    auto [left, right] = m_kinematics.ToWheelSpeeds(speed);

#ifdef DEBUG
    logSpeed(speed);
    // std::string str = "left:" + std::to_string(left.value());
    // str += " right:" + std::to_string(-right.value());
    // logger.log(LL_INFO, str.c_str());
#endif

    double left_pwm = getWheelPWM(left.value());
    double right_pwm = getWheelPWM(right.value());

    m_left_lead_motor.Set(-left_pwm);
    m_right_lead_motor.Set(right_pwm);

    // frc::ChassisSpeeds targetSpeeds = frc::ChassisSpeeds::Discretize(speed, 0.02_s);
    // logSpeed(targetSpeeds);

    m_speed = speed;
}

// ----------------------------------------------------------------------------
//
frc::ChassisSpeeds DriveTrain::getSpeed()
{
    double left_position = m_encoder_left_lead.GetPosition();
    double right_position = m_encoder_right_lead.GetPosition();

#ifdef DEBUG
    logger.log(LL_INFO, "");

    std::string str = "left pos:" + std::to_string(left_position);
    str += " right pos:" + std::to_string(right_position);

    logger.log(LL_INFO, str.c_str());
#endif

    return m_speed;
}

// ----------------------------------------------------------------------------
// speed is in meter per second (m/s)
double DriveTrain::getWheelPWM(double speed)
{
    try
    {
        double pwm = 0.0;
        double rpm = 0.0;
        double speed_ipm = speed;

        // convert speed to meter per minute
        speed_ipm *= 60.0;
        // convert to feet per minutes
        speed_ipm *= 39.37;
        // RPM = (speed / wheel circ ) * gear ration
        rpm = (speed_ipm / WHEEL_CIRC_IN ) * MOTOR_GR;

        if (abs(rpm) > MOTOR_MAX_RPM) {
            rpm < 0 ? rpm = -MOTOR_MAX_RPM : rpm = MOTOR_MAX_RPM;
        }

        pwm = rpm / MOTOR_MAX_RPM;

#ifdef DEBUG
    std::string str = "speed m/s:" + std::to_string(speed);
    str += " speed in/min:" + std::to_string(speed_ipm);
    str += " RPM:" + std::to_string(rpm);
    str += " PWM:" + std::to_string(pwm);

    logger.log(LL_INFO, str.c_str());
#endif

        return pwm;
    }
    catch (...)
    {
        logger.log(LL_ERROR, "Exception");
        return 0.0;
    }
}

// ----------------------------------------------------------------------------
//
void DriveTrain::logPose(frc::Pose2d pose)
{
#ifdef DEBUG
    auto x = pose.X().value();
    auto y = pose.Y().value();

    std::string str = "x:" + std::to_string(x);
    str += " y:" + std::to_string(y);

    logger.log(LL_INFO, str.c_str());
#endif
}

// ----------------------------------------------------------------------------
//
void DriveTrain::logSpeed(frc::ChassisSpeeds speed)
{
#ifdef DEBUG
    auto vx = speed.vx.value();       // forward velocity
    auto vy = speed.vy.value();       // sideways velocity
    auto omega = speed.omega.value(); // angular velocity

    std::string str = "vx:" + std::to_string(vx);
    str += " vy:" + std::to_string(vy);
    str += " omega:" + std::to_string(omega);

    logger.log(LL_INFO, str.c_str());
#endif
}

// ----------------------------------------------------------------------------
//
bool DriveTrain::shouldFlipPath()
{
    // Boolean supplier that controls when the path will be mirrored for the red alliance
    // This will flip the path being followed to the red side of the field.
    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    return isAllianceRed();
}

// ----------------------------------------------------------------------------
//
bool DriveTrain::isAllianceRed()
{
    auto alliance = frc::DriverStation::GetAlliance();

    if (alliance)
    {
        // LogInfo << alliance.value() << std::endl;
#ifdef DEBUG
        alliance.value() == frc::DriverStation::Alliance::kRed ? logger.log(LL_INFO, "red") : logger.log(LL_INFO, "blue");
#endif

        return alliance.value() == frc::DriverStation::Alliance::kRed;
    }

    // LogWarning << "no alliance" << std::endl;

    return false;
}

// ----------------------------------------------------------------------------
//
int DriveTrain::getAllianceLocation()
{
    std::optional<int> location = frc::DriverStation::GetLocation();

    if (location)
    {
#ifdef DEBUG
        logger.log(LL_INFO, std::to_string(*location).c_str());
#endif
        return *location;
    }

    logger.log(LL_WARNING, "no location");

    return 0;
}
