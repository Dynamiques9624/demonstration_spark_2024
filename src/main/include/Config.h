
// possition du m_controller sur driver station
#define CONTROLLER_PORT_NO 0

#define SECOND_CONTROLLER_PORT_NO 1

#define POWER_DISTRIBUTION_ID 1

//PWM pin led strip
#define LED_STRIP_PWM 2

//led strip led number
#define KLENGTH 150

//something for networkTable
#define TOPIC "/autonomous"

//DRIVE---------------------------------------------------------

#define MAX_SPEED_TELEOP_INIT 0.43

//Can id des moteur du drivetrain
#define LEFT_LEAD_MOTOR_ID 10
#define LEFT_FOLLOW_MOTOR_ID 18
#define RIGHT_LEAD_MOTOR_ID 9
#define RIGHT_FOLLOW_MOTOR_ID 6 

#define PIGEON_CAN_NUMBER 16

//max speed during autonumus 0 to 1
#define DRIVE_MAX_SPEED_AUTO 1

//value for the sensibility of turning
//value increment and limitation
#define IX 0.6
//multiplicateur 
#define FX 0.5

//value for the sensibility of driving
//value increment and limitation
#define IY 0.4
//multiplicateur 
#define FY 1

#define MAX_ACCELERATION_VALUE 0.2

//BASCUL--------------------------------------------------------

//digital pin des deux encoder
#define ENCODER_BASCUL 0

//valeur de la bascul la plus haute (60 degre et position de depart base)
#define POS_BASE_VALUE_BASCUL 95 //-81.5 //-20
//valeur de la bascul le plus bas (pointe vers le bas)
#define POS_UP_VALUE_BASCUL -10 //-172 //-95

//Can id motor bascule
#define MOTOR_BASCUL_LEFT 19
#define MOTOR_BASCUL_RIGHT 20

//position of the bascul when whant to scorre in the amp
#define BASCUL_VALUE_AMP 25//-151.5 //-90 

//position of the bascul when want the feeder to go in amp position
#define BASCUL_VALUE_DEPLOY_FEEDER_AMP 65 //80 //-96.5 //-35

#define BASCUL_AMP_GOING_DOWN_SPEED 0.30
#define BASCUL_SPEED_DOWN_IDLE 0.30

// en seconde
#define CHRONO_RELAY_SECURITE 115 //115

//PROPULSUER---------------------------------------------------------

//Can id motteur propulseur 
#define MOTOR_LANCER_AN_RIGHT 3 
#define MOTOR_LANCER_AN_LEFT 2

//Time before shooting in autonumus mode in seconds
#define WAIT_TIME_BEFORE_SHOOTING 4

//Time for shooting in autonumus mode in seconds
#define WAIT_TIME_SHOOTHING 1

#define RELAY_PIN_DIO 5

#define LANCER_SPEED 0.3

//FEEDER---------------------------------------------------------

//digital pin des deux encoder
#define ENCODER_FEEDER 1

//Can id motteur position feeder
#define MOTOR_FEEDER_LEFT 5
#define MOTOR_FEEDER_RIGHT 17

//valeur de encoder du feeder
//feeder down
#define ENCODER_FEEDER_TAKE_VALUE 300 //180 valeur peut etre
//feeder up
#define ENCODER_FEEDER_LANCER_VALUE 112 //-4 val depart avant changement -3

//angle when the feader is vertical
#define ENCODER_FEEDER_VERTICAL_ANGLE 96

//Speed 0 a 1 pour aspirer anneau
#define INTAKE_SPEED_SUCK 0.4

//speed of the intake to push the ring
#define INTAKE_PUSH_SPEED -0.5

// ----------------------------------------------------------------------------
//Speed 0 a 1 pour motor_feeder
#define FEEDER_DOWN_SPEED -0.60 ///////-0.4

#define FEEDER_DOWN_MAX_SPEED -0.30

#define FEEDER_SPEED_DOWN_INC -0.10
// ----------------------------------------------------------------------------

//Speed 0 a 1 pour motor_feeder
#define FEEDER_UP_MAX_SPEED 0.60///////0.4

//increment pour la vitesse de la remonter du feeder
#define FEEDER_SPEED_UP_INC 0.07 ////// 0.2

//value for the division for the speed of the feeder
#define ENCODER_FEEDER_SPEED_DIV ((ENCODER_FEEDER_TAKE_VALUE-ENCODER_FEEDER_VERTICAL_ANGLE)/5)
// ----------------------------------------------------------------------------

#define FEEDER_UP_SPEED_AMP 0.50

//speed of feeder when going in amp position
#define FEEDER_DOWN_AMP_MAX_SPEED -0.27 ///// -0.27

#define FEEDER_SPEED_AMP_INC 0.135 //////0.1

//digital pin limit switch feeder
#define LIMIT_SWITCH 2 

//Can id motor roue feeder intake
#define MOTOR_FEEDER_INTAKE 4

//Position of the feeder when drop ring in amp
#define AMP_POSITION_FEEDER_DOWN 353 //233

//value for when the bascul go back to normal position after amp
#define FEEDER_VALUE_DEPLOY_BASCUL_AMP 307 //187

//value for shake of the ring
#define ENCODER_FEEDER_STOP_SHAKE_VALUE (ENCODER_FEEDER_LANCER_VALUE + 70)
#define SHAKE_RING_OUT_TIME 30
#define SHAKE_RING_INTAKE 0.1


//configue pour pathplanner
#define WHEEL_DISTANCE 0.559_m
#define MOTOR_MAX_RPM 3000 // 5310
#define MOTOR_GR 10.71
#define WHEEL_DIAM_IN 6.0
#define WHEEL_CIRC_IN 18.85

// define pour les debug dans les logs

//#define DEBUG_DRIVE_TRAIN
#define DEBUG_FEEDER

#define DEBUG


//define pour utiliser les fonctionnaliter pathplanner 
//#define PATHPLANNER
