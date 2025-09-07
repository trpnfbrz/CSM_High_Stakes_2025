#include "main.h"
#include "lemlib/api.hpp"

using namespace pros;

int
armPosition = 50,
armPosInt   = 0;

bool 
allianceColourIsRed = true,
enableArmPID = true,
enableLbLoading = false,
autoMode = false,
turboToggle = false;

// VEXcode device constructors

Controller Controller1(CONTROLLER_MASTER);

Motor  
	Intake    (7, v5::MotorGears::blue, v5::MotorUnits::degrees),
	armMotor(-14, v5::MotorGears::red , v5::MotorUnits::degrees);

ADIDigitalOut 
	doinkerPiston('A'),
	mogoPiston('B');

ADIDigitalIn lbBumper('C');

Optical optical(15);

// Drivetrain configuration
Imu imu(10); 

MotorGroup 
	leftMotors({-2, -4, 6}, MotorGearset::blue),
	rightMotors({1, 3, -5}, MotorGearset::blue);

lemlib::ExpoDriveCurve driveCurve(
  5,    // deadband
  12,   // min output
  1.025 // curve
);

lemlib::Drivetrain drivetrain(
	&leftMotors,
  &rightMotors,
  12.205,                     // 31cm track width
	lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
  600,                        // drivetrain rpm is 600
  2                           // horizontal drift is 2 (for now)
);

// horizontal tracking wheel encoder
Rotation odometryH(8);
// vertical tracking wheel encoder
Rotation odometryV(-9);
// horizontal tracking wheel
lemlib::TrackingWheel horizontalTrackingWheel(&odometryH, lemlib::Omniwheel::NEW_2, -2.5556); 
// vertical tracking wheel
lemlib::TrackingWheel verticalTrackingWheel(&odometryV, lemlib::Omniwheel::NEW_2, -0.2340); //keep right tracking wheel

lemlib::OdomSensors sensors(
	&verticalTrackingWheel,   // vertical tracking wheel 1, set to null
  nullptr,                  // vertical tracking wheel 2, set to nullptr as we are using IMEs
  &horizontalTrackingWheel, // horizontal tracking wheel 1
	nullptr,                  // horizontal tracking wheel 2, set to nullptr as we don't have a second one
  &imu                      // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateralController(
	  27.5,   // proportional gain (kP)
    2,   // integral gain (kI)
    180, // derivative gain (kD)
	  3,   // anti windup
    1,   // small error range, in inches
    100, // small error range timeout, in milliseconds
    3,   // large error range, in inches
    500, // large error range timeout, in milliseconds
    0    // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(
	  1.9, // proportional gain (kP)
    0, // integral gain (kI)
    6, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(
  drivetrain,
  lateralController,
  angularController,
  sensors
);

// this runs at the start of the program
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    chassis.calibrate(); // calibrate sensors

    // print position to brain screen
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f",        chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f",        chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f",    chassis.getPose().theta); // heading

            // print turbo/auto wall stake states
                 if( turboToggle &&  autoMode) Controller1.set_text(0, 0, "turboON |autoON ");
            else if( turboToggle && !autoMode) Controller1.set_text(0, 0, "turboON |autoOFF");
            else if(!turboToggle &&  autoMode) Controller1.set_text(0, 0, "turboOFF|autoON ");
            else if(!turboToggle && !autoMode) Controller1.set_text(0, 0, "turboOFF|autoOFF");

            // // print coords to controller screen
            // Controller1.set_text(0, 0, "x:" + std::to_string(chassis.getPose().x) + "Y:" + std::to_string(chassis.getPose().x));

            delay(20); // delay to save resources
        }
    });

    if(std::isnan(chassis.getPose().x) || std::isnan(chassis.getPose().y)) {
        chassis.calibrate();
    }
}

void autonomous() {
  armMotor.set_brake_mode(MOTOR_BRAKE_COAST);
          /*****FULL FIELD MATCH AUTON*****/
            // score alliance stake
            chassis.setPose(85.5, 10.375, -121);
            armMotor.move_absolute(175, 100);
            delay(1000);

            chassis.moveToPose(93.77, 23.08, 180, 2000, {.forwards = false, .minSpeed = 30});
            delay(500);
            armMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            armMotor.move_absolute(40, 100);

            // stake 1
            chassis.moveToPose(93.77, 48, 180, 4000, {.forwards = false, .minSpeed = 30});
            chassis.waitUntilDone();
            delay(500);
            mogoPiston.set_value(true);
            delay(1000);

            // ring 2
            Intake.move(127);
            chassis.turnToHeading(90, 2000, {.minSpeed = 60});
            chassis.moveToPose(117.33, 46.64, 90, 3000, {.minSpeed = 30});
            delay(2500);

            //go to touch bottom of ladder
            chassis.moveToPose(93.77, 46.64, 90, 3000, {.forwards = false, .minSpeed = 30});
            Intake.move(0);
            chassis.moveToPoint(80.28, 54.21, 2700, {.forwards = false, .maxSpeed = 50});
            armMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            delay(2000);




            // /*****RIGHT FIELD MATCH AUTON*****/
            // //start
            // chassis.setPose(95.85, 7, 180);

            // //right side of field (pt1)
            // chassis.moveToPoint(93.77, 46.64 - 6, 4000, {.forwards = false, .minSpeed = 60});
            // chassis.moveToPoint(93.77, 46.64, 4000, {.forwards = false, .maxSpeed = 60});
            // delay(300);
            // mogoPiston.set_value(true);
            
            // //right side of field (pt2)
            // chassis.turnToHeading(90, 2000, {.minSpeed = 80});
            // chassis.moveToPose(117.33, 46.64, 90, 3000, {.minSpeed = 80});
            // Intake.move(127);
            // delay(2000);

            // //alliance stake
            // chassis.moveToPose(70.20 - 2, 23.08 - 4, -90, 1500);
            // mogoPiston.set_value(false);
            // delay(2000);

            // //go to touch bottom of ladder
            // chassis.moveToPoint(70, 70, 2700, {.forwards = false, .maxSpeed = 60});
            // delay(400);
            // Intake.move(0);
            // delay(2000);




            /*****LEFT FIELD MATCH AUTON*****/
            // //start
            // chassis.setPose(48.64, 7, 180);

            // //right side of field (pt1)
            // chassis.moveToPoint(48.64, 46.64 - 6, 4000, {.forwards = false, .minSpeed = 60});
            // chassis.moveToPoint(48.64, 46.64, 4000, {.forwards = false, .maxSpeed = 40});
            // delay(500);
            // mogoPiston.set_value(true);
            
            // //right side of field (pt2)
            // chassis.turnToHeading(-90, 2000, {.minSpeed = 80});
            // chassis.moveToPose(23.08 + 3.5, 46.64, -90, 3000, {.minSpeed = 80});
            // Intake.move(127);
            // delay(1700);

            // //intake pile
            // chassis.turnToHeading(0, 2000, {.minSpeed = 80});
            // chassis.moveToPose(23.08 + 3.5, 59.5, 0, 1500);
            // delay(2000);

            // //go to touch bottom of ladder
            // chassis.moveToPose(70, 70, -135, 2700, {.forwards = false, .maxSpeed = 60});
            // delay(400);
            // delay(2000);
            // Intake.move(0);



            
            /*****SKILLS AUTON*****/
            // // initial position of the robot
            // chassis.setPose(70.20, 9.7, 0);
            // mogoPiston.set_value(false); 

            // //ring 0 onto red alliance stake
            // Intake.move(127);
            // delay(750);
            // Intake.move(0);

            // //stake 1 collection
            // chassis.moveToPose(70.20, 22, 0, 1400, {.minSpeed = 60}); 
            // chassis.turnToHeading(-90, 500, {.minSpeed = 60});
            // chassis.moveToPoint(90, 20, 800, {.forwards = false, .maxSpeed = 60});
            // chassis.moveToPoint(100, 20, 800, {.forwards = false});
            // delay(500);
            // mogoPiston.set_value(true);

            // //ring 1, ring 2
            // Intake.move(127); 
            // chassis.moveToPose(117.33, 23.08, 90, 2000);
            // delay(800);
            // chassis.moveToPoint(134, 23.08, 2000);
            // delay(1500);

            // //ring 3
            // chassis.moveToPoint(117.33 + 5, 34, 2000, {.forwards = false, .minSpeed = 60});
            // chassis.turnToHeading(180, 800);
            // chassis.moveToPoint(117.33 + 5, 8, 700);
            // delay(2000);
            // chassis.moveToPoint(117.33 + 5, 15, 800, {.forwards = false, .minSpeed = 60});

            // //ring 4
            // chassis.turnToHeading(0, 500, {.minSpeed = 80});
            // chassis.moveToPose(117.33 + 5, 46.64 + 3, 0, 2000, {.minSpeed = 80});
            // delay(1500);
            
            // //ring 5
            // chassis.turnToHeading(0, 500, {.minSpeed = 60});
            // chassis.moveToPoint(93.77, 46.64, 2000);
            // delay(1000);

            // //stake 1 in corner
            // chassis.turnToHeading(-45, 500, {.minSpeed = 60});
            // chassis.moveToPoint(135, 5, 1500, {.forwards = false});
            // delay(1000);
            // mogoPiston.set_value(false);
            // chassis.moveToPose(117.33, 20, -45, 1500, {.minSpeed = 60});

            // // get stake 2
            // chassis.turnToHeading(90, 500, {.minSpeed = 60});
            // chassis.moveToPose(50, 23.08 - 2, 90, 5000, {.forwards = false, .minSpeed = 59}); 
            // chassis.moveToPose(46.64, 23.08 - 2, 90, 5000, {.forwards = false, .maxSpeed = 60});
            // delay(700);
            // //comment out up to here for second half
            // mogoPiston.set_value(true);

            // // ring 6, 7
            // chassis.setPose(46.64, 23.08, 90); //checkpoint 1
            // Intake.move(127);
            // chassis.turnToHeading(-90, 500, {.minSpeed = 60});
            // chassis.moveToPose(23.08, 23.08, -90, 2000, {.maxSpeed = 60});
            // delay(1500);
            // chassis.moveToPose(8, 23.08, -90, 2000, {.maxSpeed = 60});
            // delay(1000);

            // //ring 8
            // chassis.moveToPose(23.08, 34, 180, 2000, {.forwards = false});
            // chassis.moveToPoint(23.08 - 3, 10, 2000, {.minSpeed = 80});
            
            // //ring 9
            // chassis.moveToPose(23.08, 23.08, 180, 1000, {.forwards = false, .minSpeed = 60}); // back up to turn around without hitting wall
            // chassis.turnToHeading(0, 500, {.minSpeed = 60});
            // chassis.moveToPose(23.08 - 3, 46.64 + 5, 0, 4000); // move to ring 9
            // delay(1500);
            // chassis.setPose(23.08, 46.64, 0); //checkpoint 2

            // //ring 10
            // chassis.moveToPose(8, 70.20, -90, 2000, {.minSpeed = 60});
            // delay(2000);

            // //ring 11
            // chassis.moveToPose(15, 70.20, -90, 2000, {.forwards = false, .minSpeed = 60});
            // chassis.moveToPoint(46.64, 46.64, 2000, {.minSpeed = 60});
            // delay(2000);
            // chassis.turnToHeading(45, 500, {.minSpeed = 60});

            // //depositing stake in corner
            // chassis.moveToPoint(4, 4, 3000, {.forwards = false});
            // delay(1000);
            // mogoPiston.set_value(false);

            // chassis.moveToPose(23.08, 70.20 + 4, 0, 4000);
            // delay(2000);
            // chassis.setPose(23.08, 70.20, 0); //checkpoint 3
            
            // //ring 11
            // Intake.move(127);
            // chassis.moveToPoint(23.08, 93.77, 2000);

            // //ring 12, 13
            // delay(700);
            // Intake.move(0);
            // chassis.turnToHeading(90, 500, {.minSpeed = 60});
            // chassis.moveToPoint(46.64, 97.33, 1000);
            // delay(300);
            // Intake.move(127);
            // delay(600);
            // Intake.move(0);
            // delay(500);

            // //stake 3
            // chassis.moveToPoint(73.20, 121, 3000, {.forwards = false, .maxSpeed = 70});
            // delay(2000);
            // mogoPiston.set_value(true);
            // Intake.move(127);
            
            // //ring 13
            // chassis.moveToPose(93.77, 93.77, 180, 2000);

            // //ring 14
            // chassis.moveToPoint(117.33, 93.77, 2000);

            // //move up on right wall into corner
            // chassis.moveToPose(140 - 2, 117.33, 0, 2000, {.maxSpeed = 70, .minSpeed = 60});
            // delay(500);
            // chassis.moveToPose(140 - 2, 135, 0, 2000);
            // delay(1000);
            // chassis.moveToPoint(110, 110, 2000, {.minSpeed = 80});
            // // wingPiston.set_value(true);

            // //back up to deposite stake
            // chassis.moveToPose(150, 140, -135, 1000, {.forwards = false, .minSpeed = 127});
            // delay(2000);
            // mogoPiston.set_value(false);

            // //back up
            // chassis.moveToPose(100, 100, -135, 1000, {.minSpeed = 70});

            // //push stake into corner
            // chassis.moveToPoint(93.77, 120, 1000, {.forwards = false, .minSpeed = 90});
            // chassis.moveToPoint(70.20, 120, 1000, {.forwards = false, .minSpeed = 90});
            // chassis.moveToPose(46.64, 129.11, 90, 5000, {.forwards = false, .minSpeed = 127});
            // chassis.moveToPose(0, 135, 90, 1000, {.forwards = false, .minSpeed = 127});

            // delay(10000);
}

void opcontrol() {

  int
  speed          = 0,
  turn           = 0,
  driveDirection = 1,
  leftDrive      = 0, 
  rightDrive     = 0,
  ringColour     = 0,
  armPosInt      = 0,
  toggleTimeOut  = 0;

  bool 
  doinkerToggle      = true,
  clawToggle         = true,
  mogoToggle         = false,
  armToggle          = false;

  while(true) {
    // Arcade control scheme
    float leftY  = Controller1.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);  // get left y and right x positions
    float rightX = Controller1.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

    // Turbo vroom vroom
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_UP) && toggleTimeOut > 9) {
      turboToggle = !turboToggle;
      toggleTimeOut = 0;
    }
    if(turboToggle || mogoToggle) {
      chassis.arcade(leftY, rightX, false, 0.25);
    } else chassis.arcade(0.7*leftY, 0.6*rightX, false, 0.25);

    // Intake motor control
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_R1)) {
      Intake.move(127);
    } else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R2)){
      Intake.move(-127);
    } else Intake.move(0);

    // Claw arm hold toggle
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_X) && toggleTimeOut > 9) {
      armToggle = !armToggle;
      toggleTimeOut = 0;
    }
    if(armToggle) {
      armMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
    } else armMotor.set_brake_mode(MOTOR_BRAKE_COAST);
    
    // Claw arm motor control
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_DOWN) && toggleTimeOut > 9) {
      armMotor.set_brake_mode(MOTOR_BRAKE_COAST);
      armMotor.move_absolute(50, 100);
      armMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
      toggleTimeOut = 0;
    }

    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_L1)) { // any movements will turn off arm PID
      armMotor.move(-127);
    } else if(Controller1.get_digital(E_CONTROLLER_DIGITAL_L2)) {
      armMotor.move(127);
    } else armMotor.brake();
    
    // Allows lb loader Task to run when bumper is activated
    if(lbBumper.get_value() && toggleTimeOut > 499) {
      enableLbLoading = true;
    } else enableLbLoading = false;

    // Toggle for claw piston
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_A) && toggleTimeOut > 9) {
      doinkerPiston.set_value(doinkerToggle);
      doinkerToggle = !doinkerToggle;
      toggleTimeOut = 0;
    }

    // Toggle for mogo piston
    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_Y) && toggleTimeOut > 9) {
      mogoPiston.set_value(mogoToggle);
      mogoToggle = !mogoToggle;
      toggleTimeOut = 0;
    } 

    if(Controller1.get_digital(E_CONTROLLER_DIGITAL_B)) {
      autonomous(); 

      // chassis.setPose(0, 0, 0);
      // // chassis.moveToPose(0, 30, 0, 10000);
      // chassis.turnToHeading(90, 10000);
    }

    toggleTimeOut++;

    delay(20);
  }                  
}