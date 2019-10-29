#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftDriveSmart = motor(PORT3, ratio18_1, false);
motor RightDriveSmart = motor(PORT9, ratio18_1, true);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 130, mm, 1);
motor LeadArmMotor = motor(PORT1, ratio18_1, false);
motor FollowArmMotor = motor(PORT2, ratio18_1, false);
motor ClawMotor = motor(PORT10, ratio18_1, false);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1XBButtonsControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_callback_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    // calculate the drivetrain motor velocities from the controller joystick axies
    // left = Axis3
    // right = Axis2
    int drivetrainLeftSideSpeed = Controller1.Axis3.position();
    int drivetrainRightSideSpeed = Controller1.Axis2.position();
    // check if the value is inside of the deadband range
    if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
      // check if the left motor has already been stopped
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        // stop the left drive motor
        LeftDriveSmart.stop();
        // tell the code that the left motor has been stopped
        DrivetrainLNeedsToBeStopped_Controller1 = false;
      }
    } else {
      // reset the toggle so that the deadband code knows to stop the left motor next time the input is in the deadband range
      DrivetrainLNeedsToBeStopped_Controller1 = true;
    }
    // check if the value is inside of the deadband range
    if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
      // check if the right motor has already been stopped
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        // stop the right drive motor
        RightDriveSmart.stop();
        // tell the code that the right motor has been stopped
        DrivetrainRNeedsToBeStopped_Controller1 = false;
      }
    } else {
      // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
      DrivetrainRNeedsToBeStopped_Controller1 = true;
    }
    // only tell the left drive motor to spin if the values are not in the deadband range
    if (DrivetrainLNeedsToBeStopped_Controller1) {
      LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
      LeftDriveSmart.spin(forward);
    }
    // only tell the right drive motor to spin if the values are not in the deadband range
    if (DrivetrainRNeedsToBeStopped_Controller1) {
      RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
      RightDriveSmart.spin(forward);
    }
    // check the ButtonL1/ButtonL2 status to control LeadArmMotor
    if (Controller1.ButtonL1.pressing()) {
      LeadArmMotor.spin(reverse);
      Controller1LeftShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonL2.pressing()) {
      LeadArmMotor.spin(forward);
      Controller1LeftShoulderControlMotorsStopped = false;
    } else if (!Controller1LeftShoulderControlMotorsStopped) {
      LeadArmMotor.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = true;
    }
    // check the ButtonR1/ButtonR2 status to control FollowArmMotor
    if (Controller1.ButtonL1.pressing()) {
      FollowArmMotor.spin(reverse);
      Controller1RightShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonL2.pressing()) {
      FollowArmMotor.spin(forward);
      Controller1RightShoulderControlMotorsStopped = false;
    } else if (!Controller1RightShoulderControlMotorsStopped) {
      FollowArmMotor.stop();
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1RightShoulderControlMotorsStopped = true;
    }
    // check the X/B buttons status to control ClawMotor
    if (Controller1.ButtonX.pressing()) {
      ClawMotor.spin(forward);
      Controller1XBButtonsControlMotorsStopped = false;
    } else if (Controller1.ButtonB.pressing()) {
      ClawMotor.spin(reverse);
      Controller1XBButtonsControlMotorsStopped = false;
    } else if (!Controller1XBButtonsControlMotorsStopped){
      ClawMotor.stop();
      Controller1XBButtonsControlMotorsStopped = true;
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_callback_Controller1);
}
