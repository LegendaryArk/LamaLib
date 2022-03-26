#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"
#include "robotconfig.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  // pros::lcd::set_text(1, "Hello PROS User!");

  // pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  pros::vision_signature_s_t inputs[7] {
    pros::Vision::signature_from_utility(1, 1599, 3341, 2470, -4265, -3981,
                                         -4123, 2.900, 0),
    pros::Vision::signature_from_utility(2, -2231, -1643, -1937, 7951, 9405,
                                         8678, 3.000, 0),
    pros::Vision::signature_from_utility(3, 5611, 8165, 6888, -1395, -979,
                                         -1187, 3.000, 0),
    pros::Vision::signature_from_utility(4, 0, 0, 0, 0, 0, 0, 3.000, 0),
    pros::Vision::signature_from_utility(5, 0, 0, 0, 0, 0, 0, 3.000, 0),
    pros::Vision::signature_from_utility(6, 0, 0, 0, 0, 0, 0, 3.000, 0),
    pros::Vision::signature_from_utility(7, 0, 0, 0, 0, 0, 0, 3.000, 0)
  };
  lamalib::visionSensor visSensor(1, inputs);
  visSensor.setSignatures(inputs);
  double xScale =
      480.0 / 310; // Scaling the vision sensor range to the V5 Brain Screen
  double yScale = 240.0 / 212;
  int xl;
  int yl;
  int xr;
  int yr;
  while (true) {
    pros::vision_object_s_t rtn = visSensor.vSensor.get_by_sig(0, 3);
    xl = rtn.left_coord;
    yl = rtn.top_coord;
    xr = rtn.left_coord + rtn.width;
    yr = rtn.top_coord - rtn.height;
    pros::screen::set_eraser(COLOR_WHITE);
    pros::screen::erase_rect(0, 0, 480, 240);
    pros::screen::set_eraser(COLOR_RED);
    pros::screen::erase_rect(xl * xScale, yl * yScale, xr * xScale,
                             yr * yScale);
    // cout << visSensor.getMiddle(2)<< "\n";
    pros::delay(20);
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  int8_t ports[4] = {TOP_LEFT_CHASSIS, BOTTOM_LEFT_CHASSIS, TOP_RIGHT_CHASSIS,
                     BOTTOM_RIGHT_CHASSIS};
  bool reverseConfig[4] = {false, false, true, true};
  Chassis chassis(ports, reverseConfig, okapi::AbstractMotor::gearset::green);

  pros::IMU inertial(21);
  inertial.reset();
  while (inertial.is_calibrating())
    pros::delay(10);

  // OdomScales calibrated = odom.calibrate(chassis, master, inertial);
  // cout << calibrated.leftRadius << " " << calibrated.rightRadius << " " <<
  // calibrated.rearRadius << "\n";

  while (true) {
    int joyY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int joyX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.move(joyY + joyX, joyY - joyX);
    pros::delay(20);
  }
}
