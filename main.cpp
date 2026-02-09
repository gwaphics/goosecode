#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <sys/types.h>
using namespace std;
using namespace pros;
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// dt motors
pros::MotorGroup leftMotors({-18, -19, -14}, pros::MotorGearset::blue); // left motors
pros::MotorGroup rightMotors({11, 2,13}, pros::MotorGearset::blue); // right motors
// other motors
pros::Motor counterRoller(5, pros::MotorGearset::green);
pros::MotorGroup mainIntake({-1, 3}, pros::MotorGearset::green);
bool load = false;
bool unload = false;
// sensors
pros::Distance frontDS(16);
pros::Distance backDS(10);
pros::Distance rightDS(15);
pros::Distance leftDS(7);
pros::Optical colorSense(4);
// pnaumatics
pros::adi::DigitalOut tongue('C');
pros::adi::DigitalOut wing('B');
pros::adi::DigitalOut hood('D');
pros::adi::DigitalOut firstStage('E');


bool stopSkills = false;
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_275,
                              450, // drivetrain rpm is 450
                              0 // horizontal drift is 2 (for now)
);
// create an imu on port 3
pros::Imu imu1(6);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(8);
// horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2,0);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu1 // inertial sensor
);

// lateral PID controller
 lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                               0, // integral gain (kI)
                                               45, // derivative gain (kD)
                                               3, // anti windup
                                               1, // small error range, in inches
                                               100, // small error range timeout, in milliseconds
                                               3, // large error range, in inches
                                               500, // large error range timeout, in milliseconds
                                               10 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              17, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

bool intakeRed = false;
bool intakeBlue = false;
bool intakeAll = false;
bool midGoalRed = false;
bool midGoalBlue = false;
bool midGoalAll = false;
bool lowGoal = false;
std::string color = "none";
bool hoodUp = false;

// void antiJam() {
//     hood.set_value(true);

//     while (true) {
//         if (std::abs(mainIntake.get_actual_velocity()) < 20) {
//             mainIntake.move(-127);
//             counterRoller.move(-127);
//         }
//         else {
//             mainIntake.move(127);
//             counterRoller.move(127);
//         }
//         pros::delay(50);
//     }
// }

void intake(std::string clr) {
    if (clr == "red") {
        if (colorSense.get_hue() > 140 && colorSense.get_hue() < 260 && colorSense.get_proximity() > 120) {
            mainIntake.move(127);
            counterRoller.move(-127);
            pros::delay(100);
        } else {
            mainIntake.move(127);
            counterRoller.move(127);
        }
    } else if (clr == "blue") {
        if (colorSense.get_hue() < 20 || colorSense.get_hue() > 340 && colorSense.get_proximity() > 120) {
            mainIntake.move(127);
            counterRoller.move(-127);
            pros::delay(100);
        } else {
            mainIntake.move(127);
            counterRoller.move(127);
        }
    }
}
void intake() {
    // if (colorSense.get_proximity() > 70 && hoodUp == false) { // add to color versions if work
    //     counterRoller.move(0);
    //     mainIntake.move(127);
    // } else {
    //     counterRoller.move(127);
    //     mainIntake.move(127);
    // }

    counterRoller.move(127);
    mainIntake.move(127);
}

void midScore(std::string clr) {
    if (clr == "red") {
        if (colorSense.get_hue() > 140 && colorSense.get_hue() < 260 && colorSense.get_proximity() > 120) {
            mainIntake.move(127);
            counterRoller.move(127);
            pros::delay(100);
        } else {
            mainIntake.move(127);
            counterRoller.move(-127);
        }
    } else if (clr == "blue") {
        if (colorSense.get_hue() < 20 || colorSense.get_hue() > 340 && colorSense.get_proximity() > 120) {
            mainIntake.move(127);
            counterRoller.move(127);
            pros::delay(100);
        } else {
            mainIntake.move(127);
            counterRoller.move(-127);
        }
    }
}
void midScore() {
    mainIntake.move(100);
    counterRoller.move(-127);
}

void lowScore() {
    mainIntake.move(-67);
    counterRoller.move(-127);
}

double safeDistanceInchesX(pros::Distance& sensor) {
    if (sensor.get_distance() <= 0 || sensor.get_distance() > 1200) {
        return chassis.getPose().x;
    }

    return sensor.get_distance() / 25.4;
}

double safeDistanceInchesY(pros::Distance& sensor) {
    if (sensor.get_distance() <= 0 || sensor.get_distance() > 1200) {
        return chassis.getPose().y;
    }

    return sensor.get_distance() / 25.4;
}

bool update = true;
int quadrant = 1;

time_t lastChange = time_t();
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    //intake();
    //antiJam();
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            //pros::lcd::print(0, "Dist: %i", colorSense.get_proximity());
            pros::lcd::print(0, "vel: %f", mainIntake.get_actual_velocity()); // x
            pros::lcd::print(1, "vel1: %f", counterRoller.get_actual_velocity()); // x
            /*pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y*/
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            //string toPrint = "(" + to_string((int)chassis.getPose().x) + "," + to_string((int)chassis.getPose().y) + "," + to_string((int)chassis.getPose().theta) + ")";

            string toPrint = to_string(chassis.getPose().x) + " " + to_string(chassis.getPose().y) + "          ";
            controller.set_text(0,0,toPrint);

            // delay to save resources
            pros::delay(50);
            
            if (intakeRed) {
                intake("red");
            } else if (intakeBlue) {
                intake("blue");
            } else if (intakeAll) {
                intake();
            } else if (midGoalRed) {
                midScore("red");
            } else if (midGoalBlue) {
                midScore("blue");
            } else if (midGoalAll) {
                midScore();
            } else if (lowGoal) {
                lowScore();
            } else {
                mainIntake.move(0);
                counterRoller.move(0);
            }

            if (update) {
                if (quadrant == 1) {
                    if (chassis.getPose().theta >= 358 || chassis.getPose().theta <= 2) {
                        chassis.setPose(144 - safeDistanceInchesX(rightDS) - 4.5, safeDistanceInchesY(backDS) + 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 88 && chassis.getPose().theta <= 92) {
                        chassis.setPose(144 - safeDistanceInchesX(frontDS) - 2, safeDistanceInchesY(rightDS) + 4.5, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 178 && chassis.getPose().theta <= 182) {
                        chassis.setPose(144 - safeDistanceInchesX(leftDS) - 4.5, safeDistanceInchesY(frontDS) + 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 268 && chassis.getPose().theta <= 272) {
                        chassis.setPose(144 - safeDistanceInchesX(backDS) - 2, safeDistanceInchesY(leftDS) + 4.5, chassis.getPose().theta);
                    }
                } else if (quadrant == 2) {
                    if (chassis.getPose().theta >= 358 || chassis.getPose().theta <= 2) {
                        chassis.setPose(144 - safeDistanceInchesX(rightDS) - 4.5, 144 - safeDistanceInchesY(frontDS) - 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 88 && chassis.getPose().theta <= 92) {
                        chassis.setPose(144 - safeDistanceInchesX(frontDS) - 2, 144 - safeDistanceInchesY(leftDS) - 4.5, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 178 && chassis.getPose().theta <= 182) {
                        chassis.setPose(144 - safeDistanceInchesX(leftDS) - 4.5, 144 - safeDistanceInchesY(backDS) - 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 268 && chassis.getPose().theta <= 272) {
                        chassis.setPose(144 - safeDistanceInchesX(backDS) - 2, 144 - safeDistanceInchesY(rightDS) - 4.5, chassis.getPose().theta);
                    }
                } else if (quadrant == 3) {
                    if (chassis.getPose().theta >= 358 || chassis.getPose().theta <= 2) {
                        chassis.setPose(safeDistanceInchesX(leftDS) + 4.5, 144 - safeDistanceInchesY(frontDS) - 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 88 && chassis.getPose().theta <= 92) {
                        chassis.setPose(safeDistanceInchesX(backDS) + 2, 144 - safeDistanceInchesY(leftDS) - 4.5, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 178 && chassis.getPose().theta <= 182) {
                        chassis.setPose(safeDistanceInchesX(rightDS) + 4.5, 144 - safeDistanceInchesY(backDS) - 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 268 && chassis.getPose().theta <= 272) {
                        chassis.setPose(safeDistanceInchesX(frontDS) + 2, 144 - safeDistanceInchesY(rightDS) - 4.5, chassis.getPose().theta);
                    }
                } else if (quadrant == 4) {
                    if (chassis.getPose().theta >= 358 || chassis.getPose().theta <= 2) {
                        chassis.setPose(safeDistanceInchesX(leftDS) + 4.5, safeDistanceInchesY(backDS) + 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 88 && chassis.getPose().theta <= 92) {
                        chassis.setPose(safeDistanceInchesX(backDS) + 2, safeDistanceInchesY(rightDS) + 4.5, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 178 && chassis.getPose().theta <= 182) {
                        chassis.setPose(safeDistanceInchesX(rightDS) + 4.5, safeDistanceInchesY(frontDS) + 2, chassis.getPose().theta);
                    } else if (chassis.getPose().theta >= 268 && chassis.getPose().theta <= 272) {
                        chassis.setPose(safeDistanceInchesX(frontDS) + 2, safeDistanceInchesY(leftDS) + 4.5, chassis.getPose().theta);
                    }
                }
            }
            
        }
    });
}


/* 
    0 - Lateral PID test
    1 - Angular PID test
    2 - SAWP
    3 - Skills
    4 - Elims w/ Ember
    5 - 9 Ball Elims
*/
int chosenAuton = 4;

void autonomous() {
    switch(chosenAuton){
        case 0:
            // lateral pid test
            chassis.setPose(0,0,0);

            chassis.moveToPoint(0,24,10000);
            break;
        case 1:
            // angular pid test
            chassis.setPose(0,0,0);

            chassis.turnToHeading(90, 10000);
            break;
        case 2:
            // SAWP
            quadrant = 1;
            chassis.setPose(144-frontDS.get_distance()/25.4 - 2,rightDS.get_distance()/25.4 + 4.5, 90);

            // descore matchload tube and score in long goal
            chassis.moveToPoint(144-26, chassis.getPose().y, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            update = false;
            chassis.turnToHeading(180, 500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            intakeAll = true;
            update = true;
            chassis.moveToPoint(144-26, 11, 800);
            chassis.waitUntilDone();
            // add move forward
            pros::delay(100);
            chassis.moveToPoint(144-24, 42, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(180, 500);
            leftMotors.move(-80);
            rightMotors.move(-80);
            pros::delay(300);
            leftMotors.move(0);
            rightMotors.move(0);
            tongue.set_value(false);

            // get center balls and score in mid goal
            chassis.moveToPoint(144-24, 30, 750);
            chassis.waitUntilDone();
            update = false;
            chassis.turnToHeading(-45, 500);
            hood.set_value(false);
            hoodUp = false;
            chassis.waitUntilDone();
            chassis.moveToPoint(144-47, 46, 1000);
            pros::delay(650);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-100, 47, 1000);
            pros::delay(300);
            tongue.set_value(false);
            pros::delay(600);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(225, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-85, 60, 750, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            intakeAll = false;
            lowGoal = true;
            pros::delay(200);
            lowGoal = false;
            midGoalAll = true;
            pros::delay(700);
            midGoalAll = false;
            intakeAll = true;

            // descore other matchload tube and score in long goal
            chassis.moveToPoint(25, 15, 1300, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            quadrant = 4;
            update = true;
            chassis.moveToPoint(25, 7, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            // add move forward
            pros::delay(100);
            chassis.moveToPoint(21, 44, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;

            break;
        case 3:
            // Skills
            quadrant = 1;
            chassis.setPose(144-frontDS.get_distance()/25.4 - 2,rightDS.get_distance()/25.4 + 4.5, 90);
            wing.set_value(true);

            // descore matchload tube and score in long goal
            chassis.moveToPoint(144-25, chassis.getPose().y, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            update = false;
            chassis.turnToHeading(180, 500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            intakeAll = true;
            update = true;
            chassis.moveToPoint(144-25, 11, 1000);
            chassis.waitUntilDone();
            leftMotors.move(80);
            rightMotors.move(80);
            pros::delay(1500);
            leftMotors.move(0);
            rightMotors.move(0);
            pros::delay(50);

            // move to second quadrant and score in long goal
            chassis.moveToPoint(144-25, 20, 750, {.forwards = false});
            chassis.waitUntilDone();
            update = false;
            tongue.set_value(false);
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            update = true;
            pros::delay(50);
            chassis.moveToPoint(144-12, 20, 750);
            chassis.waitUntilDone();
            update = false;
            chassis.turnToHeading(180, 500);
            chassis.waitUntilDone();
            update = true;
            pros::delay(50);
            update = false;
            chassis.moveToPoint(144-12, 144-32, 2500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            quadrant = 2;
            update = true;
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-22, 144-32, 750, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-22, 144-43, 750, {.forwards = false});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(0, 500);
            tongue.set_value(true);
            leftMotors.move(-80);
            rightMotors.move(-80);
            pros::delay(1500);
            leftMotors.move(0);
            rightMotors.move(0);
            pros::delay(50);

            // descore second matchload tube and score in long goal
            chassis.moveToPoint(144-23, 144-20, 750);
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(144-23, 144-11, 750, {.maxSpeed = 80});
            chassis.waitUntilDone();
            leftMotors.move(80);
            rightMotors.move(80);
            pros::delay(1500);
            leftMotors.move(0);
            rightMotors.move(0);
            chassis.moveToPoint(144-23, 144-43, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(0, 500);
            tongue.set_value(false);
            leftMotors.move(-80);
            rightMotors.move(-80);
            pros::delay(1500);
            leftMotors.move(0);
            rightMotors.move(0);

            // get mid balls and score in low goal
            update = false;
            pros::delay(50);
            chassis.setPose(0,0,chassis.getPose().theta);
            pros::delay(50);
            chassis.moveToPoint(0, 15, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.turnToHeading(225, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-25, -4, 1200, {.maxSpeed = 80});
            pros::delay(600);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-80, -4, 1500, {.maxSpeed = 80});
            pros::delay(300);
            tongue.set_value(false);
            pros::delay(600);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(135, 750);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(-68, -12, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(145, 750);
            chassis.waitUntilDone(); // delete for time save
            firstStage.set_value(true);
            intakeAll = false;
            lowGoal = true;
            pros::delay(1500);

            // descore matchload tube in third quadrant
            chassis.moveToPoint(-99, 20, 2000, {.forwards = false, .maxSpeed = 80});
            pros::delay(300);
            firstStage.set_value(false);
            tongue.set_value(true);
            lowGoal = false;
            intakeAll = true;
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-100, 36, 750, {.maxSpeed = 80});
            chassis.waitUntilDone();
            leftMotors.move(80);
            rightMotors.move(80);
            pros::delay(1500);
            // quadrant = 3;
            // pros::delay(50);
            // update = true;
            // pros::delay(50);
            leftMotors.move(0);
            rightMotors.move(0);

            // go to fourth quadrant and score in long goal
            // chassis.moveToPoint(26, 144-20, 750, {.forwards = false});
            chassis.moveToPoint(-100, 20, 750, {.forwards = false});
            chassis.waitUntilDone();
            // update = false;
            tongue.set_value(false);
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            // update = true;
            // pros::delay(50);
            // chassis.moveToPoint(12, 144-20, 750);
            chassis.moveToPoint(-114, 20, 750);
            chassis.waitUntilDone();
            // update = false;
            chassis.turnToHeading(5, 750);
            chassis.waitUntilDone();
            // update = true;
            // pros::delay(50);
            // update = false;
            // chassis.moveToPoint(12, 32, 2500, {.forwards = false, .maxSpeed = 80});
            chassis.moveToPoint(-123, -65, 2500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            // quadrant = 4;
            // update = true;
            chassis.turnToHeading(270, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(-109, -65, 750, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-109, -52, 750, {.forwards = false});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(true);
            leftMotors.move(-80);
            rightMotors.move(-80);
            pros::delay(1500);
            leftMotors.move(0);
            rightMotors.move(0);
            chassis.setPose(0,0,0);
            pros::delay(50);

            // get 4th quad tube and score in long goal
            chassis.moveToPoint(0, 25, 1000);
            chassis.moveToPoint(0, 35, 1000, {.maxSpeed = 60});
            hood.set_value(false);
            hoodUp = false;
            chassis.waitUntilDone();
            leftMotors.move(80);
            rightMotors.move(80);
            pros::delay(1500);
            leftMotors.move(0);
            rightMotors.move(0);
            pros::delay(50);
            chassis.moveToPoint(0, -5, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);
            leftMotors.move(-80);
            rightMotors.move(-80);
            pros::delay(1500);
            leftMotors.move(0);
            rightMotors.move(0);
            chassis.setPose(0,0,0);
            pros::delay(50);

            // park and clear
            chassis.moveToPoint(-10, 30, 1500, {.maxSpeed = 80});
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.turnToHeading(-35, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-50, 40, 2000);
            pros::delay(470);
            tongue.set_value(true);

            break;
        case 4:
            // Elims w/ Ember
            quadrant = 1;
            chassis.setPose(144-frontDS.get_distance()/25.4 - 2,rightDS.get_distance()/25.4 + 4.5, 90);

            // descore matchload tube and score in long goal
            chassis.moveToPoint(144-26, chassis.getPose().y, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            update = false;
            chassis.turnToHeading(180, 500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            intakeAll = true;
            update = true;
            chassis.moveToPoint(144-26, 11, 800);
            chassis.waitUntilDone();
            // add move forward
            pros::delay(100);
            chassis.moveToPoint(144-24, 42, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(180, 500);
            leftMotors.move(-80);
            rightMotors.move(-80);
            pros::delay(300);
            leftMotors.move(0);
            rightMotors.move(0);
            tongue.set_value(false);

            // get center balls and score in mid goal
            chassis.moveToPoint(144-24, 30, 750);
            chassis.waitUntilDone();
            update = false;
            chassis.turnToHeading(-45, 750);
            hood.set_value(false);
            hoodUp = false;
            chassis.waitUntilDone();
            chassis.moveToPoint(144-47, 46, 1000);
            pros::delay(650);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(-45, 750);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(144-61, 57, 750, {.maxSpeed = 80});
            chassis.waitUntilDone();
            // firstStage.set_value(true);
            intakeAll = false;
            lowGoal = true;


            break;
        
        case 5:
            // 9 ball
            // elims w box bots
            color = "none";
            chassis.setPose(0,0,0);
            update = false;

            chassis.turnToHeading(25, 250);
            chassis.waitUntilDone();
            intakeAll = true;
            chassis.moveToPoint(15, 25, 1000);
            pros::delay(500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.moveToPoint(27, 32, 750, {.maxSpeed = 80});
            pros::delay(100);
            tongue.set_value(false);
            pros::delay(400);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 250);
            chassis.waitUntilDone();
            chassis.moveToPoint(20, 30, 750, {.forwards = false});
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(20, 5, 1000, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(33, 5, 750);
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(33, 20, 750, {.forwards = false});
            chassis.waitUntilDone();
            chassis.setPose(0,0,0);
            hood.set_value(true);
            hoodUp = true;
            pros::delay(1000);
            tongue.set_value(true);
            chassis.moveToPoint(2, 15, 650);
            chassis.moveToPoint(2, 31, 1000, {.maxSpeed = 60});
            hood.set_value(false);
            hoodUp = false;
            chassis.waitUntilDone();
            chassis.moveToPoint(2, 15, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(130, 750);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(30, -11, 1000, {.maxSpeed = 90});
            chassis.waitUntilDone();
            intakeAll = false;
            lowGoal = true;
            pros::delay(1000);
            chassis.moveToPoint(11, -2, 1000, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(11, -14, 2000, {.forwards = false});
            chassis.waitUntilDone();
            lowGoal = false;

    }
    
}

// int avg(int arr[10]) {
// int c = 0;
// for (int i =0; i < 10; i++) {
//     c+= arr[i];
// }
// return c/10;
// }
// void extake() {
//     int prev[10];
//     int count = 0;
//     mainIntake.move(-127);
//     counterRoller.move(-127);
//     while (avg(prev) < 600) {
//         prev[count] = std::min(std::abs(mainIntake.get_actual_velocity()), std::abs(mainIntake.get_actual_velocity()));
//         if (count==9) {
//             count = 0;
//         }
//         else {
//             count++;
//         }
//         pros::delay(50);
//     }
//     mainIntake.move(0);
//     counterRoller.move(0);
// }

void opcontrol() {
    bool tongueOut = false;
    bool wingUp = false;
    bool doubleOut = false;
    bool gatesOut = false;

    // loop forever
    while (true) {
        
        // Exponential drive control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))/1.2;

		double cubedLeftY = (leftY * leftY * leftY);
		double cubedRightX = (rightX * rightX * rightX);

		double expY = (cubedLeftY/20000);
		double expX = (cubedRightX/20000);

		double expL = (leftY + rightX);
		double expR = (leftY - rightX);

		leftMotors.move(expL);
        rightMotors.move(expR);
        
		// intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
            hood.set_value(true);
            hoodUp = true;
            if (color == "red") {
                intakeRed = true;
            } else if (color == "blue") {
                intakeBlue = true;
            } else {
                intakeAll = true;
            }
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { 
            hood.set_value(false);
            hoodUp = false;
            if (color == "red") {
                intakeRed = true;
            } else if (color == "blue") {
                intakeBlue = true;
            } else {
                intakeAll = true;
            }
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (color == "red") {
                midGoalRed = true;
            } else if (color == "blue") {
                midGoalBlue = true;
            } else {
                midGoalAll = true;
            }
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            mainIntake.move(-127);
            counterRoller.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            mainIntake.move(50);
            counterRoller.move(-20);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            mainIntake.move(-50);
            counterRoller.move(-127); 
            firstStage.set_value(true);
        } else {
			intakeAll = false;
            midGoalAll = false;
            intakeRed = false;
            intakeBlue = false;
            midGoalRed = false;
            midGoalBlue = false;
            lowGoal = false;
            hood.set_value(false);
            hoodUp = false;
            firstStage.set_value(false);
		}

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            wingUp = !wingUp;
            pros::delay(150);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { 
            tongueOut = !tongueOut;
            pros::delay(150);
        }

        if (wingUp) {
            wing.set_value(true);
        } else {
            wing.set_value(false);
        }

        if (tongueOut) {
            tongue.set_value(true);
        } else {
            tongue.set_value(false);
        }

        // delay to save resources
        pros::delay(5);
    }
}
