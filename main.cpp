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
pros::MotorGroup rightMotors({5, 11, 13}, pros::MotorGearset::blue); // left motors
pros::MotorGroup leftMotors({-7, -10,-12}, pros::MotorGearset::blue); // right motors
// other motors
pros::Motor counterRoller(18, pros::MotorGearset::green);
pros::MotorGroup mainIntake({15, -19}, pros::MotorGearset::green);
bool load = false;
bool unload = false;
// sensors
pros::Distance frontDS(8);
//pros::Distance backDS(10);
pros::Distance rightDS(16);
pros::Distance leftDS(21);
pros::Optical colorSense(19);
// pnaumatics
pros::adi::DigitalOut tongue('G');
pros::adi::DigitalOut frontWing('A');
pros::adi::DigitalOut backWing('E');
pros::adi::DigitalOut hood('F');
pros::adi::DigitalOut firstStage('D');
pros::adi::DigitalOut midGoalDescore('H');
pros::adi::DigitalOut B('B');
pros::adi::DigitalOut C('C');


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
pros::Imu imu1(17);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(4);
// horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2,0);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu1 // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(8, // proportional gain (kP) //8
                                              0, // integral gain (kI)
                                              50, // derivative gain (kD) //50
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP) //2
                                              0, // integral gain (kI)
                                              52, // derivative gain (kD) //50
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
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
bool autoMidGoal = false;
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
    mainIntake.move(127);
    counterRoller.move(-127);
}

void autoMidScore() {
    mainIntake.move(60);
    counterRoller.move(-115);
}

void lowScore() {
    mainIntake.move(-127);
    counterRoller.move(-127);
}

double safeDistanceInchesX(pros::Distance& sensor) {
    if (sensor.get_distance() <= 0 || sensor.get_distance() > 2000) {
        return chassis.getPose().x;
    }

    return sensor.get_distance() / 25.4;
}

double safeDistanceInchesY(pros::Distance& sensor) {
    if (sensor.get_distance() <= 0 || sensor.get_distance() > 2000) {
        return chassis.getPose().y;
    }

    return sensor.get_distance() / 25.4;
}

bool update = false;
int quadrant = 1;
int tolerance = 5;

double theta() {
    if (chassis.getPose().theta < -tolerance) {
        return chassis.getPose().theta + 360;
    }
    return chassis.getPose().theta;
}

time_t lastChange = time_t();
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0, 0, 0);
    //intake();
    //antiJam();
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            //pros::lcd::print(0, "Dist: %i", colorSense.get_proximity());
            pros::lcd::print(0, "X Pos: %f", chassis.getPose().x); // x
            // pros::lcd::print(0, "X Pos: %f", leftDS.get_distance()); // x
            pros::lcd::print(1, "Y Pos: %f", chassis.getPose().y); // y
            /*pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y*/
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            //string toPrint = "(" + to_string((int)chassis.getPose().x) + "," + to_string((int)chassis.getPose().y) + "," + to_string((int)chassis.getPose().theta) + ")";

            // string toPrint = to_string(round(chassis.getPose().x));
            // controller.set_text(0,0,toPrint);
            // controller.set_text(0,12,to_string(round(chassis.getPose().y)));

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
            } else if (autoMidGoal) {
                autoMidScore();
            } else if (lowGoal) {
                lowScore();
            } else {
                mainIntake.move(0);
                counterRoller.move(0);
            }

            if (update) {
                if (quadrant == 1) {
                    if (theta() >= 360-tolerance || theta() <= 0+tolerance) {
                        chassis.setPose(144 - safeDistanceInchesX(rightDS) - 4.5, chassis.getPose().y, chassis.getPose().theta);
                    } else if (theta() >= 90-tolerance && theta() <= 90+tolerance) {
                        chassis.setPose(144 - safeDistanceInchesX(frontDS) - 2, safeDistanceInchesY(rightDS) + 4.5, chassis.getPose().theta);
                    } else if (theta() >= 180-tolerance && theta() <= 180+tolerance) {
                        chassis.setPose(144 - safeDistanceInchesX(leftDS) - 4.5, safeDistanceInchesY(frontDS) + 2, chassis.getPose().theta);
                    } else if (theta() >= 270-tolerance && theta() <= 270+tolerance) {
                        chassis.setPose(chassis.getPose().x, safeDistanceInchesY(leftDS) + 4.5, chassis.getPose().theta);
                    }
                } else if (quadrant == 2) {
                    if (theta() >= 360-tolerance || theta() <= 0+tolerance) {
                        chassis.setPose(144 - safeDistanceInchesX(rightDS) - 4.5, 144 - safeDistanceInchesY(frontDS) - 2, chassis.getPose().theta);
                    } else if (theta() >= 90-tolerance && theta() <= 90+tolerance) {
                        chassis.setPose(144 - safeDistanceInchesX(frontDS) - 2, 144 - safeDistanceInchesY(leftDS) - 4.5, chassis.getPose().theta);
                    } else if (theta() >= 180-tolerance && theta() <= 180+tolerance) {
                        chassis.setPose(144 - safeDistanceInchesX(leftDS) - 4.5, chassis.getPose().y, chassis.getPose().theta);
                    } else if (theta() >= 270-tolerance && theta() <= 270+tolerance) {
                        chassis.setPose(chassis.getPose().x, 144 - safeDistanceInchesY(rightDS) - 4.5, chassis.getPose().theta);
                    }
                } else if (quadrant == 3) {
                    if (theta() >= 360-tolerance || theta() <= 0+tolerance) {
                        chassis.setPose(safeDistanceInchesX(leftDS) + 4.5, 144 - safeDistanceInchesY(frontDS) - 2, chassis.getPose().theta);
                    } else if (theta() >= 90-tolerance && theta() <= 90+tolerance) {
                        chassis.setPose(chassis.getPose().x, 144 - safeDistanceInchesY(leftDS) - 4.5, chassis.getPose().theta);
                    } else if (theta() >= 180-tolerance && theta() <= 180+tolerance) {
                        chassis.setPose(safeDistanceInchesX(rightDS) + 4.5, chassis.getPose().y, chassis.getPose().theta);
                    } else if (theta() >= 270-tolerance && theta() <= 270+tolerance) {
                        chassis.setPose(safeDistanceInchesX(frontDS) + 2, 144 - safeDistanceInchesY(rightDS) - 4.5, chassis.getPose().theta);
                    }
                } else if (quadrant == 4) {
                    if (theta() >= 360-tolerance || theta() <= 0+tolerance) {
                        chassis.setPose(safeDistanceInchesX(leftDS) + 4.5, chassis.getPose().y, chassis.getPose().theta);
                    } else if (theta() >= 90-tolerance && theta() <= 90+tolerance) {
                        chassis.setPose(chassis.getPose().x, safeDistanceInchesY(rightDS) + 4.5, chassis.getPose().theta);
                    } else if (theta() >= 180-tolerance && theta() <= 180+tolerance) {
                        chassis.setPose(safeDistanceInchesX(rightDS) + 4.5, safeDistanceInchesY(frontDS) + 2, chassis.getPose().theta);
                    } else if (theta() >= 270-tolerance && theta() <= 270+tolerance) {
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
    2 - Left 7 Ball Wing Push
    3 - Right 7 Ball Wing Push
    4 - Left 4+3 double descore
    5 - Right 4+3
    6 - All mid goal SAWP
    7 - Reg SAWP
*/
int chosenAuton = 1;

void autonomous() {
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    switch(chosenAuton){
        case 0:
            // lateral pid test
            update = false;
            chassis.setPose(0,0,0);

            chassis.moveToPoint(0,24,10000);
            break;
        case 1:
            // angular pid test
            update = false;
            chassis.setPose(0,0,0);

            chassis.turnToHeading(180, 10000);
            break;
        case 2:
            // Left 7 Ball Wing Push
            update = true;
            quadrant = 4;
            chassis.setPose(safeDistanceInchesX(leftDS) + 4.5, 24, 0);

            intakeAll = true;
            chassis.moveToPoint(46,50,1000);
            pros::delay(500);
            tongue.set_value(true);
            update = false;
            chassis.waitUntilDone();
            chassis.turnToHeading(-160,1000);
            chassis.waitUntilDone();
            chassis.moveToPoint(35, 30, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(-90, 750);
            update = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(23, 29, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(23, 12, 1200, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(23, 45, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);
            pros::delay(1200);
            chassis.moveToPoint(23, 32, 750);
            chassis.waitUntilDone();
            chassis.turnToHeading(-90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(34, 31, 750, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(34, 53, 750);
            chassis.waitUntilDone();
            break;
        case 3:
            // Right 7 Ball Wing Push
            update = false;
            quadrant = 1;
            chassis.setPose(144 - safeDistanceInchesX(rightDS) - 4.5, 24, 0);

            intakeAll = true;
            chassis.moveToPoint(144-48,52,1000);
            pros::delay(500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(125,1000);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-24, 30, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(-180, 750, {.maxSpeed = 100});
            update = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(144-23, 11, 1250, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-24, 45, 1000, {.forwards = false});
            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);
            update = false;
            pros::delay(1000);
            leftMotors.move(0);
            rightMotors.move(0);
            pros::delay(100);
            chassis.moveToPoint(144-24, 30, 750);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            update = true;
            chassis.moveToPoint(144-34, 31, 750, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 750, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-35, 53, 750, {.forwards = false, .minSpeed = 100});
            chassis.waitUntilDone();
            chassis.turnToHeading(160, 750, {.minSpeed = 127});
            chassis.waitUntilDone();
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            break;
        case 4:
            // Left 4+3 double descore
            update = true;
            quadrant = 4;
            chassis.setPose(54, 24, -90);

            intakeAll = true;
            chassis.moveToPoint(24, chassis.getPose().y, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.moveToPoint(23, 11, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(23, 45, 900, {.forwards = false});

            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);
            pros::delay(1000);
            hood.set_value(false);
            hoodUp = false;

            chassis.moveToPoint(23, 23, 750);
            chassis.waitUntilDone();
            update = false;
            chassis.turnToHeading(45, 750, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(50, 50, 1000, {.maxSpeed = 80});
            pros::delay(650);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(-135, 750, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(61, 66, 1000, {.forwards = false, .maxSpeed = 80});
            pros::delay(800);
            intakeAll = false;
            lowGoal = true;
            chassis.waitUntilDone();
            lowGoal = false;
            midGoalAll = true;
            pros::delay(1000);
            tongue.set_value(false);

            chassis.moveToPoint(35, 35, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(34, 50, 750, {.minSpeed = 127});
            chassis.waitUntilDone();


            break;
        case 5:
            // Right 4+3
            update = false;
            quadrant = 1;
            chassis.setPose(144-53, 24, 0); // 144 - safeDistanceInchesX(rightDS) - 4.5

            intakeAll = true;
            chassis.moveToPoint(144-48,53,1000);
            pros::delay(500);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(-45,1000, {.maxSpeed = 100});
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(144-61, 66, 1000);
            chassis.waitUntilDone();
            intakeAll = false;
            lowGoal = true;
            firstStage.set_value(true);
            pros::delay(1000);
            chassis.moveToPoint(144-25, 30, 1500, {.forwards = false, .maxSpeed = 80});
            lowGoal = false;
            intakeAll = true;
            firstStage.set_value(false);
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(-180, 750, {.maxSpeed = 100});
            update = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(144-23, 11, 1250, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-24, 45, 1000, {.forwards = false});
            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);
            update = false;
            pros::delay(1000);
            leftMotors.move(0);
            rightMotors.move(0);
            pros::delay(100);
            chassis.moveToPoint(144-24, 30, 750);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-36, 30, 750, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 750, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-36, 53, 750, {.forwards = false, .minSpeed = 100});
            chassis.waitUntilDone();
            leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            break;
        case 6:
            // All mid goal SAWP
            update = false;
            chassis.setPose(144-54, 24, 90);

            chassis.moveToPoint(144-24, 24, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 500);
            update = true;
            intakeAll = true;
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-24, 11, 900, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-24, 45, 900, {.forwards = false});
            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);
            update = false;
            pros::delay(1000);
            leftMotors.move(0);
            rightMotors.move(0);
            pros::delay(100);
            hood.set_value(false);
            hoodUp = false;

            chassis.moveToPoint(chassis.getPose().x, 23, 1000); // 144-24
            chassis.waitUntilDone();
            chassis.turnToHeading(-45, 750, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-48, 48, 1000, {.maxSpeed = 80});
            pros::delay(650);
            tongue.set_value(true);
            chassis.turnToHeading(-45, 500);
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(144-61, 61, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            intakeAll = false;
            lowGoal = true;
            firstStage.set_value(true);
            pros::delay(1000);
            chassis.moveToPoint(144-48, 48, 750, {.forwards = false, .maxSpeed = 80});
            lowGoal = false;
            intakeAll = true;
            firstStage.set_value(false);
            chassis.waitUntilDone();

            chassis.turnToHeading(-90, 750, {.maxSpeed = 100});
            update = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(48, 48, 1200, {.maxSpeed = 100});
            pros::delay(900);
            tongue.set_value(true);
            pros::delay(300);
            chassis.waitUntilDone();
            chassis.turnToHeading(-140, 500, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(59, 62, 500, {.forwards = false, .maxSpeed = 80});
            pros::delay(200);
            intakeAll = false;
            lowGoal = true;
            pros::delay(300);
            lowGoal = false;
            chassis.waitUntilDone();
            lowGoal = false;
            midGoalAll = true;

            break;
        case 7:
            // Reg SAWP
            update = false;
            chassis.setPose(144-54, 24, 90);

            chassis.moveToPoint(144-24, 24, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 500);
            update = true;
            intakeAll = true;
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.moveToPoint(144-24, 11, 900, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-24, 45, 900, {.forwards = false});
            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);
            update = false;
            pros::delay(1000);
            leftMotors.move(0);
            rightMotors.move(0);
            pros::delay(100);
            hood.set_value(false);
            hoodUp = false;

            chassis.moveToPoint(chassis.getPose().x, 23, 750); // 144-24
            chassis.waitUntilDone();
            chassis.turnToHeading(-45, 500, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(144-48, 48, 1000, {.maxSpeed = 80});
            pros::delay(650);
            tongue.set_value(true);
            chassis.waitUntilDone();

            chassis.turnToHeading(-90, 500);
            update = true;
            chassis.waitUntilDone();
            tongue.set_value(false);
            chassis.moveToPoint(48, 48, 1000, {.maxSpeed = 100});
            pros::delay(900);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToHeading(-140, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(59, 62, 500, {.forwards = false, .maxSpeed = 80});
            pros::delay(200);
            intakeAll = false;
            lowGoal = true;
            pros::delay(300);
            lowGoal = false;
            chassis.waitUntilDone();
            lowGoal = false;
            midGoalAll = true;
            pros::delay(700);
            midGoalAll = false;
            intakeAll = true;

            quadrant = 4;
            chassis.moveToPoint(24, 24, 1000, {.maxSpeed = 100});
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(24, 11, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();

            chassis.moveToPoint(24, 45, 900, {.forwards = false});
            chassis.waitUntilDone();
            leftMotors.move(-80);
            rightMotors.move(-80);
            hood.set_value(true);
            hoodUp = true;
            tongue.set_value(false);

            break;
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

// pros::adi::DigitalOut tongue('G');
// pros::adi::DigitalOut frontWing('A');
// pros::adi::DigitalOut backWing('H');
// pros::adi::DigitalOut hood('F');
// pros::adi::DigitalOut firstStage('D');
// pros::adi::DigitalOut midGoalDescore('H');

void opcontrol() {
    bool tongueOut = false;
    bool frontWingUp = false;
    bool backWingUp = false;
    bool hoodPistonUp = false;
    bool firstStageUp = false;
    bool midGoalDescoreUp = false;
    bool BUp = false;
    bool CUp = false;

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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
            frontWingUp = !frontWingUp;
            pros::delay(250);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) { 
            backWingUp = !backWingUp;
            pros::delay(250);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { 
            tongueOut = !tongueOut;
            pros::delay(250);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { 
            midGoalDescoreUp = !midGoalDescoreUp;
            pros::delay(250);
        }

        if (frontWingUp) {
            frontWing.set_value(true);
        } else {
            frontWing.set_value(false);
        }

        if (backWingUp) {
            backWing.set_value(true);
        } else {
            backWing.set_value(false);
        }

        if (tongueOut) {
            tongue.set_value(true);
        } else {
            tongue.set_value(false);
        }

        if (midGoalDescoreUp) {
            midGoalDescore.set_value(true);
        } else {
            midGoalDescore.set_value(false);
        }



        // piston testing
        // pros::adi::DigitalOut tongue('G');
        // pros::adi::DigitalOut frontWing('A');
        // pros::adi::DigitalOut backWing('E');
        // pros::adi::DigitalOut hood('F');
        // pros::adi::DigitalOut firstStage('D');
        // pros::adi::DigitalOut midGoalDescore('H');
        // pros::adi::DigitalOut B('B');
        // pros::adi::DigitalOut C('C');

        // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //     frontWingUp = !frontWingUp;
        //     frontWing.set_value(frontWingUp);
        //     controller.set_text(0,0,"A");
        //     pros::delay(150);
        // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { 
        //     backWingUp = !backWingUp;
        //     backWing.set_value(backWingUp);
        //     controller.set_text(0,0,"E");
        //     pros::delay(150);
        // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) { 
        //     tongueOut = !tongueOut;
        //     tongue.set_value(tongueOut);
        //     controller.set_text(0,0,"G");
        //     pros::delay(150);
        // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        //     hoodPistonUp = !hoodPistonUp;
        //     hood.set_value(hoodPistonUp);
        //     controller.set_text(0,0,"F");
        //     pros::delay(150);
        // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        //     firstStageUp = !firstStageUp;
        //     firstStage.set_value(firstStageUp);
        //     controller.set_text(0,0,"D");
        //     pros::delay(150);
        // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        //     midGoalDescoreUp = !midGoalDescoreUp;
        //     midGoalDescore.set_value(midGoalDescoreUp);
        //     controller.set_text(0,0,"H");
        //     pros::delay(150);
        // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        //     BUp = !BUp;
        //     B.set_value(BUp);
        //     controller.set_text(0,0,"B");
        //     pros::delay(150);
        // } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        //     CUp = !CUp;
        //     C.set_value(CUp);
        //     controller.set_text(0,0,"C");
        //     pros::delay(150);
        // }

        // delay to save resources
        pros::delay(5);
    }
}
