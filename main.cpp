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
pros::MotorGroup leftMotors({15, 16, 20}, pros::MotorGearset::blue); // left motors
pros::MotorGroup rightMotors({-13, -14,-18}, pros::MotorGearset::blue); // right motors
// other motors
pros::Motor counterRoller(6, pros::MotorGearset::green);
pros::MotorGroup mainIntake({5, -12}, pros::MotorGearset::green);
bool load = false;
bool unload = false;
// sensors
pros::Distance frontDS(4);
//pros::Distance backDS(10);
pros::Distance rightDS(9);
pros::Distance leftDS(21);
pros::Optical colorSense(19);
// pnaumatics
pros::adi::DigitalOut tongue('C');
pros::adi::DigitalOut frontWing('A');
pros::adi::DigitalOut backWing('H');
pros::adi::DigitalOut hood('F');
pros::adi::DigitalOut firstStage('D');
pros::adi::DigitalOut midGoalDescore('G');


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
pros::Imu imu1(19);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(7);
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
            pros::lcd::print(1, "Y Pos: %f", chassis.getPose().y); // x
            /*pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y*/
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            //string toPrint = "(" + to_string((int)chassis.getPose().x) + "," + to_string((int)chassis.getPose().y) + "," + to_string((int)chassis.getPose().theta) + ")";

            string toPrint = to_string(round(chassis.getPose().theta));
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
    2 - SAWP
    3 - Counter SAWP
    4 - Right 7 ball wing push
    5 - Left 7 ball wing push
    6 - Right 3+4 ball wing push
    7 - Left 3+4 ball wing push
    8 - States Skills
    9 - Elims w 40kE
*/
int chosenAuton = 4;

void autonomous() {
    switch(chosenAuton){
        case 0:
            // lateral pid test
            update = false;
            chassis.setPose(0,0,0);

            chassis.moveToPoint(0,24,10000);
            break;
        case 1:
            // angular pid test
            chassis.setPose(0,0,0);

            chassis.turnToHeading(90, 10000);
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

void opcontrol() {
    bool tongueOut = false;
    bool frontWingUp = false;
    bool backWingUp = false;
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
            frontWingUp = !frontWingUp;
            pros::delay(150);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) { 
            backWingUp = !backWingUp;
            pros::delay(150);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { 
            tongueOut = !tongueOut;
            pros::delay(150);
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

        // delay to save resources
        pros::delay(5);
    }
}
