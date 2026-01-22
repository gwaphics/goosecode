#include "main.h"
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
#include <string>
using namespace std;
using namespace pros;
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// dt motors
pros::MotorGroup leftMotors({-18, -19, -14}, pros::MotorGearset::blue); // left motors
pros::MotorGroup rightMotors({11, 2,13}, pros::MotorGearset::blue); // right motors
// other motors
pros::Motor counterRoller(5, pros::MotorGearset::green);
pros::MotorGroup mainIntake({-1, -3}, pros::MotorGearset::green);
bool load = false;
bool unload = false;
// sensors
pros::Distance frontDS(16);
pros::Distance backDS(15);
pros::Distance rightDS(10);
pros::Distance leftDS(7);
pros::Optical colorSense(4);
// pnaumatics
pros::adi::DigitalOut tongue('C');
pros::adi::DigitalOut wing('B');
pros::adi::DigitalOut hood('A');

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
    if (colorSense.get_proximity() > 70 && hoodUp == false) { // add to color versions if work
        counterRoller.move(0);
        mainIntake.move(127);
    } else {
        counterRoller.move(127);
        mainIntake.move(127);
    }
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
    mainIntake.move(90);
    counterRoller.move(-90);
}

void lowScore() {
    mainIntake.move(-127);
    counterRoller.move(-127);
}

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
            string toPrint = to_string(frontDS.get()) + " " + to_string(rightDS.get()) + "          ";
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
        }
    });
}

// // auto functions for distance sensors
// void movePoint(int x, int y, int timeout) {
//     if (imu1.get_heading() > 180) {
//         while (chassis.getPose().y - frontDS.get_distance() > x && timeout > 0) {
//             leftMotors.move(50);
//             rightMotors.move(50);
//             timeout -= 50;
//             pros::delay(50);
//         }
//     }
// }

// position reset w/ distance sensors (using two walls closest to bot when facing any direction in each quadrant)
int quadrant = 1;
void reset() {
    if (quadrant == 1) {
        // bottom right quadrant
        if (chassis.getPose().theta > 315 && chassis.getPose().theta <= 45) {
            chassis.turnToHeading(0, 250);
            chassis.waitUntilDone();
            chassis.setPose(rightDS.get_distance()/25.4, backDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 45 && chassis.getPose().theta <= 135) {
            chassis.turnToHeading(90, 250);
            chassis.waitUntilDone();
            chassis.setPose(frontDS.get_distance()/25.4, rightDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 135 && chassis.getPose().theta <= 225) {
            chassis.turnToHeading(180, 250);
            chassis.waitUntilDone();
            chassis.setPose(leftDS.get_distance()/25.4, frontDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 225 && chassis.getPose().theta <= 315) {
            chassis.turnToHeading(270, 250);
            chassis.waitUntilDone();
            chassis.setPose(backDS.get_distance()/25.4, leftDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        }
    } else if (quadrant == 2) {
        // top right quadrant
        if (chassis.getPose().theta > 315 && chassis.getPose().theta <= 45) {
            chassis.turnToHeading(0, 250);
            chassis.waitUntilDone();
            chassis.setPose(rightDS.get_distance()/25.4, frontDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 45 && chassis.getPose().theta <= 135) {
            chassis.turnToHeading(90, 250);
            chassis.waitUntilDone();
            chassis.setPose(frontDS.get_distance()/25.4, leftDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 135 && chassis.getPose().theta <= 225) {
            chassis.turnToHeading(180, 250);
            chassis.waitUntilDone();
            chassis.setPose(leftDS.get_distance()/25.4, backDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 225 && chassis.getPose().theta <= 315) {
            chassis.turnToHeading(270, 250);
            chassis.waitUntilDone();
            chassis.setPose(backDS.get_distance()/25.4, rightDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        }
    } else if (quadrant == 3) {
        // top left quadrant
        if (chassis.getPose().theta > 315 && chassis.getPose().theta <= 45) {
            chassis.turnToHeading(0, 250);
            chassis.waitUntilDone();
            chassis.setPose(leftDS.get_distance()/25.4, frontDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 45 && chassis.getPose().theta <= 135) {
            chassis.turnToHeading(90, 250);
            chassis.waitUntilDone();
            chassis.setPose(backDS.get_distance()/25.4, leftDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 135 && chassis.getPose().theta <= 225) {
            chassis.turnToHeading(180, 250);
            chassis.waitUntilDone();
            chassis.setPose(rightDS.get_distance()/25.4, backDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 225 && chassis.getPose().theta <= 315) {
            chassis.turnToHeading(270, 250);
            chassis.waitUntilDone();
            chassis.setPose(frontDS.get_distance()/25.4, rightDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        }
    } else if (quadrant == 4) {
        // bottom left quadrant
        if (chassis.getPose().theta > 315 && chassis.getPose().theta <= 45) {
            chassis.turnToHeading(0, 250);
            chassis.waitUntilDone();
            chassis.setPose(leftDS.get_distance()/25.4, backDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 45 && chassis.getPose().theta <= 135) {
            chassis.turnToHeading(90, 250);
            chassis.waitUntilDone();
            chassis.setPose(backDS.get_distance()/25.4, rightDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 135 && chassis.getPose().theta <= 225) {
            chassis.turnToHeading(180, 250);
            chassis.waitUntilDone();
            chassis.setPose(rightDS.get_distance()/25.4, frontDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        } else if (chassis.getPose().theta > 225 && chassis.getPose().theta <= 315) {
            chassis.turnToHeading(270, 250);
            chassis.waitUntilDone();
            chassis.setPose(frontDS.get_distance()/25.4, leftDS.get_distance()/25.4, chassis.getPose().theta);
            pros::delay(50);
        }
    }
    
}


/* 
    0 - lateral PID test
    1 - angular PID test
    2 - 
    3 - 
    4 - DS Testing
    5 - skills
    6 - SAWP
    7 - aneek skills
    8 - elims w box bots
    9 - distance sensor test
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
            // lemlib movement with origin at corner of field instead of bot pos
            // chassis.setPose(frontDS.get_distance(), leftDS.get_distance(), 270);

            // chassis.moveToPoint(frontDS.get_distance()+30, leftDS.get_distance(), 10000);
            break;
        case 3:
            // chassis.setPose(frontDS.get_distance(), leftDS.get_distance(), 270);

            // movePoint(-30, 0, 10000);
            break;
        case 4:
            // testing
            chassis.setPose(frontDS.get_distance()/25.4,rightDS.get_distance()/25.4,90);
            quadrant = 1;

            // move to match loader
            chassis.moveToPoint(chassis.getPose().x+27, chassis.getPose().y, 2000);
            chassis.waitUntilDone();
            chassis.turnToHeading(180, 500);
            chassis.waitUntilDone();
            reset();

            // get balls out of match loader
            // tongue.set_value(true);
            chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-8, 2000);
            chassis.waitUntilDone();
            reset();

            // score on long goal
            chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y+30, 2000, {.forwards = false});


            break;
        case 5:
        // skills
            color = "none";
            chassis.setPose(0,0,0);
            wing.set_value(true);

            // get balls out bottom right tube
            chassis.moveToPoint(0,40,1500, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            tongue.set_value(true);
            chassis.waitUntilDone();
            intakeAll = true;
            chassis.moveToPoint(15, 40, 2500, {.maxSpeed = 60});
            chassis.waitUntilDone();

            // move to far right side of field and score on right long goal
            chassis.moveToPoint(0, 40, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750);
            tongue.set_value(false);
            chassis.waitUntilDone();
            chassis.moveToPoint(0, 52, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-70, 52, 3000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone(); 
            chassis.moveToPoint(-80, 38, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-57, 37, 1000, {.forwards = false});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.moveToPoint(-60, 37, 1500, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.setPose(0, 0, 270);
            chassis.waitUntilDone();

            // get far right match load tube and score on right long goal
            tongue.set_value(true);
            chassis.moveToPoint(-20, 1, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(-35, 0, 2500, {.maxSpeed = 60});
            chassis.waitUntilDone();
            chassis.moveToPoint(5, 1, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hoodUp = true;
            hood.set_value(true);
            tongue.set_value(false);
            chassis.moveToPoint(8, 0, 1500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.setPose(0, 0, 270);
            chassis.waitUntilDone();

            // get stacks of 4 balls on far side of field and score on low goal on way to far left match load tube
            chassis.turnToHeading(170, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(-1, -25, 2000, {.maxSpeed = 20});
            chassis.moveToPoint(-2, -50, 1000, {.maxSpeed = 80});
            chassis.moveToPoint(-2, -70, 2500, {.maxSpeed = 20});
            chassis.waitUntilDone();
            chassis.turnToHeading(45, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(7, -56, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(40, 750);
            chassis.waitUntilDone();
            intakeAll = false;
            lowGoal = true;
            pros::delay(2500);
            lowGoal = false;

            // move to far left match load tube
            chassis.moveToPoint(-30, -84, 2000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(270, 750);
            chassis.waitUntilDone();
            intakeAll = true;
            chassis.moveToPoint(-42, -83, 2500, {.maxSpeed = 60});
            chassis.waitUntilDone();
            pros::delay(1000);

            // go to near side of field and score on left long goal
            chassis.moveToPoint(-30, -84, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750);
            tongue.set_value(false);
            chassis.waitUntilDone();
            chassis.moveToPoint(-30, -96, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(40, -103, 3500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(50, -91, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(35, -88, 1000, {.forwards = false});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(90, 750);
            chassis.setPose(0, 0, 90);
            pros::delay(1500);

            // get near left match load tube and score on left long goal
            tongue.set_value(true);
            chassis.moveToPoint(20, 2, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(33, 3, 2500, {.maxSpeed = 60});
            chassis.waitUntilDone();
            chassis.moveToPoint(-5, 3, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hoodUp = true;
            hood.set_value(true);
            tongue.set_value(false);
            chassis.moveToPoint(-5, 1, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(90, 750);
            chassis.setPose(0, 0, 90);
            pros::delay(1000);

            // get 4 ball stack on near left side of field and score in mid goal
            chassis.turnToHeading(350, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(-2, 20, 2000, {.maxSpeed = 30});
            chassis.waitUntilDone();
            chassis.turnToHeading(135, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-11, 28, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(135, 750);
            intakeAll = false;
            chassis.waitUntilDone();
            lowGoal = true;
            pros::delay(100);
            lowGoal = false;
            midGoalAll = true;
            pros::delay(2500);

            // park and clear
            chassis.moveToPoint(30, 10, 1500, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(35, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(45, 45, 5000);
            pros::delay(420);
            tongue.set_value(true);

            break;
        case 6:
        // SAWP
            chassis.setPose(0,0,0);

            // grab teammate ball
            intakeAll = true;
            //chassis.moveToPoint(0, 10, 750);

            // get balls out of bottom right tube and score on right long goal
            chassis.moveToPoint(0, -37, 1200, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            tongue.set_value(true);
            chassis.waitUntilDone();
            chassis.moveToPoint(-15, -38, 1100, {.maxSpeed = 70});
            chassis.moveToPoint(17, -40, 700,{.forwards = false});
            chassis.waitUntilDone();
            tongue.set_value(false);
            hood.set_value(true);
            hoodUp = true;
            chassis.moveToPoint(20, -40, 800,{.forwards = false, .maxSpeed = 70});
            chassis.waitUntilDone();
            chassis.setPose(0,0, 270);
            
            // get middle balls
            chassis.moveToPoint(-7, 0, 500);
            chassis.turnToHeading(10, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(6, 18, 1500, {.maxSpeed = 60});
            chassis.waitUntilDone();
            
            chassis.turnToHeading(0, 300);
            chassis.waitUntilDone();
            
            chassis.moveToPoint(8, 55, 800);//change
            chassis.moveToPoint(8, 65, 550, {.maxSpeed = 40});
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(-45, 500);
            chassis.waitUntilDone();
            intakeAll = false;
            chassis.moveToPoint(27, 53, 750,{.forwards = false, .maxSpeed = 50});//change
            
            chassis.waitUntilDone();
            lowGoal = true;
            pros::delay(250);
            lowGoal = false;
            midGoalAll = true;
            pros::delay(700);
            midGoalAll = false;
            tongue.set_value(false);

            chassis.moveToPoint(-10, 91, 1150, {.maxSpeed = 90});
            intakeAll = true;
            chassis.waitUntilDone();
            tongue.set_value(true);
            chassis.turnToHeading(270, 700); // max speed 70
            
            chassis.moveToPoint(-30, 92, 1000, {.maxSpeed = 60});
            chassis.turnToHeading(250, 200);
            chassis.moveToPoint(10, 90, 800, {.forwards = false});
          
            chassis.waitUntilDone();
            tongue.set_value(false);
            hood.set_value(true);
            hoodUp = true;
            chassis.moveToPoint(10, 90, 1500, {.forwards = false});

            break;
        case 7:
            // aneek skills
            color = "none";
            chassis.setPose(0,0,0);
            wing.set_value(true);

            // get balls out bottom right tube
            chassis.moveToPoint(0,38,1500, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            tongue.set_value(true);
            chassis.waitUntilDone();
            intakeAll = true;
            chassis.moveToPoint(15, 38, 2500, {.maxSpeed = 60});
            chassis.waitUntilDone();

            // move to far right side of field and score on right long goal
            chassis.moveToPoint(0, 38, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750);
            tongue.set_value(false);
            chassis.waitUntilDone();
            chassis.moveToPoint(0, 49, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(-83, 49, 3000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone(); 
            chassis.turnToPoint(-83, 37.5, 750);
            chassis.moveToPoint(-83, 37.5, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.moveToPoint(-57, 37.5, 1000, {.forwards = false});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.moveToPoint(-60, 37, 1500, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750); 
            chassis.setPose(0, 0, 270);
            chassis.waitUntilDone();

            // get far right match load tube and score on right long goal
            tongue.set_value(true);
            pros::delay(100);
            chassis.moveToPoint(-20, 0, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(-35, 0, 2500, {.maxSpeed = 60});
            chassis.waitUntilDone();
            chassis.moveToPoint(5, 1, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hoodUp = true;
            hood.set_value(true);
            tongue.set_value(false);
            chassis.moveToPoint(8, 0, 1500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750); 
            chassis.setPose(0, 0, 270);
            intakeAll = true;

            // get stacks of 4 balls on far side of field and score on low goal on way to far left match load tube
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(-5, 0, 1000);
            chassis.waitUntilDone();
            chassis.turnToPoint(7, -25, 1000);
            chassis.waitUntilDone();
            chassis.moveToPoint(7, -25, 2000, {.maxSpeed = 80});
            chassis.turnToPoint(7, -60, 1000);
            chassis.moveToPoint(7, -60, 1000, {.maxSpeed = 80});
            chassis.turnToPoint(7, -65, 500);
            chassis.moveToPoint(7, -65, 1000, {.maxSpeed = 40});
            chassis.waitUntilDone();
            chassis.turnToPoint(15, -60, 1000);
            chassis.waitUntilDone();
            chassis.moveToPoint(15, -60, 2000);
            chassis.waitUntilDone();
            chassis.turnToHeading(40, 500);
            chassis.waitUntilDone();
            intakeAll = false;
            lowGoal = true;
            pros::delay(2500);

            // move to far left match load tube
            chassis.moveToPoint(-15, -90, 2000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            lowGoal = false;
            tongue.set_value(true);
            chassis.turnToHeading(270, 750);
            chassis.waitUntilDone();
            intakeAll = true;
            chassis.moveToPoint(-35, -89, 3500, {.maxSpeed = 60});
            chassis.waitUntilDone();

            // go to near side of field and score on left long goal
            chassis.moveToPoint(-15, -90, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 750);
            tongue.set_value(false);
            chassis.waitUntilDone();
            chassis.moveToPoint(-15, -103, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(270, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(50, -108, 3500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.moveToPoint(60, -95, 1000, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(40, -92, 1000, {.forwards = false});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(90, 750);
            chassis.setPose(0, 0, 90);
            pros::delay(1500);

            // get near left match load tube and score on left long goal
            tongue.set_value(true);
            chassis.moveToPoint(20, 0, 750);
            chassis.waitUntilDone();
            hood.set_value(false);
            hoodUp = false;
            chassis.moveToPoint(33, 0, 2500, {.maxSpeed = 60});
            chassis.waitUntilDone();
            chassis.moveToPoint(-5, 0, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hoodUp = true;
            hood.set_value(true);
            tongue.set_value(false);
            chassis.moveToPoint(-5, 0, 1000, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            hood.set_value(true);
            hoodUp = true;
            chassis.turnToHeading(90, 750);
            chassis.setPose(0, 0, 90);
            pros::delay(1000);

            // get 4 ball stack on near left side of field and score in mid goal
            // chassis.turnToHeading(350, 750);
            // chassis.waitUntilDone();
            
            // chassis.moveToPoint(-2, 20, 2000, {.maxSpeed = 30});
            // chassis.waitUntilDone();
            // hood.set_value(false);
            // hoodUp = false;
            // chassis.turnToHeading(135, 750);
            // chassis.waitUntilDone();
            // chassis.moveToPoint(-11, 28, 1000, {.forwards = false, .maxSpeed = 80});
            // chassis.waitUntilDone();
            // chassis.turnToHeading(135, 750);
            // intakeAll = false;
            // chassis.waitUntilDone();
            // lowGoal = true;
            // pros::delay(100);
            // lowGoal = false;
            // midGoalAll = true;
            // pros::delay(2500);

            // park and clear
            chassis.moveToPoint(25, 7, 1500, {.maxSpeed = 80});
            chassis.waitUntilDone();
            chassis.turnToHeading(35, 750);
            chassis.waitUntilDone();
            chassis.moveToPoint(45, 50, 5000);
            pros::delay(420);
            tongue.set_value(true);

            break;
        case 8:
            // elims w box bots
            color = "none";
            chassis.setPose(0,0,0);

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
            mainIntake.move(45);
            counterRoller.move(-20);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            mainIntake.move(-70);
            counterRoller.move(-50); 
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
