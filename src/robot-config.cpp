#include "robot-config.h"

////////////// DO NOT REMOVE //////////////

// Defines the Brain and Controller
brain Brain;
controller Controller1;

///////////////////////////////////////////

//////////// Drive Train Motors ////////////

motor LFT = motor(PORT9, ratio6_1, false);
motor LFB = motor(PORT7, ratio6_1, true);
motor LBB = motor(PORT8, ratio6_1, false);
motor LBT = motor(PORT10, ratio6_1, true);

motor RFT = motor(PORT4, ratio6_1, false);
motor RFB = motor(PORT3, ratio6_1, true);
motor RBB = motor(PORT6, ratio6_1, true);
motor RBT = motor(PORT5, ratio6_1, false);
///////////////////////////////////////////

//////////// Odometry Sensors ////////////
rotation rotation1 = rotation(PORT12);
rotation rotation2 = rotation(PORT13);

/////////////////////////////////////////

//////////// Inertial Sensors ////////////

inertial inertial1 = inertial(PORT16);
//inertial inertial2 = inertial(PORT18);

/////////////////////////////////////////

motor intakeL = motor(PORT19, ratio6_1, true);
motor intakeR = motor(PORT17, ratio6_1, false);
motor colorSort = motor(PORT2, ratio18_1, true);
motor bottomStage = motor(PORT20, ratio6_1, false);
motor topStage = motor(PORT1, ratio6_1, false); //not always spinning

motor_group mainIntake = motor_group(intakeL, intakeR, bottomStage);
motor_group justIntake = motor_group(intakeL, intakeR);

//Pistons

/// @brief Pincers
digital_out matchLoad = digital_out(Brain.ThreeWirePort.F);

digital_out intakeFlap = digital_out(Brain.ThreeWirePort.D);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.B);


digital_out dropDown = digital_out(Brain.ThreeWirePort.H);


//Color Sort Opticals
optical bottomColorSort = optical(PORT11);
