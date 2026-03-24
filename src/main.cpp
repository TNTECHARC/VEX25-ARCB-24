/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Organization:       Autonomous Robotics Club (ARC)                      */
/*    Authors:            Coby Smith and Joseph Dye                           */
/*    Created:            9/9/2024                                            */
/*    Description:        ARC Template                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "screen.h"
#include "util.h"
#include "Drive.h"
#include "images.h"


using namespace vex;

////////////////////////// GLOBAL VARIABLES //////////////////////////

  // Competition Instance
  competition Competition;

  int odomType = TWO_AT_45;;

  bool isColorSorting = false;

  bool isInAuton = false;
  int lastPressed = 0;
  int teamColor = 0; //red = 0, blue = 1
  int driver = 0; //Elliot = 0, Jacob = 1
  bool odomDebugEnabled = true;

  // Define Values for the Chassis here:
  Drive chassis
  (
    motor_group(LFT, LFB, LBB, LBT), // Left drive train motors
    motor_group(RFT, RFB, RBB, RBT), // Right drive train motors
    PORT20,               // Inertial Sensor Port
    2.66,              // The diameter size of the wheel in inches 2.66
    1,                   // Wheel Ratio
    12,                   // The maximum amount of the voltage used in the drivebase (1 - 12)
    odomType,
    2.077,                  //Odometry wheel diameter (set to zero if no odom) (1.96 robot behind by .2) *1.143, 2 made it get off farther 1.955
    -1.270,               //Odom pod1 offset  -3.867, -1.28
    -1.270                //Odom pod2 offset  -3.867, -1.28
  );

//////////////////////////////////////////////////////////////////////

///////////////////////// Prototypes /////////////////////////////////

void setDriveTrainConstants();
void Auton_1();
void Auton_2();
void Auton_3();
void Auton_4();
void Auton_5();
void Auton_6();
void Auton_7();
void Auton_8();
void default_Auton();
void PAutonGenerator(float);

void toggleLift();
void toggleIntakeFlap();
void slowIntake();

void toggleColorSort();
void stopAllIntakeMotors();
void toggleDropDown();
int odomDebugThread();

//////////////////////////////////////////////////////////////////////


/// @brief Runs before the competition starts
void preAuton() 
{
  setDriveTrainConstants();

  chassis.brake(coast);       // make sure they aren’t holding weirdly
  chassis.driveMotors(0, 0);  

  enum preAutonStates{START_SCREEN = 0, SELECTION_SCREEN = 1};
  int currentScreen = START_SCREEN;
  //int lastPressed = 0;

  // Calibrates/Resets the Brains sensors before the competition
  inertial1.calibrate();
  rotation1.resetPosition();
  rotation2.resetPosition();

  vex::color colors[8] = {vex::color::red, vex::color::red, vex::color::red, vex::color::red, 
                          vex::color::blue, vex::color::blue, vex::color::blue, vex::color::blue};
  std::string names[8] = {"S Auto", "UCF", "LArm5", "MidDef5", 
                          "LongDef5", "LonTop5", "Auton 7", "2xLong5"};
  // std::string names[8] = {"Auton 1", "Auton 2", "Auton 3", "Auton 4", 
  //                         "Auton 5", "Auton 6", "Auton 7", "Auton 8"};
  Button buttons[9];
  createAutonButtons(colors, names, buttons);
  buttons[0].setChosen(true);

  Text selectionLabel;
  Text configLabel;
  Button startScreenButtons[5];
  createPreAutonScreen(startScreenButtons, selectionLabel, configLabel);
  
  int temp;

  Controller1.Screen.print(buttons[lastPressed].getName().c_str());

  while(!isInAuton){
    showPreAutonScreen(startScreenButtons, selectionLabel, configLabel, buttons[lastPressed].getName(), teamColor, driver);
    while(currentScreen == START_SCREEN){
      if(Brain.Screen.pressing()){
        if(checkPreAutonButtons(startScreenButtons, teamColor, driver, configLabel)){
          currentScreen = SELECTION_SCREEN;
        }
        Controller1.Screen.clearLine();
        Controller1.Screen.setCursor(1, 1);
        std::string colorString = teamColor ? "Blue" : "Red";
        std::string driverString = driver ? "Jacob" : "Elliot";
        std::string controllerPrint = buttons[lastPressed].getName() + " - " + colorString + " - " + driverString;
        Controller1.Screen.print(controllerPrint.c_str());
      }
      wait(10, msec);
    }

    showAutonSelectionScreen(buttons);
    while(currentScreen == SELECTION_SCREEN){
      if(Brain.Screen.pressing()){
        temp = checkButtonsPress(buttons);
        if(temp >= 0 && temp < 8){
          lastPressed = temp;
          Controller1.Screen.clearLine();
          Controller1.Screen.setCursor(1, 1);
          std::string colorString = teamColor ? "Blue" : "Red";
          std::string driverString = driver ? "Jacob" : "Elliot";
          std::string controllerPrint = buttons[lastPressed].getName() + " - " + colorString + " - " + driverString;
          Controller1.Screen.print(controllerPrint.c_str());
        }
      }
      if(temp == 8)
        currentScreen = START_SCREEN;
      wait(10, msec);
    }
    wait(10, msec);
  }
  Brain.Screen.clearScreen();
}

/// @brief Runs during the Autonomous Section of the Competition
void autonomous() 
{  
  drawSponsors();
  isInAuton = true;
  rotation1.resetPosition();
  rotation2.resetPosition();
  inertial1.resetHeading();
  
  wait(100, msec);

  setDriveTrainConstants();
  //PAutonGenerator(24);


  Auton_1(); // Skills Left
  //Auton_2(); // UCF Counter (Good Enough)
  //Auton_3(); // Match 8 in then descore arm??
  //Auton_4(); // Match Middle Defense Left
  //Auton_5(); // Match Long Goal Left Defense
  //Auton_6(); // Top load
  //Auton_7(); // (score bottom)
  //Auton_8(); // (double load long goal)

  // chassis.moveable();



  // //Allows for selection of different auton routes
  // switch (lastPressed) 
  // {
  //   case 0:
  //     Auton_1();
  //     break;
  //   case 1:
  //     Auton_2();
  //     break;
  //   case 2:
  //     Auton_3();
  //     break;
  //   case 3:
  //     Auton_4();
  //     break;
  //   case 4:
  //     Auton_5();
  //     break;
  //   case 5:
  //     Auton_6();
  //     break;
  //   case 6:
  //     Auton_7();
  //     break;
  //   case 7:
  //     Auton_8();
  //     break;
  //   default:
  //     default_Auton();
  //     break;
  // }
}

/// @brief Runs during the UserControl section of the competition
void usercontrol() 
{
  drawSponsors();
 
  

  // User control code here, inside the loop
  bool flapState = false;
  int lastSeen = teamColor;
  

  chassis.brake(coast);
  mainIntake.setStopping(coast);

  mainIntake.setVelocity(85, percent);
  colorSort.setVelocity(100, percent);
  topStage.setVelocity(100, percent);

  Controller1.ButtonL1.pressed(toggleLift);
  // Controller1.ButtonUp.pressed(toggleIntakeFlap);
  // Controller1.ButtonDown.pressed(slowIntake);
  Controller1.ButtonRight.pressed(toggleColorSort);
  Controller1.ButtonL2.pressed(toggleDropDown);
  Controller1.ButtonL2.released(toggleDropDown);

  //For Skills Auton

  bottomColorSort.setLight(ledState::on);
  bottomColorSort.integrationTime(20);
  while (1) {

      if(driver)
        chassis.tank();
      else
        chassis.arcade();

      if(bottomColorSort.color() == vex::color::red){
        lastSeen = 0;
      }else if(bottomColorSort.color() == vex::color::blue){
        lastSeen = 1;
      }

      if(Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()){
        mainIntake.spin(forward);
        topStage.spin(forward, 50, percent);
        flapState = false;
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else{
          colorSort.spin(reverse);
        }
      }else if(Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
        mainIntake.spin(reverse);
        topStage.spin(reverse);
        colorSort.spin(forward, 25, percent);
      }else if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
        mainIntake.spin(forward);
        topStage.spin(forward);
        flapState = true;
        colorSort.spin(fwd);
      }else if(Controller1.ButtonDown.pressing()){
        mainIntake.spin(reverse, 35, percent);
        topStage.spin(reverse);
        colorSort.spin(forward, 20, percent);
      }else if(Controller1.ButtonUp.pressing()){
        mainIntake.spin(forward);
        topStage.spin(forward, 35, percent);
        flapState = true;
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else{
          colorSort.spin(reverse);
        }
      }else{
        mainIntake.stop();
        colorSort.stop();
        topStage.stop();
      }
      if(!Controller1.ButtonR1.pressing() && !Controller1.ButtonDown.pressing() && !Controller1.ButtonUp.pressing()){
        flapState = false;
      }

      if(Controller1.ButtonA.pressing()){
        matchLoad.set(true);
        mainIntake.spin(forward);
        topStage.spin(forward, 50, percent);
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else
          colorSort.spin(reverse);
      }else{
        matchLoad.set(false);
        if(!Controller1.ButtonR1.pressing() && !Controller1.ButtonDown.pressing() && !Controller1.ButtonR2.pressing() && !Controller1.ButtonUp.pressing()){
          mainIntake.stop();
          colorSort.stop();
          topStage.stop();
        }
      }
      intakeFlap.set(flapState);
    wait(20, msec);
  }
    
}

void toggleLift(){
  static bool liftState = false;
  liftState = !liftState;
  intakeLift.set(liftState);
}

void toggleIntakeFlap(){
  static bool staticFlap = false;
  staticFlap = !staticFlap;
  intakeFlap.set(staticFlap);
}

void toggleDropDown(){

  static bool staticDrop = false;
  staticDrop = !staticDrop;
  dropDown.set(staticDrop);

}

void slowIntake(){
  static bool isSlowed = false;
  isSlowed = !isSlowed;
  if(isSlowed){
    mainIntake.setVelocity(50, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(50, percent);
  }else{
    mainIntake.setVelocity(85, percent);
    colorSort.setVelocity(100, percent);
    topStage.setVelocity(100, percent);
  }
}

void toggleColorSort(){
  isColorSorting = !isColorSorting;
}


void stopAllIntakeMotors() {
  mainIntake.stop();
  colorSort.stop();
  topStage.stop();
  bottomStage.stop();
  chassis.brake(coast);
  mainIntake.setStopping(coast);
  topStage.setStopping(coast);
  colorSort.setStopping(coast);
}

int main() 
{

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  preAuton();

  // Prevent main from exiting with an infinite loop.
  while (true) 
  {
    wait(100, msec);
  }
}


/// @brief Sets the PID values for the DriveTrain
void setDriveTrainConstants()
{

  
    // Set the Drive PID values for the DriveTrain
    // chassis.setDriveConstants(
    //     0.75,  // Kp - Proportion Constant
    //     0.000, // Ki - Integral Constant
    //     1.8, // Kd - Derivative Constant
    //     0.5, // Settle Error
    //     200, // Time to Settle
    //     2500 // End Time 5000
    // );  

    // Set the Turn PID values for the DriveTrain
    chassis.setTurnConstants(
        0.27,    // Kp - Proportion Constant 0.3
        0.0001,      // Ki - Integral Constant
        1.5,      // Kd - Derivative Constant 1.4
        1,    // Settle Error
        200,    // Time to Settle
        2500  //1000  // End Time
    );







  //Working
    // Set the Drive PID values for the DriveTrain
    chassis.setDriveConstants(
        0.85,  // Kp - Proportion Constant
        0.000, // Ki - Integral Constant
        2.5, // Kd - Derivative Constant
        0.5, // Settle Error
        200, // Time to Settle
        2500 // End Time 5000
    );  

    // // Set the Turn PID values for the DriveTrain
    // chassis.setTurnConstants(
    //     0.22,    // Kp - Proportion Constant
    //     0.000,      // Ki - Integral Constant
    //     1.5,      // Kd - Derivative Constant 1.4
    //     0.75,    // Settle Error
    //     200,    // Time to Settle
    //     2500  //1000  // End Time
    // );




}


//Default Auton Function
/// @brief Runs if no auton is selected
void default_Auton()
{
  Brain.Screen.print("No Auton Selected.");
  std::cout << "\n\n\n\n\nNO AUTON SELECTED------------------------------------\n";

}


//Auton Route Functions
/// @brief Auton Slot 1 - Write code for route within this function.
void Auton_1()
{
    //Brain.Screen.print("Skills Left Running.");
   chassis.driveDistanceWithOdom(6);
   wait(5, sec);
   chassis.driveDistanceWithOdom(12);

}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2()
{
  //Brain.Screen.print("UCF Running.");
      

}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3()
{
    //Brain.Screen.print("Match long with descore arm 5");
    

}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4()
{
    //Brain.Screen.print("Match Middle Defense Running.");


}

/// @brief Auton Slot 5 - Write code for route within this function.
void Auton_5()
{
    //Brain.Screen.print("Match Long Goal Left Defence Running.");
   

}

/// @brief Auton Slot 6 - Write code for route within this function.
void Auton_6()
{
    //Brain.Screen.print("Match Load long then top mid");
   
        
}

/// @brief Auton Slot 7 - Write code for route within this function.
void Auton_7()
{
    //Brain.Screen.print("Match Line Grab");
  

}

/// @brief Auton Slot 8 - Write code for route within this function.
void Auton_8()
{
    


}


int odomDebugThread() {
  while (odomDebugEnabled) {
    std::cout << "X POS: " << chassis.chassisOdometry.getXPosition()
              << " Y POS: " << chassis.chassisOdometry.getYPosition()
              << " HEADING: " << chassis.chassisOdometry.getHeading()
              << std::endl;

    vex::this_thread::sleep_for(100);
  }
  return 0;
}


/// @brief Automatically computes the best P value given a certain distance
/// @param distance distance to cover
void PAutonGenerator(float distance){
  //P value
  float pValue = 1.0;
  
  //PID constants
  float iValue = 0.0;
  float dValue = 0.0;
  float settleError = 0.50;
  float timeToSettle = 200;
  float endTime = 2500;

  //Storage
  std::vector<float> yErrors;
  std::vector<float> xErrors;
  float yAvgError;
  float xAvgError;
  int crossCountY = 0;
  int crossCountX = 0;
  float prevError = 0.0;
  bool hasPrevError = false;

  //Number of times for loops to iterate
  int numIterations = 10;

  //Ratios/Conditionals for P
  float incrementRatio = 1;
  float decrementRatio = 0.8;
  float acceptableError = 0.01 * distance;

  //SD Card variables
  std::string filename = "PTest.csv";
  std::ostringstream oss;

  float actualError = acceptableError+1; //Set higher than acceptableError so while loop will initally run

  chassis.setPosition(0, 0, 0);
  
  while(fabs(actualError) > acceptableError){
    chassis.setDriveConstants(pValue, iValue, dValue, settleError, timeToSettle, endTime);

    writeNewLineToCard(filename);
    oss << "Current P: " << pValue;
    writeToCard(filename, oss.str());
    oss.str("");
    oss.clear();
    writeNewLineToCard(filename);

    //Test on Y-axis
    for(int i=0; i<numIterations;i++){
      chassis.turnToAngle(0);
      chassis.setPosition(0, 0, 0);
      chassis.driveDistanceWithOdom(distance);
      writeToCard(filename, chassis.chassisOdometry.getYPosition()-distance);
      writeCommaToCard(filename);
      yErrors.push_back(chassis.chassisOdometry.getYPosition()-distance);
      if(hasPrevError && prevError * yErrors.at(yErrors.size()-1) < 0){
        crossCountY++;
      }
      prevError = yErrors.at(yErrors.size()-1);
      hasPrevError = true;

      chassis.driveDistanceWithOdom(-distance);
    }
    writeNewLineToCard(filename);

    //Average errors
    yAvgError = 0.0;
    for(int i=0;i<yErrors.size();i++){
      yAvgError += fabs(yErrors.at(i));
    }
    yAvgError /= yErrors.size();

    oss << "AVG ERR FOR P=" << pValue << " ON Y-AXIS: " << yAvgError;
    writeToCard(filename, oss.str());
    oss.str("");
    oss.clear();
    writeNewLineToCard(filename);

    //-----------------------------------------------------------//

    hasPrevError = false;
    prevError = 0.0;
    //Test on X-axis
    for(int i=0; i<numIterations;i++){
      chassis.turnToAngle(90);
      chassis.setPosition(0, 0, 90);
      chassis.driveDistanceWithOdom(distance);
      writeToCard(filename, chassis.chassisOdometry.getXPosition()-distance);
      writeCommaToCard(filename);
      xErrors.push_back(chassis.chassisOdometry.getXPosition()-distance);
      if(hasPrevError && prevError * xErrors.at(xErrors.size()-1) < 0){
        crossCountX++;
      }
      prevError = xErrors.at(xErrors.size()-1);
      hasPrevError = true;

      chassis.driveDistanceWithOdom(-distance);
    }
    writeNewLineToCard(filename);

    //Average errors
    xAvgError = 0.0;
    for(int i=0;i<xErrors.size();i++){
      xAvgError += fabs(xErrors.at(i));
    }
    xAvgError /= xErrors.size();

    oss << "AVG ERR FOR P=" << pValue << " ON X-AXIS: " << xAvgError;
    writeToCard(filename, oss.str());
    oss.str("");
    oss.clear();
    writeNewLineToCard(filename);

    //If avgError is more than the acceptable, decrease P
    if(yAvgError > acceptableError && xAvgError > acceptableError){
      if(crossCountY > numIterations/3 && crossCountX > numIterations/3){
        pValue -= incrementRatio;
        incrementRatio = incrementRatio / 2;
        pValue += incrementRatio;
      }else{
        pValue += incrementRatio;
      }
    }

    yErrors.clear();
    xErrors.clear();
    crossCountY = 0;
    crossCountX = 0;
    hasPrevError = false;
    prevError = 0.0;

    actualError = std::max(yAvgError, xAvgError);
  }
}