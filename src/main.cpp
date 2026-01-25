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

  // Define Values for the Chassis here:
  Drive chassis
  (
    motor_group(LFT, LFB, LBB, LBT), // Left drive train motors
    motor_group(RFT, RFB, RBB, RBT), // Right drive train motors
    PORT20,               // Inertial Sensor Port
    2.66,              // The diameter size of the wheel in inches
    1,                   // 
    12,                   // The maximum amount of the voltage used in the drivebase (1 - 12)
    odomType,
    1.955,                  //Odometry wheel diameter (set to zero if no odom) (1.96 robot behind by .2)
    -1.250,               //Odom pod1 offset  -3.867, -1.22
    -1.250                //Odom pod2 offset  -3.867, -1.22
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

void toggleLift();
void toggleIntakeFlap();
void slowIntake();

void toggleColorSort();
void stopAllIntakeMotors();
void toggleDropDown();

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
  std::string names[8] = {"S Auto", "Auton 2", "Park5", "MidDef5", 
                          "LongDef5", "TopRam5", "Auton 7", "Auton 8"};
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


  //Auton_1(); // Skills Left
  //Auton_2(); // (double load long goal)
  //Auton_3(); // Match Park Left
  //Auton_4(); // Match Middle Defense Left
  //Auton_5(); // Match Long Goal Left Defense
  //Auton_6(); // Top Descore
  //Auton_7(); // (score bottom)
  //Auton_8(); // (score top)



  //Allows for selection of different auton routes
  switch (lastPressed) 
  {
    case 0:
      Auton_1();
      break;
    case 1:
      Auton_2();
      break;
    case 2:
      Auton_3();
      break;
    case 3:
      Auton_4();
      break;
    case 4:
      Auton_5();
      break;
    case 5:
      Auton_6();
      break;
    case 6:
      Auton_7();
      break;
    case 7:
      Auton_8();
      break;
    default:
      default_Auton();
      break;
  }
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
  Controller1.ButtonLeft.pressed(toggleDropDown);


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
        // if(flapState){
        //   topStage.spin(forward);
        // }else{
        //   topStage.stop();
        // }
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else{
          colorSort.spin(reverse);
        }
      }else if(Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
        mainIntake.spin(reverse);
        topStage.spin(reverse);
        colorSort.spin(forward, 25, percent);
      }else if(Controller1.ButtonL2.pressing()){
        matchLoad.set(true);
        mainIntake.spin(forward);
        if(lastSeen == teamColor || !isColorSorting){
          colorSort.spin(forward);
        }else{
          colorSort.spin(reverse);
        }
      }else if(Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
        mainIntake.spin(forward);
        topStage.spin(forward);
        flapState = true;
        // if(lastSeen == teamColor || !isColorSorting){
        //   colorSort.spin(forward);
        // }else{
        //   colorSort.spin(reverse);
        // }
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
        matchLoad.set(false);
        mainIntake.stop();
        colorSort.stop();
        topStage.stop();
      }
      if(!Controller1.ButtonR1.pressing() && !Controller1.ButtonDown.pressing() && !Controller1.ButtonUp.pressing()){
        flapState = false;
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
    chassis.setDriveConstants(
        0.7,  // Kp - Proportion Constant
        0.0001, // Ki - Integral Constant
        1.7, // Kd - Derivative Constant
        1.00, // Settle Error
        200, // Time to Settle
        2500 // End Time 5000
    );  

    // Set the Turn PID values for the DriveTrain
    chassis.setTurnConstants(
        0.25,    // Kp - Proportion Constant
        0.000,      // Ki - Integral Constant
        1.4,      // Kd - Derivative Constant 1.4
        2,    // Settle Error
        200,    // Time to Settle
        2500  //1000  // End Time
    );
    
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
    std::cout << "\n\n\n\n\nSTART------------------------------------\n";

    //Initial Settings
      chassis.setTurnMaxVoltage(8);
      chassis.setPosition(-46,15,0);
      mainIntake.setVelocity(100, percent);
      colorSort.setVelocity(100, percent);
      topStage.setVelocity(100, percent);
      bottomStage.setVelocity(100, percent);


    //Legend
      //Pencers              matchLoad.set(true);
      //Intakes              
        //Main/Pencers       mainIntake.spin(fwd);
        //Top                topStage.spin(fwd);
        //Color              colorSort.spin(fwd);
        //Bottom             bottomStage.spin(fwd);
        //Intake Only        justIntake.spin(fwd);
      //Outake               toggleLift();
      //Dropdown             toggleDropDown();
      //flap                 toggleIntakeFlap();


    


    //First T
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd, 50, percent);
      chassis.driveDistanceWithOdom(47); 
      wait(0.5, sec);
      matchLoad.set(true);
 
    //Goes for matchload
      chassis.driveDistanceWithOdom(-15); 
      chassis.turnToAngle(270);
      toggleLift();
      matchLoad.set(false);

      chassis.driveDistanceWithOdomTime(13.5, 1000); 
      topStage.stop();
      toggleLift();
      matchLoad.set(true);
      wait(2, sec);
      colorSort.stop();   
      chassis.driveDistanceWithOdom(-14); 
      chassis.turnToAngle(88); 
      toggleLift(); 
      matchLoad.set(false);
      chassis.driveDistanceWithOdomTime(11, 1000);
      toggleIntakeFlap();
      colorSort.spin(fwd);
      bottomStage.spin(fwd);
      topStage.spin(fwd, 25, percent);
      chassis.driveDistanceWithOdomTime(5.5, 1000); 
    //Load long with 8
      topStage.spin(fwd, 100, percent);
      wait(1, sec);
      bottomStage.spin(reverse, 50, percent);
      wait(0.2, sec);
      bottomStage.spin(fwd, 100, percent);
      wait(1.5, sec);
      bottomStage.stop();
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();

      chassis.driveDistanceWithOdom(-8.5);
      toggleIntakeFlap(); 
      chassis.turnToAngle(45);
      chassis.driveDistanceWithOdom(17); 
      chassis.turnToAngle(87); 
      chassis.driveDistanceWithOdom(33);
    //Grabs 2 from bellow
      chassis.turnToAngle(180);
      chassis.driveDistanceWithOdomTime(5, 1000);
      toggleDropDown();
      wait(0.3, sec);
      matchLoad.set(true);
      chassis.driveDistanceWithOdomTime(-5, 1000);
    //Grab 2 Blue from center under goal
      toggleDropDown(); 
      chassis.turnToAngle(90);
      toggleLift(); 
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      chassis.driveDistanceWithOdom(28);
      topStage.stop();
      matchLoad.set(false);


    //Getting 3 blue and 3 red from wall intake
      toggleLift(); 
      chassis.turnToAngle(130);
      chassis.driveDistanceWithOdom(19.5);
      chassis.turnToAngle(90);
      topStage.spin(forward);
      chassis.driveDistanceWithOdomTime(9.5, 1000); 
      matchLoad.set(true);
      wait(2, sec);
      colorSort.stop();
      topStage.stop();

    //Scoring 5 blue and 2 red
      chassis.driveDistanceWithOdomTime(-4, 1500);
      chassis.turnToAngle(270);
      matchLoad.set(false);
      chassis.driveDistanceWithOdomTime(22, 1000); 
      toggleIntakeFlap();
      colorSort.spin(fwd);
      topStage.spin(fwd, 20, percent);
      wait(0.1, sec); 
      chassis.driveDistanceWithOdomTime(3.5, 1000); 
      topStage.spin(forward, 100, percent);
      colorSort.spin(forward);
      bottomStage.spin(forward, 100, percent);
      mainIntake.spin(forward);
      wait(0.8, sec);
      toggleIntakeFlap();
      wait(0.5, sec);
      topStage.stop();



    //Goes for wall balls
      chassis.driveDistanceWithOdom(-15.5); 
      chassis.turnToAngle(0);
      chassis.driveDistanceWithOdomTime(15, 1000);
      matchLoad.set(true);
      chassis.driveDistanceWithOdomTime(-5, 1000);
      chassis.turnToAngleTime(5, 1000, 8);
      chassis.driveDistanceWithOdom(-56.5); 
      chassis.turnToAngle(90);
      matchLoad.set(false);


    //Out of park zone
      chassis.driveDistanceWithOdomTime(10, 1000); //9.5
      toggleDropDown();
      wait(1, sec);
      chassis.driveDistanceWithOdomTime(-7, 1000);
      chassis.turnToAngle(315);
      mainIntake.spin(forward);
      bottomStage.spin(forward);
      colorSort.spin(forward);
      topStage.spin(forward);
      chassis.driveDistanceWithOdom(27); 
      toggleDropDown(); 
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();



    //Into bottom goal
      chassis.turnToAngle(222); //224
      chassis.driveDistanceWithOdomTime(12, 1000, 6);
      chassis.driveDistanceWithOdomTime(-2, 1000); 

    
    //Load into bottom goal
      toggleLift(); 
      justIntake.spin(reverse, 10, percent); 
      bottomStage.spin(reverse, 40, percent); 
      colorSort.spin(fwd, 10, percent);
      topStage.spin(reverse, 60, percent);
      wait(4, sec);
      bottomStage.spin(reverse, 20, percent);
      wait(0.5, sec);
      justIntake.stop();
      bottomStage.stop();
      colorSort.stop();
      topStage.stop();

      chassis.driveDistanceWithOdom(-10);
      chassis.turnToAngle(285);
      mainIntake.spin(forward);
      colorSort.spin(forward);
      topStage.spin(forward);
      chassis.driveDistanceWithOdom(75); 
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //Park
      chassis.turnToAngle(195); 
      chassis.setDriveMaxVoltage(12);
      chassis.driveDistance(50);






}

/// @brief Auton Slot 2 - Write code for route within this function.
void Auton_2()
{
    
}

/// @brief Auton Slot 3 - Write code for route within this function.
void Auton_3()
{

    //Brain.Screen.print("Match Park Left");
    std::cout << "\n\n\n\n\nSTART------------------------------------\n";

        //Initial Settings
      chassis.setTurnMaxVoltage(8);
      chassis.setPosition(-46,8,0); //-46, 8
      mainIntake.setVelocity(100, percent);
      colorSort.setVelocity(100, percent);
      topStage.setVelocity(100, percent);
      bottomStage.setVelocity(100, percent);

      //Legend
      //Pencers              matchLoad.set(true);
      //Intakes              
        //Main/Pencers       mainIntake.spin(fwd);
        //Top                topStage.spin(fwd);
        //Color              colorSort.spin(fwd);
        //Bottom             bottomStage.spin(fwd);
        //Intake Only        justIntake.spin(fwd);
      //Outake               toggleLift();
      //Dropdown             toggleDropDown();
      //flap                 toggleIntakeFlap();
      //PurePursuit          movetopos(x,y,angle);



  //New Code

//moves to match loader


      chassis.turnToAngle(270);
      std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
      wait(.5, sec);
            chassis.turnToAngle(180);
            std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
            wait(.5, sec);
                  chassis.turnToAngle(90);
                  std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
                  wait(.5, sec);
                       chassis.turnToAngle(0);
                       std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
      //chassis.movetopos(10,10,90);
      wait(10, sec);

      chassis.movetopos(-46,46,0);//47
      std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
      
      //chassis.turnToPosition(-58, 47);
      chassis.turnToAngle(270);
      std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
      
      //toggleLift();//up
      chassis.movetopos(-58,46,270);//-58
      std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
      
      chassis.driveDistanceWithOdomTime(2, 1000); //bc it drives into match loader
      std::cout << chassis.chassisOdometry.getXPosition() << ", " << chassis.chassisOdometry.getYPosition() <<"," << chassis.chassisOdometry.getHeading() << "\n";
      
      //toggleLift();//down

    //Intakes all 6
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec); //3 to long
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //turns to discard the 3 blues
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(45);//320,0
      mainIntake.spin(reverse);
      bottomStage.spin(reverse);
      wait(0.5, sec); //1.5 to long, 1 to long
      mainIntake.stop();
      bottomStage.stop();
      matchLoad.set(false);
    
    //Moves back to match loader
      chassis.turnToAngle(270);//270, 265 was off
      chassis.driveDistanceWithOdomTime(7, 1000);

    //Intakes 5
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec);//3
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //Moves to long goal
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(87); //90 off right, 85 may have missed alligners
      matchLoad.set(false);
      justIntake.spin(fwd);
      toggleLift();//up
            //chassis.driveDistanceWithOdomTime(24, 1000);//bc it drives into long goal
      chassis.driveDistanceWithOdomTime(18, 1000); //20 and 4 works, 18 and 6 slightly too far

    //Loads all 8
      toggleIntakeFlap();//open
      topStage.spin(fwd, 100, percent);
        mainIntake.spin(fwd, 100, percent);
        colorSort.spin(fwd, 50, percent);

        chassis.driveDistanceWithOdomTime(6, 1000);
      wait(1, sec);
      bottomStage.spin(reverse, 50, percent);
      wait(0.2, sec);
      bottomStage.spin(fwd, 100, percent);
      wait(1.5, sec);//2
      bottomStage.stop();
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
      justIntake.stop();



      
    //Park Ending
      chassis.driveDistanceWithOdom(-6);
      toggleIntakeFlap();//close
      toggleLift();//down


      chassis.turnToAngle(247);
      chassis.driveDistanceWithOdom(18);
      chassis.setDriveMaxVoltage(8);//6 doesnt quite allign with wall
      chassis.movetopos(-60,23,175); //-62,28,180
      wait(.5, sec); // need to block
      chassis.driveDistanceWithOdomTime(-20,1000); //8, 12 can be more
      chassis.turnToAngle(187);//185
      chassis.setDriveMaxVoltage(12); 
      chassis.driveDistance(70); //60 works
      //chassis.driveMotors(12,12); 
      // wait(2.5, sec);
      // chassis.driveMotors(0,0);



  //Old code, works but not super consistent


// //moves to match loader
//       chassis.driveDistanceWithOdom(38);
//       chassis.turnToAngle(270);
//       //toggleLift();//up
//       chassis.driveDistanceWithOdomTime(14, 1000); //bc it drives into match loader
//       //toggleLift();//down

//     //Intakes all 6
//       mainIntake.spin(fwd);
//       colorSort.spin(fwd);
//       topStage.spin(fwd);
//       matchLoad.set(true);
//       wait(2, sec); //3 to long
//       mainIntake.stop();
//       colorSort.stop();
//       topStage.stop();


//     //turns to discard the 3 blues
//       chassis.driveDistanceWithOdom(-6);
//       chassis.turnToAngle(45);//320,0
//       mainIntake.spin(reverse);
//       bottomStage.spin(reverse);
//       wait(0.5, sec); //1.5 to long, 1 to long
//       mainIntake.stop();
//       bottomStage.stop();
//       matchLoad.set(false);
    
//     //Moves back to match loader
//       chassis.turnToAngle(270);//270, 265 was off
//       chassis.driveDistanceWithOdomTime(7, 1000);

//     //Intakes 5
//       mainIntake.spin(fwd);
//       colorSort.spin(fwd);
//       topStage.spin(fwd);
//       matchLoad.set(true);
//       wait(2, sec);//3
//       mainIntake.stop();
//       colorSort.stop();
//       topStage.stop();


//     //Moves to long goal
//       chassis.driveDistanceWithOdom(-6);
//       chassis.turnToAngle(87); //90 off right, 85 may have missed alligners
//       matchLoad.set(false);
//       justIntake.spin(fwd);
//       toggleLift();//up
//             //chassis.driveDistanceWithOdomTime(24, 1000);//bc it drives into long goal
//       chassis.driveDistanceWithOdomTime(18, 1000); //20 and 4 works, 18 and 6 slightly too far

//     //Loads all 8
//       toggleIntakeFlap();//open
//       topStage.spin(fwd, 100, percent);
//         mainIntake.spin(fwd, 100, percent);
//         colorSort.spin(fwd, 50, percent);

//         chassis.driveDistanceWithOdomTime(6, 1000);
//       wait(1, sec);
//       bottomStage.spin(reverse, 50, percent);
//       wait(0.2, sec);
//       bottomStage.spin(fwd, 100, percent);
//       wait(1.5, sec);//2
//       bottomStage.stop();
//       mainIntake.stop();
//       colorSort.stop();
//       topStage.stop();
//       justIntake.stop();



      
//     //Park Ending
//       chassis.driveDistanceWithOdom(-6);
//       toggleIntakeFlap();//close
//       toggleLift();//down


//       chassis.turnToAngle(247);
//       chassis.driveDistanceWithOdom(18);
//       chassis.setDriveMaxVoltage(8);//6 doesnt quite allign with wall
//       chassis.movetopos(-60,23,175); //-62,28,180
//       wait(.5, sec); // need to block
//       chassis.driveDistanceWithOdomTime(-20,1000); //8, 12 can be more
//       chassis.turnToAngle(187);//185
//       chassis.setDriveMaxVoltage(12); 
//       chassis.driveDistance(70); //60 works
//       //chassis.driveMotors(12,12); 
//       // wait(2.5, sec);
//       // chassis.driveMotors(0,0);
      

  



}

/// @brief Auton Slot 4 - Write code for route within this function.
void Auton_4()
{
    //Brain.Screen.print("Match Middle Defense Running.");


    //Brain.Screen.print("Match Park Left");
    std::cout << "\n\n\n\n\nSTART------------------------------------\n";

        //Initial Settings
      chassis.setTurnMaxVoltage(8);
      chassis.setPosition(-46,8,0);
      mainIntake.setVelocity(100, percent);
      colorSort.setVelocity(100, percent);
      topStage.setVelocity(100, percent);
      bottomStage.setVelocity(100, percent);

      //Legend
      //Pencers              matchLoad.set(true);
      //Intakes              
        //Main/Pencers       mainIntake.spin(fwd);
        //Top                topStage.spin(fwd);
        //Color              colorSort.spin(fwd);
        //Bottom             bottomStage.spin(fwd);
        //Intake Only        justIntake.spin(fwd);
      //Outake               toggleLift();
      //Dropdown             toggleDropDown();
      //flap                 toggleIntakeFlap();
      //PurePursuit          movetopos(x,y,angle);






//moves to match loader
      chassis.driveDistanceWithOdom(38);
      chassis.turnToAngle(270);
      //toggleLift();//up
      chassis.driveDistanceWithOdomTime(14, 1000); //bc it drives into match loader
      //toggleLift();//down

    //Intakes all 6
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec); //3 to long
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //turns to discard the 3 blues
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(45);//320
      mainIntake.spin(reverse);
      bottomStage.spin(reverse);
      wait(0.5, sec); //1.5 to long, 1 to long
      mainIntake.stop();
      bottomStage.stop();
      matchLoad.set(false);
    
    //Moves back to match loader
      chassis.turnToAngle(270);//270, 265 was off
      chassis.driveDistanceWithOdomTime(7, 1000);

    //Intakes 5
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec);//3
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //Moves to long goal
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(87); //90 off right, 85 may have missed alligners
      matchLoad.set(false);
      justIntake.spin(fwd);
      toggleLift();//up
      //chassis.driveDistanceWithOdomTime(24, 1000);//bc it drives into long goal
      chassis.driveDistanceWithOdomTime(19, 1000); //20 and 4 works, 18 and 6 slightly too far

    //Loads all 8
      toggleIntakeFlap();//open
      topStage.spin(fwd, 100, percent);
        mainIntake.spin(fwd, 100, percent);
        colorSort.spin(fwd, 50, percent);

        chassis.driveDistanceWithOdomTime(6, 1000);

      wait(1, sec);
      bottomStage.spin(reverse, 50, percent);
      wait(0.2, sec);
      bottomStage.spin(fwd, 100, percent);
      wait(1.5, sec);//2
      bottomStage.stop();
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
      justIntake.stop();



      
    
    //Block Middle Ending
        // chassis.driveDistanceWithOdom(-9);
        chassis.driveDistanceWithOdom(-30);
        toggleIntakeFlap();//close
        toggleLift();//down


        chassis.turnToAngle(112);
        chassis.driveDistanceWithOdom(72);

        // chassis.turnToAngle(145);
        // chassis.driveDistanceWithOdom(32);
        // chassis.turnToAngle(45);
        // chassis.driveDistanceWithOdom(20);//22
        // chassis.turnToAngle(134);
        // chassis.driveDistanceWithOdom(27);//24, 28 borderline on the line












}

/// @brief Auton Slot 5 - Write code for route within this function.
void Auton_5()
{
    //Brain.Screen.print("Match Long Goal Left Defence Running.");
    std::cout << "\n\n\n\n\nSTART------------------------------------\n";

        //Initial Settings
      chassis.setTurnMaxVoltage(8);
      chassis.setPosition(-46,8,0);
      mainIntake.setVelocity(100, percent);
      colorSort.setVelocity(100, percent);
      topStage.setVelocity(100, percent);
      bottomStage.setVelocity(100, percent);

      //Legend
      //Pencers              matchLoad.set(true);
      //Intakes              
        //Main/Pencers       mainIntake.spin(fwd);
        //Top                topStage.spin(fwd);
        //Color              colorSort.spin(fwd);
        //Bottom             bottomStage.spin(fwd);
        //Intake Only        justIntake.spin(fwd);
      //Outake               toggleLift();
      //Dropdown             toggleDropDown();
      //flap                 toggleIntakeFlap();
      //PurePursuit          movetopos(x,y,angle);






//moves to match loader
      chassis.driveDistanceWithOdom(38);
      chassis.turnToAngle(270);
      //toggleLift();//up
      chassis.driveDistanceWithOdomTime(14, 1000); //bc it drives into match loader
      //toggleLift();//down

    //Intakes all 6
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec); //3 to long
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //turns to discard the 3 blues
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(45);//320
      mainIntake.spin(reverse);
      bottomStage.spin(reverse);
      wait(0.5, sec); //1.5 to long, 1 to long
      mainIntake.stop();
      bottomStage.stop();
      matchLoad.set(false);
    
    //Moves back to match loader
      chassis.turnToAngle(270);//270, 265 was off
      chassis.driveDistanceWithOdomTime(7, 1000);

    //Intakes 5
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec);//3
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //Moves to long goal
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(87); //90 off right, 85 may have missed alligners
      matchLoad.set(false);
      justIntake.spin(fwd);
      toggleLift();//up
                  //chassis.driveDistanceWithOdomTime(24, 1000);//bc it drives into long goal
      chassis.driveDistanceWithOdomTime(19, 1000); //20 and 4 works, 18 and 6 slightly too far

    //Loads all 8
      toggleIntakeFlap();//open
      topStage.spin(fwd, 100, percent);
        mainIntake.spin(fwd, 100, percent);
        colorSort.spin(fwd, 50, percent);

        chassis.driveDistanceWithOdomTime(6, 1000);
      wait(1, sec);
      bottomStage.spin(reverse, 50, percent);
      wait(0.2, sec);
      bottomStage.spin(fwd, 100, percent);
      wait(1.5, sec);//2
      bottomStage.stop();
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
      justIntake.stop();


    //Block Long Goal Ending
        chassis.driveDistanceWithOdom(-4);
        wait(0.2, sec);
        toggleIntakeFlap();//close
        wait(0.5,sec);
        chassis.driveDistanceWithOdomTime(8, 1000);
        

 

}

/// @brief Auton Slot 6 - Write code for route within this function.
void Auton_6()
{
    //Brain.Screen.print("Match Top Middle Descore");
    std::cout << "\n\n\n\n\nSTART------------------------------------\n";

        //Initial Settings
      chassis.setTurnMaxVoltage(8);
      chassis.setPosition(-46,8,0);
      mainIntake.setVelocity(100, percent);
      colorSort.setVelocity(100, percent);
      topStage.setVelocity(100, percent);
      bottomStage.setVelocity(100, percent);

      //Legend
      //Pencers              matchLoad.set(true);
      //Intakes              
        //Main/Pencers       mainIntake.spin(fwd);
        //Top                topStage.spin(fwd);
        //Color              colorSort.spin(fwd);
        //Bottom             bottomStage.spin(fwd);
        //Intake Only        justIntake.spin(fwd);
      //Outake               toggleLift();
      //Dropdown             toggleDropDown();
      //flap                 toggleIntakeFlap();
      //PurePursuit          movetopos(x,y,angle);






//moves to match loader
      chassis.driveDistanceWithOdom(38);
      chassis.turnToAngle(270);
      //toggleLift();//up
      chassis.driveDistanceWithOdomTime(14, 1000); //bc it drives into match loader
      //toggleLift();//down

    //Intakes all 6
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec); //3 to long
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //turns to discard the 3 blues
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(45);//320
      mainIntake.spin(reverse);
      bottomStage.spin(reverse);
      wait(0.5, sec); //1.5 to long, 1 to long
      mainIntake.stop();
      bottomStage.stop();
      matchLoad.set(false);
    
    //Moves back to match loader
      chassis.turnToAngle(270);//270, 265 was off
      chassis.driveDistanceWithOdomTime(7, 1000);

    //Intakes 5
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec);//3
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //Moves to long goal
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(87); //90 off right, 85 may have missed alligners
      matchLoad.set(false);
      justIntake.spin(fwd);
      toggleLift();//up
      //chassis.driveDistanceWithOdomTime(24, 1000);//bc it drives into long goal
      chassis.driveDistanceWithOdomTime(19, 1000); //20 and 4 works, 18 and 6 slightly too far

    //Loads all 8
      toggleIntakeFlap();//open
      topStage.spin(fwd, 100, percent);
        mainIntake.spin(fwd, 100, percent);
        colorSort.spin(fwd, 50, percent);

        chassis.driveDistanceWithOdomTime(6, 1000);
      wait(1, sec);
      bottomStage.spin(reverse, 50, percent);
      wait(0.2, sec);
      bottomStage.spin(fwd, 100, percent);
      wait(1.5, sec);//1.5
      bottomStage.stop();
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
      justIntake.stop();


    //Descore Top Middle Ending
        chassis.driveDistanceWithOdom(-17);
        toggleIntakeFlap();//close
        toggleLift();//down

        chassis.turnToAngle(134);
        chassis.driveDistanceWithOdom(45);//50
        chassis.driveDistanceWithOdomTime(13,1000);
        chassis.driveDistanceWithOdom(-10);
        chassis.driveDistanceWithOdomTime(10,1000);
        chassis.driveDistanceWithOdom(-10);
        chassis.driveDistanceWithOdomTime(10,1000);
        chassis.driveDistanceWithOdom(-10);
        chassis.driveDistanceWithOdomTime(10,1000);
        chassis.driveDistanceWithOdom(-10);



        

}

/// @brief Auton Slot 7 - Write code for route within this function.
void Auton_7()
{
    //Brain.Screen.print("Match Line Grab");
    std::cout << "\n\n\n\n\nSTART------------------------------------\n";

        //Initial Settings
      chassis.setTurnMaxVoltage(8);
      chassis.setPosition(-46,8,0);
      mainIntake.setVelocity(100, percent);
      colorSort.setVelocity(100, percent);
      topStage.setVelocity(100, percent);
      bottomStage.setVelocity(100, percent);

      //Legend
      //Pencers              matchLoad.set(true);
      //Intakes              
        //Main/Pencers       mainIntake.spin(fwd);
        //Top                topStage.spin(fwd);
        //Color              colorSort.spin(fwd);
        //Bottom             bottomStage.spin(fwd);
        //Intake Only        justIntake.spin(fwd);
      //Outake               toggleLift();
      //Dropdown             toggleDropDown();
      //flap                 toggleIntakeFlap();
      //PurePursuit          movetopos(x,y,angle);






//moves to match loader
      chassis.driveDistanceWithOdom(38);
      chassis.turnToAngle(270);
      //toggleLift();//up
      chassis.driveDistanceWithOdomTime(14, 1000); //bc it drives into match loader
      //toggleLift();//down

    //Intakes all 6
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec); //3 to long
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //turns to discard the 3 blues
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(45);//320
      mainIntake.spin(reverse);
      bottomStage.spin(reverse);
      wait(0.5, sec); //1.5 to long, 1 to long
      mainIntake.stop();
      bottomStage.stop();
      matchLoad.set(false);
    
    //Moves back to match loader
      chassis.turnToAngle(270);//270, 265 was off
      chassis.driveDistanceWithOdomTime(7, 1000);

    //Intakes 5
      mainIntake.spin(fwd);
      colorSort.spin(fwd);
      topStage.spin(fwd);
      matchLoad.set(true);
      wait(2, sec);//3
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();


    //Moves to long goal
      chassis.driveDistanceWithOdom(-6);
      chassis.turnToAngle(87); //90 off right, 85 may have missed alligners
      matchLoad.set(false);
      justIntake.spin(fwd);
      toggleLift();//up
      //chassis.driveDistanceWithOdomTime(24, 1000);//bc it drives into long goal
      chassis.driveDistanceWithOdomTime(19, 1000); //20 and 4 works, 18 and 6 slightly too far

    //Loads all 8
      toggleIntakeFlap();//open
      topStage.spin(fwd, 100, percent);
        mainIntake.spin(fwd, 100, percent);
        colorSort.spin(fwd, 50, percent);

        chassis.driveDistanceWithOdomTime(6, 1000);
      wait(1, sec);
      bottomStage.spin(reverse, 50, percent);
      wait(0.2, sec);
      bottomStage.spin(fwd, 100, percent);
      wait(1.5, sec);//1.5
      bottomStage.stop();
      mainIntake.stop();
      colorSort.stop();
      topStage.stop();
      justIntake.stop();


    //Descore Top Middle Ending
        chassis.driveDistanceWithOdom(-17);
        toggleIntakeFlap();//close
        toggleLift();//down

        chassis.turnToAngle(135);
        //chassis.driveDistanceWithOdom(25);//50
        // chassis.movetopos(-22,22,135);
        chassis.driveDistanceWithOdom(20);



      //extra time stuff
      chassis.turnToAngle(90);
      chassis.setDriveMaxVoltage(8);
      //chassis.movetopos(0,22,0);
      //intake line

      isColorSorting = true;
      mainIntake.spin(forward);
      topStage.spin(forward);
      colorSort.spin(forward);

      chassis.setDriveMaxVoltage(6);
      chassis.movetopos(0,36,320);
      chassis.movetopos(-23,60,230);
      chassis.movetopos(-37,47,90);
      chassis.turnToAngle(90);
      chassis.driveDistanceWithOdomTime(5,1000);
      isColorSorting = false;

        


}

/// @brief Auton Slot 8 - Write code for route within this function.
void Auton_8()
{
    //Brain.Screen.print("Auton 8 Running");

}
