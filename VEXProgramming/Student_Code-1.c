/**
 * @file Student_Code.c
 * @author your name (you@domain.com)
 * @brief description of this file
 * @version 0.1
 * @date yyyy-mm-dd
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Libraries. DO NOT REMOVE */
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Student_Code.h"

// ---------------------- Defining physical robot parameters --------------------------
// Update these numbers to match the physical robot (information found in the lab manual)
int drivingWheelDiameter = 0;	    // diameter of the driving wheels [mm]
int robotWidth = 0;					// width of the robot including the wheel thickness [mm]
int wheelWidth = 0;					// width of the driving wheel [mm]
double drivingWheelRatio = 0.0;	    // ratio of wheel shaft rotations to wheel motor shaft rotations
double armRatio = 0.0;				// ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 0.0;	    // number of encoder ticks per 1 revolution of the motor shaft
// ------------------------------------------------------------------------------------

/* Write your code in the function below. You may add helper functions below the studentCode function. */

//The main function for the robot to complete the assigned task; picking up the payload, dropping it on the target, and navigating back to the start zone
void student_main()
{
    int sonar;

    armUp(5000);
    delay(1000);                     //Initialises arm for remainder of run
    resetEncoder(ArmEncoder);

    sonar = readSensor(SonarSensor);       //Used when reversing after picking up payload
    
    //In this section the payload gets picked up:
    drive_straight_ultrasonic(350.0);

    move_arm(-15.0);

    drive_straight_distance(100, 1.6, 0);
    delay(1000);
    
    armUp(5000);
    delay(1000);                         

    //In this section the robot uses the encoders to navigate to the target:
    drive_straight_distance(-(sonar-500), 1.6, 0);

    turn_angle(270, 4); 

    drive_straight_distance(400, 1.6, 0);         

    turn_angle(90, 4);

    drive_straight_distance(1225, 1.6, 0);

    turn_angle(270, 4);

    //In this section the robot drops the payload on the target, and navigates to the black line:
    drive_straight_ultrasonic(375.0);       

    move_arm(-15.0);                  //payload dropped on target

    drive_straight_distance(-150, 1.6, 0);      

    armUp(5000);
    delay(1000);

    turn_angle(180, 4);

    drive_until_black(60);

    //Finally, line following is performed and then encoder based navigation back to the start zone:
    line_following();

    drive_straight_distance(100, 1.6, 0);

    turn_angle(90, 4);

    drive_straight_distance(750, 1.6, 0);
}



// ----------------------------------------------- Function defintions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h


//Drives a specified distance in mm (forward or reverse). Drives in a straight line with PI controller accuracy
void drive_straight_distance(double distance, double Kp, double Ki)
{
    double currentPos, error, u = 0, errorSum = 0, KpStraight = 1.3;
    int L, R, encoError, deltaU, initialTime = 0 , finalTime = 0;

    //Zero encoders, initialise timer
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    resetTimer(T_1);

    do {
        currentPos = get_pos_wheel();         //Gets current position from wheel encoders
        error = distance - currentPos;          //Updates the error (E=R-Y)

        if (fabs(u) < 60) {         //Anti-windup control
            errorSum = errorSum + error;
        }
        
        u = (int) saturate((Kp*error)+(Ki*errorSum), -60, 60);         //Control effort

        lcd_print(LCDLine1,"power: %d",u);
        lcd_print(LCDLine2,"errorSum: %f",errorSum);         //Prints values for testing purposes
        lcd_print(LCDLine3,"error: %f",error);

        //Straightening control:
        L = readSensor(LeftEncoder);
        R = readSensor(RightEncoder);
        encoError = R-L;
        deltaU = (int) KpStraight*encoError;

        //Sending power to motors:
        motorPower(LeftMotor, convert_power(u+deltaU));
        motorPower(RightMotor, convert_power(u-deltaU));

        //Terminates function when in desired range for 3 seconds
        if ((fabs(distance - currentPos) >= 10)) {      //If robot is not in the desired range reset timer values 
            initialTime = 0;
            finalTime = 0;
        }
        else if ((fabs(distance - currentPos) <= 10) && (initialTime == 0)) {    //Record the time when the robot has enter the desired range 
            initialTime = readTimer(T_1);
        }
        else if ((fabs(distance - currentPos) <= 10) && (initialTime != 0)) {    //Record the present time of the robot in the desired range 
            finalTime = readTimer(T_1);
        }

        delay(100); //Repeat at ~10Hz
    } while((initialTime == 0 ) || (finalTime == 0) || ((finalTime - initialTime) < 3000));

    //Ensures motors have 0 power when function terminates
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

//Used for converting encoder counts to distance, used in "drive_straight_distance". Returns distance in mm
double get_pos_wheel()
{
    double average, distance;
    
    average = (readSensor(RightEncoder) + readSensor(LeftEncoder))/2.0;
    
    //Circumference of wheel = 323.58, 900 encoder counts = 1 revolution, 1 count = 0.36mm
    distance = average * 0.36;
    return distance;
}

//Drives in a straight line until the colour black is detected by the light sensors
void drive_until_black(int u)
{
    int encoError, deltaU, R, L;
    double KpStraight = 1.3;

    //Zero encoders
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);

    do {
        //Straightening control:
        L = readSensor(LeftEncoder);
        R = readSensor(RightEncoder);
        encoError = R - L;
        deltaU = (int) KpStraight*encoError;

        //Sending power to motors:
        motorPower(LeftMotor, convert_power(u+deltaU));
        motorPower(RightMotor, convert_power(u-deltaU));

        delay(100); //Repeats at ~10Hz

    //"When the colour black is seen by the sensors", terminate
    } while(((readSensor(MidLight) <= 2350) && (readSensor(LeftLight) <= 2350) && (readSensor(RightLight) <= 2350)));
    
    //Ensures 0 motor power at termination
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}

//Converts a power inputted in percentage form to mV form. Returns voltage (to be used by motors) in mV:
int convert_power(double powerPercent)
{
    //The range of power percentage is from -100 (reverses motor) to 100 
    double lowerLimits = -100; 
    double upperLimits = 100;
    double powerLevel;

    powerLevel = saturate(powerPercent, lowerLimits, upperLimits); //Ensures powerPercent can not be out of the range
    int voltage = (powerLevel*50); //From -5000mV to 5000mV
    return voltage;
}  

//Function can be ignored; used purely for testing sensor values
void test()
{
//Test sensor reading for brown
    int left, mid, right;
    mid = readSensor(MidLight);
    left = readSensor(LeftLight);
    right = readSensor(RightLight);
    lcd_print(LCDLine1,"left: %d mid: %d right: %d ", left, mid, right);
}

//Moves the arm to a specified angle in degrees (relative to horizontal plane):
void move_arm(double armAngle)
{
    //Arm starts at highest position with encoder = 0. Not reset throughout run to store arm's position
    double currentPos, u, error, errorSum = 0, Kp = 4.5;
    int armVoltage;

    do {
        currentPos = get_pos_arm();
        error = currentPos - armAngle;

        u = (-1*Kp*error);        //Control effort

        armVoltage = convert_power(saturate(u, -70, 70));
        motorPower(ArmMotor, armVoltage);

        delay(50);     //Repeats at 20Hz

    } while (fabs(armAngle - currentPos) >= 2);    //Stops when within 2 degrees of target angle

    motorPower(ArmMotor, 0);    //Ensures no movement after this condition is reached
}

//Finds the angle (in degrees, relative to horizontal plane) that the arm is currently at, and returns this
double get_pos_arm()
{
    double initialPos = 47.0, currentPos, angle;

    currentPos = (readSensor(ArmEncoder) / 17.5);        //17.5 encoder counts = 1 degree

    angle = initialPos + currentPos;
    return angle;     
}

//Drives the robot until a specified distance from a wall/object (distance in mm)
void drive_straight_ultrasonic(double distance)
{ 
    double sonar, driveDistance;

    sonar = readSensor(SonarSensor);

    driveDistance = sonar - distance;     //Drives until the specified "distance" away

    drive_straight_distance(driveDistance, 2, 0);
}

//Turns the robot a specified angle in degrees; positive angle = CCW, negative angle = CW
void turn_angle(double angle, double backlash)
{
    double currentPos, error, Kp = 2.5, Ki = 0, KpMatchwheels = 1, errorSum = 0;
    int L, R, encoError, deltaU, u = 0, u = 0;

    //Zero encoders
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);

    do{
        currentPos = get_angle();      //Get current position from wheel encoders, in degrees
        
        error = angle - currentPos;       //Update the error (E=R-Y)
        errorSum = errorSum + error;

        u = (int) fabs(saturate((Kp*error) + (Ki*errorSum), -50, 50));        //Control effort

        //Straightening control, so each wheel should turn same amounts
        L = fabs(readSensor(LeftEncoder));
        R = fabs(readSensor(RightEncoder));
        encoError = R - L;
        deltaU = (int) KpMatchwheels*encoError;

        //Sending power to motors:
        //Positive error = turning CCW:
        if (error > 0) {
            motorPower(LeftMotor, convert_power((-1*u) - deltaU)); 
            motorPower(RightMotor, convert_power(u - deltaU));         //These values enable anti-clockwise motor turning

            lcd_print(LCDLine2, "left motor power = %d", convert_power((-1*u) - deltaU));
            lcd_print(LCDLine3, "right motor power = %d", convert_power(u - deltaU));
        }
        //Negative error = turning CW:
        else {
            motorPower(LeftMotor, convert_power(u + deltaU)); 
            motorPower(RightMotor, convert_power((-1*u) + deltaU));       //These values enable clockwise motor turning

            lcd_print(LCDLine2, "left motor power = %d", convert_power(u + deltaU));
            lcd_print(LCDLine3, "right motor power = %d", convert_power((-1*u) + deltaU));
        }
        lcd_print(LCDLine1, "error = %f", error);

        delay(100); //Repeats at ~10Hz

    } while ((fabs(angle - currentPos)) >= 5);       //Stops when within 5 degrees of the target angle

    //Ensures 0 motor power when this condition is reached
    motorPower(LeftMotor, convert_power(0)); 
    motorPower(RightMotor, convert_power(0));

}

//Returns the current angle of the robot when turning (always positive). Used to assist the turning function
double get_angle()
{
    double distance, angle;

    distance = ((fabs(readSensor(RightEncoder))+fabs(readSensor(LeftEncoder)))/2.0) * 0.36;     //Finds approximate absolute distance moved by each wheel

    //114 determined from the circumference of the robot's circular turning path:
    angle = (distance/114)*(180/3.14159265);      //Finds absolute value of angle turned in degrees , relative to centre of robot

    return angle;
}

//Determines the "case" for line following, i.e. what combination of light sensors are seeing brown/not seeing brown:

//CASES: 0 = Doesn't see brown, 1 = sees brown

//Case 1: 111   (drive straight)
//Case 2: 110   (turn left)
//Case 3: 101   (drive straight)
//Case 4: 100   (turn left)
//Case 5: 011   (turn right)
//Case 6: 010   (drive straight)
//Case 7: 001   (turn right)
//Case 8: 000   (reverse)
//Case 9: a black line is seen (cease operation)

int line_following_case()
{
    int Case;

    int leftSensor = readSensor(LeftLight);
    int midSensor = readSensor(MidLight);
    int rightSensor = readSensor(RightLight);


    //CASES: 0 = Doesn't see brown, 1 = sees brown
    //Values have been assigned based on experimental testing of sensor values
    if ((leftSensor > 1000) && (leftSensor < 2200)) {
        if ((midSensor > 700) && (midSensor < 2200)) {
            if ((rightSensor > 1000) && (rightSensor < 2200)) {
                Case = 1;      //Case 1: 111   (drive straight)      
            }
            else {
                Case = 2;      //Case 2: 110   (turn left)
            }  
        }
        else {
            if ((rightSensor > 1000) && (rightSensor < 2200)) {
                Case = 3;      //Case 3: 101   (drive straight)
            }
            else {
                Case = 4;      //Case 4: 100   (turn left)
            }  
        }
    }
    else {
        if ((midSensor > 700) && (midSensor < 2200)) {
            if ((rightSensor > 1000) && (rightSensor < 2200)) {
                Case = 5;      //Case 5: 011   (turn right)      
            }
            else {
                Case = 6;      //Case 6: 010   (drive straight)
            }  
        }
        else {
            if ((rightSensor > 1000) && (rightSensor < 2200)) {
                Case = 7;      //Case 7: 001   (turn right)
            }
            else {
                Case = 8;      //Case 8: 000   (reverse, try to catch line again) 
            }  
        }
    }
    if ((leftSensor > 2350) || (midSensor > 2350) || (rightSensor > 2350)) {
        Case = 9;      //Case 9: a black line is seen (cease operation)
    }
    return Case;
}

//The primary line following function, enabling the robot to follow a brown line until a black line is detected
void line_following()
{
    int Case = line_following_case();
    int defaultPower = 30;

    //While sensors do not see the colour black:
    while (Case != 9) {;
        if ((Case == 8)) {        //Reverses when all off line, tries to catch line again
            motorPower(LeftMotor, -defaultPower);
            motorPower(RightMotor, -defaultPower);
        }
        else if ((Case == 2)) {                       //Turning left
            motorPower(LeftMotor, convert_power(0)); 
            motorPower(RightMotor, convert_power(defaultPower));
        }
        else if (Case == 4){                          //Turning left more aggressively
            motorPower(LeftMotor, convert_power(-defaultPower)); 
            motorPower(RightMotor, convert_power(defaultPower));
        }
        else if (Case == 5) {                               //Turning right
            motorPower(LeftMotor, convert_power(defaultPower)); 
            motorPower(RightMotor, convert_power(0));
        }
        else if (Case == 7) {                                 //Turning right more aggressively
            motorPower(LeftMotor, convert_power(defaultPower));   
            motorPower(RightMotor, convert_power(-defaultPower));
        }
        else if ((Case == 1) || (Case == 6) || (Case == 3)) {     //Driving straight when sensors detect line values "symmetrically"
            motorPower(LeftMotor, convert_power(defaultPower));
            motorPower(RightMotor, convert_power(defaultPower));
        }
    
        //Used for testing purposes:
        lcd_print(LCDLine1, "case = %d", Case);
        lcd_print(LCDLine2, "left = %d", readSensor(LeftLight));
        lcd_print(LCDLine3, "mid = %d", readSensor(MidLight));
        lcd_print(LCDLine4, "right = %d", readSensor(RightLight));

        delay(50);          //Repeats at ~20Hz
        Case = line_following_case();
    }

    //Ensures motors are at 0 when function terminated, i.e. a black line is detected
    motorPower(LeftMotor, 0);
    motorPower(RightMotor, 0);
}





