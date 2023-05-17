/*////////////////////////////////////////////////////////////////////////////////////
*  Date   : 03/2023
*  Developpers : Ouaaddi Mohamedbahae , khimouj Mounir , Nabil Oulhaj , Hicham Redone
*  National School of Applied Sciences ,University of cadi ayyad ,Marrakech ,Morroco 
*
*  The code provided below is an algorithm that uses 2 infrared  sensors for a line
*    following robot and uses PID correction to keep the robot move forward 
*
*/////////////////////////////////////////////////////////////////////////////////////

//we need to include the motors library to control them , and the PID library 
#include "AFMotor.h"
#include"PID.h"

// the output of the left seneor is connected to A2 and the output of the right sensor to A1
#define left_pin A2 
#define right_pin A1 

// here we define an object called pid and the 2 motors
PID pid(0.05,0,0.005,10,10,-300,300);
//Kp = 0.05 , Ki = 0 ,Kd = 0.005 , tau = 10s  , T = 10s ,min_output = -300 and max_output = 300

AF_DCMotor right_motor(1);
AF_DCMotor left_motor(2);

//we store the values from the infrared sensor in the following varibles
int left_value,right_value;
int mtrSpd = 70 ;//the reference speed
// the flag variable is created to stop the robot in final .
int flag=0;

void setup() {
  Serial.begin(9600);
  pinMode(left_pin, INPUT);
  pinMode(right_pin, INPUT);

  //set the motors to move forward by default
  right_motor.run(FORWARD);
  left_motor.run(FORWARD);

}

void loop() {
  //read the values 
  left_value = analogRead(left_pin);
  right_value = analogRead(right_pin);
  Serial.print(left_value);
  Serial.print("\t");
  Serial.print(right_value);

  // the error is the difference between the 2 values , and we want the difference to be 0 
  // so 0 is the setpoint in pid input
  // the output of the pid function  (s) is the quantity in wich we gonna perturbate the reference speed
  float s = pid.Up_date(0,right_value - left_value);
  Serial.print("\t");
  Serial.print(s);

  //to protect the motors, we need to limit the speed to 200
  float left_speed = constrain(mtrSpd - s, 0, 200);
  float right_speed = constrain(mtrSpd + s, 0, 200);
  left_motor.setSpeed(left_speed);
  right_motor.setSpeed(right_speed);
  Serial.print("\t");
  Serial.print(left_speed);
  Serial.print("\t");
  Serial.println(right_speed);
  
  //if the 2 sensors read black value ,then stop we reach the end
  if(left_value >= 600 && right_value  >= 600){
    left_motor.run(RELEASE);
    right_motor.run(RELEASE);
    exit;// shut down the arduino board
  }
  }

