//
//
// I am learning to code, working of AtMega328 and other chips by going through documentations
// And side by side looking into how a pick and place robotic arm works, learning how to simulate in, linear actuators, etc
// GPS feature looks a bit difficult because we need a bigger arena or accurate positioning by keeping a constant video capture overhead
//
//


/* include library */
#include <ESP8266WiFi.h>

/* define port */
WiFiClient client;
WiFiServer server(80);

/* WIFI settings */
const char* ssid = "userssid";
const char* password = "passwordofwifi";

/* data received from application */
String  data =""; 

/* lets define L293D motor control pins */
int leftMotorForward = 2;     /* assume GPIO2(D4) -> IN3   */
int rightMotorForward = 15;   /* assume GPIO15(D8) -> IN1  */
int leftMotorBackward = 0;    /* assume GPIO0(D3) -> IN4   */
int rightMotorBackward = 13;  /* assume GPIO13(D7) -> IN2  */


/* define L293D enable pins */
int rightMotorENB = 14; /* assume GPIO14(D5) -> Motor-A Enable */
int leftMotorENB = 12;  /* assume GPIO12(D6) -> Motor-B Enable */

void setup()
{
  /* initialize motor control pins as output */
  pinMode(leftMotorForward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT); 
  pinMode(leftMotorBackward, OUTPUT);  
  pinMode(rightMotorBackward, OUTPUT);

  /* initialize motor enable pins as output */
  pinMode(leftMotorENB, OUTPUT); 
  pinMode(rightMotorENB, OUTPUT);

  /* start server communication */
  server.begin();
}

void loop()
{
    /* If the server available, run the "checkClient" function */  
    client = server.available();
    if (!client) return; 
    data = checkClient ();

/* Run function according to incoming data*/

    /* If the incoming data is "forward", run the "MotorForward" function */
    if (data == "forward") MotorForward();
    /* If the incoming data is "backward", run the "MotorBackward" function */
    else if (data == "backward") MotorBackward();
    /* If the incoming data is "left", run the "TurnLeft" function */
    else if (data == "left") TurnLeft();
    /* If the incoming data is "right", run the "TurnRight" function */
    else if (data == "right") TurnRight();
    /* If the incoming data is "stop", run the "MotorStop" function */
    else if (data == "stop") MotorStop();
} 

/*FORWARD*/
void MotorForward(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH);
  digitalWrite(leftMotorForward,HIGH);
  digitalWrite(rightMotorForward,HIGH);
  digitalWrite(leftMotorBackward,LOW);
  digitalWrite(rightMotorBackward,LOW);
}

/*BACKWARD*/
void MotorBackward(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH);
  digitalWrite(leftMotorBackward,HIGH);
  digitalWrite(rightMotorBackward,HIGH);
  digitalWrite(leftMotorForward,LOW);
  digitalWrite(rightMotorForward,LOW);
}

/*TURN LEFT*/
void TurnLeft(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH); 
  digitalWrite(leftMotorForward,LOW);
  digitalWrite(rightMotorForward,HIGH);
  digitalWrite(rightMotorBackward,LOW);
  digitalWrite(leftMotorBackward,HIGH);  
}

/*TURN RIGHT*/
void TurnRight(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH);
  digitalWrite(leftMotorForward,HIGH);
  digitalWrite(rightMotorForward,LOW);
  digitalWrite(rightMotorBackward,HIGH);
  digitalWrite(leftMotorBackward,LOW);
}

/*STOP*/
void MotorStop(void)   
{
  digitalWrite(leftMotorENB,LOW);
  digitalWrite(rightMotorENB,LOW);
  digitalWrite(leftMotorForward,LOW);
  digitalWrite(leftMotorBackward,LOW);
  digitalWrite(rightMotorForward,LOW);
  digitalWrite(rightMotorBackward,LOW);
}

///////////////////////////////////////////////////////////////////////////////
/*
#include <Wire.h>
#include <ESP8266WiFi.h>
//#include <MPU6050.h>

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float pid, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
double kp = 3.55;//3.55
double ki = 0.005;//0.003
double kd = 2.05;//2.05
double throttle=1300; //initial value of throttle to the motors
//float desired_angle = 0; 

WiFiClient client;
WiFiServer server(80);
const char* ssid = "hotspot";
const char* password = "password";

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  //right_prop.attach(3); //attatch the right motor to pin 3
  //left_prop.attach(5);  //attatch the left motor to pin 5

  server.begin();
  
  time = millis(); //Start counting time in milliseconds
  delay(7000);
}

void loop() {
  // put your main code here, to run repeatedly:
  client = server.available();
  if (!client) return; 
  data = checkClient ();


  
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;
   
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers
  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();
  Gyro_angle[0] = Gyr_rawX/131.0;
  Gyro_angle[1] = Gyr_rawY/131.0;
  
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

  error = Total_angle[1] - desired_angle;
  pid_p = kp*error;
  if(-3 <error <3)
  {
    pid_i = pid_i+(ki*error);  
  }
  pid_d = kd*((error - previous_error)/elapsedTime);
  if (abs(error) >= 5.0){
  pid = pid_p + pid_i + pid_d;
  if(pid < -1000)
  {
    pid = -1000;
  }
  if(pid > 1000)
  {
    pid = 1000;
  }
  pwmLeft = throttle + PID;
  pwmRight = throttle - PID;
  moveBot(pwmLeft, pwmRight, 0);
  }
  else
  {
    pastErrors = 0;
    pastError = 0;
    Serial.println("All done!");
    moveBot(0, 0, 0);
  }
}

void moveBot(int left, int right, int del){
  if (left < 0){
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in2, LOW);
  }
  if (right < 0){
    digitalWrite(in3, LOW);
  }
  else{
    digitalWrite(in3, HIGH);
  }
  int sL = abs(left);
  int sR = abs(right);
  analogWrite(12, sL);
  analogWrite(13, sR);    
  if (del) delay(del);
}
*/
//////////////////////////////////////////////////////////////////////////////

//RECEIVE DATA FROM the second task (path finding)
//Under progress
//In python file:
//Arduino = serial.Serial('COM3',9600) #Create Serial port object called arduinoSerialData
//time.sleep(2) #wait for 2 seconds for the communication to get established
//Arduino.write(b'f')
//print("Forward")
//time.sleep(1)
//
//Meanwhile in ino
//loop:
//while (Serial.available()){
//    data = Serial.read();
//    command = String(data);
//    moveRobot(command);
//}
//In moveRobot function, 
//void moveRobot(String motion){
//
//  if(motion == "f"){  // RW - Fwd(10 - Pos, 11 - Neg); LW - Fwd(12 - Pos, 13 - Neg)
//    digitalWrite(10,HIGH);
//    digitalWrite(11,LOW);
//    digitalWrite(12,HIGH);
//    digitalWrite(13,LOW);
//    Serial.println("Forward");
//  }

//Robtic arm (Literaly don't know much about it, but found it very intresting and intriguing)
//Trying out my logic
//
//Include libraries
//#include <Servo.h>

//define variables
//#define DEBUG true //display ESP8266 messages on Serial Monitor
//#define SERV1 8    //servo 1 on digital port 8
//#define SERV2 9    //servo 2 on digital port 9
//#define SERV3 10   //servo 3 on digital port 10
//
//don't know how many servos we need, maybe 5-6 is enough
//
//Servo s1; //servo 1
//Servo s2; //servo 2
//Servo s3; //servo 3
//
//
//define starting angle for each servo
//choose a safe position to start from
//it will try to move instantaniously to that position when powered up
//those angles will depend on the angle of each servo during the assemble
//int angle1 = 90; //servo 1 current angle
//int angle2 = 30; //servo 2 current angle
//int angle3 = 0;  //servo 3 current angle
//
//Currently Working on
//The code which will increase/decrease the angle of each servo in small steps until it matches received setpoint. 
//In the end of each cycle a delay is used to limit the speed of the motors.
