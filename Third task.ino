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
