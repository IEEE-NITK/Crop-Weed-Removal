//
//
// I am learning to code, working of AtMega328 and other chips by going through documentations
// And side by side looking into how a robotic arm works, learning how to simulate, linear actuators, etc
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

/*RECEIVE DATA FROM the second task (path finding)*/
/*Under progress*/
