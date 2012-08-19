#include "TimedAction.h"
#include <Wire.h>

bool debug = true;
#define speedMultiplier 1

/* I2C */
#define  SLAVE_ADDRESS 0x29  //slave address, any number from 0x01 to 0x7F
#define  REG_MAP_SIZE 2
#define  MAX_SENT_BYTES 3
byte msg[REG_MAP_SIZE];
byte registerMapTemp[REG_MAP_SIZE - 1];
byte receivedCommands[MAX_SENT_BYTES];

/* Encoders */
#define leftEncoder1 2
#define leftEncoder2 4
#define rightEncoder1 3
#define rightEncoder2 5

volatile long leftMotorPosition  = 0;
volatile long rightMotorPosition = 0;
long lastLeftMotorPosition  = 0;
long lastRightMotorPosition  = 0;
int leftMotorSpeed;
int rightMotorSpeed;

int loopCounter = 0;
int lastSpeedUpdate = 0;
int lastDebugEncoders = 0;

TimedAction debugEncodersTimedAction = TimedAction(100,debugEncoders);
TimedAction updateMotorSpeedsTimedAction = TimedAction(20,updateMotorSpeeds);

void setup() {
  if (debug)
    Serial.begin(9600);

  /* Setup encoders */
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT); 
  attachInterrupt(0,leftEncoder,RISING); // pin 2
  attachInterrupt(1,rightEncoder,RISING); // pin 3

  /* I2C slave init*/
  Wire.begin(SLAVE_ADDRESS); 
  Wire.onRequest(requestEvent);
  //Wire.onReceive(receiveEvent);
}

void requestEvent() {
  /* 32 bit signed long leftWheelPosition
   32 bit signed long leftWheelPosition
   8 bit signed char left speed
   8 bit signed char right speed
   */
  //byte msg[10];
  /*long leftMotorPosition = -1000;
   long rightMotorPosition = 2000000000;
   int leftMotorSpeed = -128;
   int rightMotorSpeed = 126;*/
  /*msg[0] = (byte )((leftMotorPosition >> 24) & 0xff);
   msg[1] = (byte )((leftMotorPosition >> 16) & 0xff);
   msg[2] = (byte )((leftMotorPosition >> 8) & 0xff);
   msg[3] = (byte )(leftMotorPosition & 0xff);
   
   msg[4] = (byte )((rightMotorPosition >> 24) & 0xff);
   msg[5] = (byte )((rightMotorPosition >> 16) & 0xff);
   msg[6] = (byte )((rightMotorPosition >> 8) & 0xff);
   msg[7] = (byte )(rightMotorPosition & 0xff);*/

  msg[0] = (byte )leftMotorSpeed*speedMultiplier;
  msg[1] = (byte )rightMotorSpeed*speedMultiplier;

  /*msg[10] = (byte)1;
   msg[11] = (byte)rightMotorSpeed;
   msg[12] = (byte)150;
   msg[13] = (byte)300;*/

  /*for (int i=0; i<REG_MAP_SIZE; i++) {
   Serial.print(msg[i], DEC);
   Serial.print(" ");
   }
   Serial.println();*/

  Wire.write(msg, REG_MAP_SIZE);  //Set the buffer up to send all 14 bytes of data
}

void receiveEvent(int bytesReceived) {
  for (int a = 0; a < bytesReceived; a++)
  {
    if ( a < MAX_SENT_BYTES)
    {
      receivedCommands[a] = Wire.read();
    }
    else
    {
      Wire.read();  // if we receive more data then allowed just throw it away
    }
  }
}

void debugEncoders() {
  Serial.print(leftMotorPosition);
  Serial.print(" ");
  Serial.print(rightMotorPosition);
  Serial.print(" ");
  Serial.print(leftMotorSpeed*speedMultiplier);
  Serial.print(" ");
  Serial.println(rightMotorSpeed*speedMultiplier);

}
void updateMotorSpeeds() {
  leftMotorSpeed = leftMotorPosition - lastLeftMotorPosition;
  rightMotorSpeed = rightMotorPosition - lastRightMotorPosition;
  lastLeftMotorPosition = leftMotorPosition;
  lastRightMotorPosition = rightMotorPosition;  
}
void loop() {
  if (debug)
    debugEncodersTimedAction.check();
  updateMotorSpeedsTimedAction.check();
}

void leftEncoder() { 
  if(PIND & _BV(PIND4)) // read pin 4
      leftMotorPosition++;
  else
    leftMotorPosition--;    
}
void rightEncoder() {
  if(PIND & _BV(PIND5)) // read pin 5
      rightMotorPosition++;
  else
    rightMotorPosition--;  
}



