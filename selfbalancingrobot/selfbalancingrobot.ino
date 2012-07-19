#include <PID_v1.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>
#include <math.h>
#include <TimedAction.h>
#include <Button.h>        //github.com/JChristensen/Button
#include <EEPROM.h>
#include <avr/wdt.h>
#include <SerialCommand.h>
#include <FIR.h>

#define   GYR_Y                 4                              // Gyro Y (IMU pin #4)
#define   ACC_Z                 2                              // Acc  Z (IMU pin #7)
#define   ACC_X                 0                              // Acc  X

#define FILTERTAPS 5


struct Configure {
  word speedPIDKp;
  word speedPIDKi;
  word speedPIDKd;
  word speedPIDOutputLowerLimit;
  word speedPIDOutputHigherLimit;
  word anglePIDAggKp;
  word anglePIDAggKi;
  word anglePIDAggKd;
  word anglePIDConKp;
  word anglePIDConKi;
  word anglePIDConKd;
  word anglePIDLowerLimit;
  uint8_t anglePIDSampling;
  uint8_t speedPIDSampling;
  uint8_t angleSensorSampling;
  uint8_t motorSpeedSensorSampling;
  uint8_t v1;
  uint8_t v2;  
  uint8_t v3;
  uint8_t v4;
};
Configure configuration;

SerialCommand sCmd;     // The SerialCommand object

boolean started = false;

TimedAction debugTimedAction = TimedAction(200,debugEverything);
TimedAction updateMotorStatusesTimedAction = TimedAction(20,updateMotorStatuses);
TimedAction updateIMUSensorsTimedAction = TimedAction(20,updateIMUSensors);

Button startBtn(7, false, false, 20);
Button stopBtn(6, false, false, 20);
Button calibrateBtn(4, false, false, 20);

int pwm_a = 5;   //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13

float imuValues[6];
float ypr[3];
float roll;

float motorSpeed;
float leftMotorSpeed;
float rightMotorSpeed;
float motor1Calibration = 1;
float motor2Calibration = 1.4;


/* PID variables */
double anglePIDSetpoint, anglePIDInput, anglePIDOutput;
double speedPIDInput, speedPIDOutput, speedPIDSetpoint;

//Specify the links and initial tuning parameters
PID anglePID(&anglePIDInput, &anglePIDOutput, &anglePIDSetpoint, 0, 0, 0, REVERSE);
PID speedPID(&speedPIDInput, &speedPIDOutput, &speedPIDSetpoint, 0.1, 0.005, 0.001, DIRECT);

FIR rollFIR;
FIR speedFIR;

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

void setup() { 
  Serial.begin(57600);
  Serial.println("setup");
  initSerialCommand();

  loadConfig();

  Wire.begin();

  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);

  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  analogWrite(pwm_a, 0);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, 0);

  digitalWrite(dir_a, LOW);
  digitalWrite(dir_b, HIGH);

  //initMotorPIDs();
  initAnglePID();
  initSpeedPID();  
  initTimedActions();

  float rollFIRcoef[FILTERTAPS] = 
  { 
    0.021, 0.096, 0.146, 0.096, 0.021          
  };
  float gain = 0;
  for (int i=0; i<FILTERTAPS; i++) {
    gain += rollFIRcoef[i];
  }
  rollFIR.setCoefficients(rollFIRcoef);
  rollFIR.setGain(gain);

  float speedFIRcoef[FILTERTAPS] = { 
    0.000, 0.0, 0.526, 0.0, 0.00              };
  gain = 0;
  for (int i=0; i<FILTERTAPS; i++) {
    gain += speedFIRcoef[i];
  }
  speedFIR.setCoefficients(speedFIRcoef);
  speedFIR.setGain(gain);

  wdt_enable(WDTO_2S);
}

void initTimedActions() {
  updateMotorStatusesTimedAction.setInterval(configuration.motorSpeedSensorSampling);
  updateIMUSensorsTimedAction.setInterval(configuration.angleSensorSampling);
}

void initAnglePID() {
  anglePIDSetpoint = 0;
  anglePID.SetOutputLimits(-100, 100);
  //anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(configuration.anglePIDSampling);
}

void initSpeedPID() {
  speedPIDSetpoint = 0;
  speedPID.SetOutputLimits(-(float)configuration.speedPIDOutputLowerLimit/100, (float)configuration.speedPIDOutputLowerLimit/100);
  //anglePID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(configuration.speedPIDSampling);
  speedPID.SetTunings((float)configuration.speedPIDKp / 100, (float)configuration.speedPIDKi / 100, (float)configuration.speedPIDKd / 100);
}

void updateIMUSensors() {
  sixDOF.getValues(imuValues);  
  sixDOF.getYawPitchRoll(ypr);
  roll = rollFIR.process(ypr[2]);
  anglePIDInput = roll;
}


void loop() {   
  wdt_reset();

  // update sensors and sometimes debug
  updateMotorStatusesTimedAction.check();
  updateIMUSensorsTimedAction.check();
  debugTimedAction.check();

  if (started) {
    // speed pid. input is wheel speed. output is angleSetpoint
    motorSpeed = (leftMotorSpeed+rightMotorSpeed)/20;
    speedPIDInput = speedFIR.process(motorSpeed);
    speedPID.Compute();
    anglePIDSetpoint = speedPIDOutput;

    if (abs(rightMotorSpeed) < 5) {
      motor1Calibration = 1;
      motor2Calibration = 1.3;  
    }
    else if (abs(rightMotorSpeed < 10)) {
      motor1Calibration = 1;
      motor2Calibration = 1.3;
    }
    else if (abs(rightMotorSpeed < 15)) {
      motor1Calibration = 1;
      motor2Calibration = 1.2;
    }
    else {
      motor1Calibration = 1;
      motor2Calibration = 1;          
    }

    // update angle pid tuning
    if(abs(anglePIDInput) < (float)configuration.anglePIDLowerLimit / 100 && configuration.anglePIDLowerLimit != 0) { 
      //we're close to setpoint, use conservative tuning parameters
      anglePID.SetTunings((float)configuration.anglePIDConKp / 100, (float)configuration.anglePIDConKi / 100, (float)configuration.anglePIDConKd / 100);
    }
    else if (abs(anglePIDInput) < 45) {
      //we're far from setpoint, use aggressive tuning parameters
      anglePID.SetTunings((float)configuration.anglePIDAggKp / 100, (float)configuration.anglePIDAggKi / 100, (float)configuration.anglePIDAggKd / 100);
    }
    else {
      anglePID.SetTunings(0, 0, 0);
      stopMotors();
    }
    anglePID.Compute();


    // give motors new setpoints and update
    /*setMotorPIDSetpoint(1, anglePIDOutput); // höger
     setMotorPIDSetpoint(2, anglePIDOutput); // vänster*/
    moveMotor(1, anglePIDOutput);
    moveMotor(2, anglePIDOutput);
    //updateMotorPIDs(); 
  }
  else {
    moveMotor(1, 0);
    moveMotor(2, 0);      
  }

  // read serial and do commands if requested
  sCmd.readSerial();

  // check buttons and do actions if released
  startBtn.read();
  stopBtn.read();
  if (startBtn.wasReleased() || stopBtn.wasReleased()) {
    get_msg_from_console();
    saveConfig();
    debugConfiguration();

    //initMotorPIDs();
    initAnglePID();
    initSpeedPID();
    initTimedActions();
  }
  if (startBtn.wasReleased()) {
    Serial.println("startBtn.wasReleased");
    anglePID.SetMode(AUTOMATIC);
    speedPID.SetMode(AUTOMATIC);
    started = true;
  }
  if (stopBtn.wasReleased()) {
    Serial.println("stopBtn.wasReleased");
    stopMotors();
    anglePID.SetMode(MANUAL);
    speedPID.SetMode(MANUAL);
    started = false;
  }
}

void debugEverything() {
  //debugImu();
  //debugAnglePID();
  //debugSpeedPID();
  //debugMotorSpeeds();
  //debugMotorCalibrations();
  //debugMotorSpeedCalibration();
  debugChart();
  Serial.println();
}

void debugChart() {
  /*Serial.print(anglePIDOutput*motor1Calibration);
   Serial.print(",");
   Serial.print(anglePIDOutput*motor2Calibration);
   Serial.print(",");*/
  Serial.print(rightMotorSpeed);
  Serial.print(",");
  Serial.print(leftMotorSpeed);
  Serial.print(",");
  Serial.print(motorSpeed, 4);
  Serial.print(",");
  Serial.print(speedPIDInput, 4);
  Serial.print(",");
  Serial.print(speedPIDOutput, 4);
  Serial.print(",");
  Serial.print(speedPIDSetpoint, 4);
  Serial.print(",");
  Serial.print(anglePIDInput, 4);
  Serial.print(",");
  Serial.print(anglePIDOutput, 4);
  Serial.print(",");
  Serial.print(anglePIDSetpoint, 4);
}

void debugSensorValues() {
  /*Serial.print("aX: ");
   printInt(sensorValues[0], 4);
   Serial.print("\taY: ");
   printInt(sensorValues[1], 4);
   Serial.print("\taZ: ");
   printInt(sensorValues[2], 4);
   Serial.print("\tgX: ");
   printInt(sensorValues[3], 4);
   Serial.print("\tgY: ");
   printInt(sensorValues[4], 4);
   Serial.print("\tgZ: ");
   printInt(sensorValues[5], 4);
   Serial.print("\tACC_angle: ");
   printFloat(ACC_angle, 5);
   Serial.print("\tGYRO_rate: ");
   printFloat(GYRO_rate, 5);
   Serial.print("\tactAngle: ");
   printFloat(actAngle, 5);
   Serial.print("\tyaw: ");
   printFloat(ypr[0], 5);
   Serial.print("\tpitch: ");
   printFloat(ypr[1], 5);
   Serial.print("\troll: ");
   printFloat(ypr[2], 5);*/
  Serial.print("\tfirroll: ");
  printFloat(roll, 5);
  Serial.println();
}

void debugAnglePID() {
  /*Serial.print("\troll: ");
   printFloat(ypr[2], 4);*/
  Serial.print("\tanglePID I: ");
  printFloat(anglePIDInput, 4);
  Serial.print("\tO: ");
  printFloat(anglePIDOutput, 4);
  Serial.print("\tS: ");
  printFloat(anglePIDSetpoint, 4);
}

void debugSpeedPID() {
  Serial.print("\tspeedPID I: ");
  printFloat(speedPIDInput, 4);
  Serial.print("\tO: ");
  printFloat(speedPIDOutput, 4);
  Serial.print("\tS: ");
  printFloat(speedPIDSetpoint, 4);
}

void debugMotorCalibrations() {
  Serial.print("\tm1 C: ");
  printFloat(motor1Calibration, 4);
  Serial.print("\tm2 C: ");
  printFloat(motor2Calibration, 4);
}
void debugMotorSpeeds() {
  Serial.print("\tl m S: ");
  printFloat(leftMotorSpeed, 4);
  Serial.print("\tr m S: ");
  printFloat(rightMotorSpeed, 4);
}

void debugImu() {
  Serial.print("aX: ");
  printInt(imuValues[0], 4);
  Serial.print("\taY: ");
  printInt(imuValues[1], 4);
  Serial.print("\taZ: ");
  printInt(imuValues[2], 4);
  Serial.print("\tgX: ");
  printInt(imuValues[3], 4);
  Serial.print("\tgY: ");
  printInt(imuValues[4], 4);
  Serial.print("\tgZ: ");
  printInt(imuValues[5], 4);
}

void debugMotorSpeedCalibration() {
  Serial.print("\t");
  Serial.print(anglePIDOutput*motor1Calibration);
  Serial.print("\t");
  Serial.print(anglePIDOutput*motor2Calibration);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  Serial.print("\t");
  Serial.print(leftMotorSpeed);
}

void printInt(int number, byte width) {
  int currentMax = 10;
  if (number < 0) 
    currentMax = 1;
  for (byte i=1; i<width; i++){
    if (fabs(number) < currentMax) {
      Serial.print(" ");
    }
    currentMax *= 10;
  }
  Serial.print(number);
}
void printFloat(float number, byte width) {
  int currentMax = 10;
  if (number < 0) 
    currentMax = 1;
  for (byte i=1; i<width; i++){
    if (fabs(number) < currentMax) {
      Serial.print(" ");
    }
    currentMax *= 10;
  }
  Serial.print(number);
}

void debugConfiguration() {
  Serial.print("speedPIDKp: ");
  Serial.println(configuration.speedPIDKp);
  Serial.print("speedPIDKi: ");
  Serial.println(configuration.speedPIDKi);
  Serial.print("speedPIDKd: ");
  Serial.println(configuration.speedPIDKd);

  Serial.print("speedPidOutputLowerLimit: ");
  Serial.println(configuration.speedPIDOutputLowerLimit);
  Serial.print("speedPidOutputHigherLimit: ");
  Serial.println(configuration.speedPIDOutputHigherLimit);

  Serial.print("anglePIDAggKp: ");
  Serial.println(configuration.anglePIDAggKp);
  Serial.print("anglePIDAggKi: ");
  Serial.println(configuration.anglePIDAggKi);
  Serial.print("anglePIDAggKd: ");
  Serial.println(configuration.anglePIDAggKd);

  Serial.print("anglePIDConKp: ");
  Serial.println(configuration.anglePIDConKp);
  Serial.print("anglePIDConKi: ");
  Serial.println(configuration.anglePIDConKi);
  Serial.print("anglePIDConKd: ");
  Serial.println(configuration.anglePIDConKd);

  Serial.print("anglePIDLowerLimit: ");
  Serial.println(configuration.anglePIDLowerLimit);

  Serial.print("anglePIDSampling: ");
  Serial.println(configuration.anglePIDSampling);
  Serial.print("motorsPIDSampling: ");
  Serial.println(configuration.speedPIDSampling);
  Serial.print("angleSensorSampling: ");
  Serial.println(configuration.angleSensorSampling);
  Serial.print("motorSpeedSensorSampling: ");
  Serial.println(configuration.motorSpeedSensorSampling);
}


























