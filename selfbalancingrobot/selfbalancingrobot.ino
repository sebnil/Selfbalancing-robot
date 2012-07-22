#include <PID_v1.h> //github.com/mwoodward/Arduino-PID-Library
#include <FreeSixIMU.h> // imu. www.varesano.net/projects/hardware/FreeIMU#library
#include <FIMU_ADXL345.h> // imu
#include <FIMU_ITG3200.h> // imu
#include <Wire.h> // for i2c
#include <math.h> // where would we be without math?
#include <TimedAction.h> // for updating sensors and debug
#include <Button.h>        //github.com/JChristensen/Button
#include <EEPROM.h> // for storing configuraion
#include <avr/wdt.h> // watchdog
#include <FIR.h>  //github.com/sebnil/FIR-filter-Arduino-Library
#include <MovingAvarageFilter.h> //github.com/sebnil/Moving-Avarage-Filter--Arduino-Library-

// configure struct that is stored in eeprom and updated from the lcd console.
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

//SerialCommand sCmd;     // The SerialCommand object

boolean started = false; // if the robot is started or not

// these take care of the timing of things
TimedAction debugTimedAction = TimedAction(200,debugEverything);
TimedAction updateMotorStatusesTimedAction = TimedAction(20,updateMotorStatuses);
TimedAction updateIMUSensorsTimedAction = TimedAction(20,updateIMUSensors);

// button declarations
Button startBtn(7, false, false, 20);
Button stopBtn(6, false, false, 20);
Button calibrateBtn(4, false, false, 20);

// motor controller
int pwm_a = 5;   //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13

// imu variables
float imuValues[6];
float ypr[3];
float roll;

// motor speeds and calibrations
float motorSpeed;
float leftMotorSpeed;
float rightMotorSpeed;
float motor1Calibration = 1;
float motor2Calibration = 1.4;

// PID variables
double anglePIDSetpoint, anglePIDInput, anglePIDOutput;
double speedPIDInput, speedPIDOutput, speedPIDSetpoint;

// The cascading PIDs. The tunings are updated from the code
PID anglePID(&anglePIDInput, &anglePIDOutput, &anglePIDSetpoint, 0, 0, 0, REVERSE);
PID speedPID(&speedPIDInput, &speedPIDOutput, &speedPIDSetpoint, 0, 0, 0, DIRECT);

// filters
FIR rollFIR;
FIR speedFIR;
MovingAvarageFilter speedMovingAvarageFilter(20);

// Set the FreeSixIMU object. This handles the communcation to the IMU.
FreeSixIMU sixDOF = FreeSixIMU();

void setup() { 
  Serial.begin(57600);
  Serial.println("setup");
  //initSerialCommand();

  //Set control pins to be outputs
  pinMode(pwm_a, OUTPUT);  
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  digitalWrite(dir_a, LOW);
  digitalWrite(dir_b, HIGH);

  // stop the motors if they are running
  stopMotors();

  // load config from eeprom
  loadConfig();

  // init i2c and IMU
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);

  // init PIDs
  initAnglePID();
  initSpeedPID();  
  //initMotorPIDs();

  // init the timers
  initTimedActions();

  // init the filters
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
    0.000, 0.0, 50.0, 0.0, 0.00                          };
  gain = 0;
  for (int i=0; i<FILTERTAPS; i++) {
    gain += speedFIRcoef[i];
  }
  speedFIR.setCoefficients(speedFIRcoef);
  speedFIR.setGain(gain);

  // set the watchdog to 2s (this will restart the arduino if it freezes)
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
    /*setMotorPIDSetpoint(1, anglePIDOutput); // pid control on right motor (not used since it is too slow)
     setMotorPIDSetpoint(2, anglePIDOutput); // left */
    //updateMotorPIDs(); 
    moveMotor(1, anglePIDOutput);
    moveMotor(2, anglePIDOutput);

  }
  else {
    moveMotor(1, 0);
    moveMotor(2, 0);      
  }

  // read serial and do commands if requested (commented since i could not get it working on Arduino Leonardo)
  //sCmd.readSerial();

  // check buttons and do actions if released
  startBtn.read();
  stopBtn.read();
  if (startBtn.wasReleased() || stopBtn.wasReleased()) {
    get_msg_from_console();
    saveConfig();
    debugConfiguration();

    //initMotorPIDs(); // pids on motors are too slow
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

/* just debug functions. uncomment the debug information you want in debugEverything */
void debugEverything() {
  //debugImu();
  //debugAnglePID();
  //debugSpeedPID();
  //debugMotorSpeeds();
  //debugMotorCalibrations();
  //debugMotorSpeedCalibration();
  debugChart2();
  //Serial.println();
}

void debugChart() {
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

void debugChart2() {
  sendPlotData("rightMotorSpeed", rightMotorSpeed);
  sendPlotData("leftMotorSpeed", leftMotorSpeed);
  sendPlotData("motorSpeed", motorSpeed);
  sendPlotData("speedPIDInput", speedPIDInput);
  sendPlotData("speedPIDOutput", speedPIDOutput);
  sendPlotData("speedPIDSetpoint", speedPIDSetpoint);
  sendPlotData("anglePIDInput", anglePIDInput);
  sendPlotData("anglePIDOutput", anglePIDOutput);
  sendPlotData("anglePIDSetpoint", anglePIDSetpoint);
}
void sendPlotData(String seriesName, float data) {
  Serial.print("{");
  Serial.print(seriesName);
  Serial.print(",T,");
  Serial.print(data);
  Serial.println("}");
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




