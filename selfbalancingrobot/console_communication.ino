#define CONSOLE_I2C_ADDRESS 0x20
#define CONSOLE_I2C_MSG_SIZE 32
char consoleMsg[CONSOLE_I2C_MSG_SIZE];

void get_msg_from_console() {  
  Wire.beginTransmission(CONSOLE_I2C_ADDRESS);
  Wire.requestFrom(CONSOLE_I2C_ADDRESS,CONSOLE_I2C_MSG_SIZE);
  Wire.endTransmission();

  //Serial.println("get_msg_from_console");

  union uu {
    Configure configuration;
    unsigned char bytes[CONSOLE_I2C_MSG_SIZE];
  } 
  configurationUnion;

  for (int i=0; i<CONSOLE_I2C_MSG_SIZE; i++) {
    configurationUnion.bytes[i] = Wire.read();
    //Serial.print(configurationUnion.bytes[i]);
    //Serial.print(" ");
  }
  //Serial.println("get_msg_from_console2");

  configuration = (configurationUnion.configuration);

  /*configuration = (configurationUnion.configuration);*/
  /*Serial.print(configuration.anglePIDAggKp);
  Serial.print(" ");
  Serial.print(configuration.anglePIDAggKi);
  Serial.print(" ");
  Serial.print(configuration.anglePIDAggKd);
  Serial.print(" ");
  Serial.print(configuration.anglePIDConKp);
  Serial.print(" ");
  Serial.print(configuration.anglePIDConKi);
  Serial.print(" ");
  Serial.print(configuration.anglePIDConKd);
  Serial.print(" ");
  Serial.print(configuration.anglePIDLowerLimit);
  Serial.print(" ");
  Serial.print(configuration.motorPIDKp);
  Serial.print(" ");
  Serial.print(configuration.motorPIDKi);
  Serial.print(" ");
  Serial.print(configuration.motorPIDKd);
  Serial.print(" ");
  Serial.print(configuration.anglePIDSampling);
  Serial.print(" ");
  Serial.print(configuration.motorsPIDSampling);
  Serial.print(" ");
  Serial.print(configuration.angleSensorSampling);
  Serial.print(" ");
  Serial.print(configuration.motorSpeedSensorSampling);
  Serial.print(" ");

  Serial.println();
  Serial.println();*/

}

/*void get_msg_from_console2() {  
 Wire.beginTransmission(CONSOLE_I2C_ADDRESS);
 Wire.requestFrom(CONSOLE_I2C_ADDRESS,CONSOLE_I2C_MSG_SIZE);
 Wire.endTransmission();
 
 union uu {
 Configure configuration;
 unsigned char bytes[CONSOLE_I2C_MSG_SIZE];
 } 
 configurationUnion;
 for (int i=0; i<CONSOLE_I2C_MSG_SIZE; i++) {
 consoleMsg[i] = Wire.read();
 configurationUnion.bytes[i] = consoleMsg[i];
 }
 for (int i=0; i<CONSOLE_I2C_MSG_SIZE; i++) {
 Serial.print((uint8_t)consoleMsg[i]);
 Serial.print(" ");
 }
 Serial.println();
 
 Serial.print(configurationUnion.configuration.anglePIDAggKp);
 Serial.print(" ");
 Serial.print(configurationUnion.configuration.anglePIDAggKi);
 Serial.print(" ");
 Serial.print(configurationUnion.configuration.anglePIDAggKd);
 Serial.print(" ");
 Serial.print(configurationUnion.configuration.anglePIDConKp);
 Serial.print(" ");
 Serial.print(configurationUnion.configuration.anglePIDConKi);
 Serial.print(" ");
 Serial.print(configurationUnion.configuration.anglePIDConKd);
 Serial.print(" ");
 Serial.println();
 Serial.println();
 }
 */

