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
}


