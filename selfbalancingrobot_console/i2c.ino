/* I2C */
#define  SLAVE_ADDRESS 0x20  //slave address, any number from 0x01 to 0x7F
#define  REG_MAP_SIZE 32
byte msg[REG_MAP_SIZE];

/*
  word anglePIDAggKp;
 word anglePIDAggKi;
 word anglePIDAggKd;
 word anglePIDConKp;
 word anglePIDConKi;
 word anglePIDConKd;
 word anglePIDLowerLimit;
 word motorPIDKp;
 word motorPIDKi;
 word motorPIDKd;
 uint8_t anglePIDSampling;
 uint8_t motorsPIDSampling;
 uint8_t angleSensorSampling;
 uint8_t motorSpeedSensorSampling;
 char version_of_program[4];
 
 10*16 + 8*4 + 8*4 = 224
 384/8 = 28
 
 */

union uu {
  Configure configuration;
  unsigned char bytes[REG_MAP_SIZE];
} 
configurationUnion;

void initI2C() {
  /* I2C slave init*/
  Wire.begin(SLAVE_ADDRESS); 
  Wire.onRequest(requestEvent);  
}
void requestEvent() {
  //byte* msg = reinterpret_cast<byte*>(&configuration);
  Serial.println("requestEvent");
  configurationUnion.configuration = configuration;
  int i;
  for (i=0; i<REG_MAP_SIZE; i++) {
    msg[i] = configurationUnion.bytes[i];
    //msg[i] = i;
  }
  //Wire.write(msg, REG_MAP_SIZE);
  Wire.write(reinterpret_cast<byte*>(&configuration), REG_MAP_SIZE);

  Serial.println("requestEvent done");
  //Serial.println(msg, HEX);
  
  /*char[] msg = reinterpret_cast<char[]>(&configuration);
  Serial.println(msg, HEX);
  */
  /*for (i=0; i<REG_MAP_SIZE; i++) {
   Serial.print((uint8_t)msg[i]);
   Serial.print(" ");
   }*/
  Serial.println();
}





