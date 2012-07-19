#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Button.h>        //github.com/JChristensen/Button
#include <EEPROM.h>
#include <Wire.h>

// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static unsigned char __attribute__ ((progmem)) logo16_glcd_bmp[]={
  0x30, 0xf0, 0xf0, 0xf0, 0xf0, 0x30, 0xf8, 0xbe, 0x9f, 0xff, 0xf8, 0xc0, 0xc0, 0xc0, 0x80, 0x00, 
  0x20, 0x3c, 0x3f, 0x3f, 0x1f, 0x19, 0x1f, 0x7b, 0xfb, 0xfe, 0xfe, 0x07, 0x07, 0x07, 0x03, 0x00, };

struct Configure {
  word speedPIDKp;
  word speedPIDKi;
  word speedPIDKd;
  word speedPidOutputLowerLimit;
  word speedPidOutputHigherLimit;
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
Configure configuration = {
  1,2,3,4,5,6,7,8,9,10,11,12,13,14};

enum screen {
  optionsAdaptivePIDAggressive = 0,
  optionsAdaptivePIDConservative,
  optionsAdaptivePIDLimits,
  optionsSpeedPID,
  optionsSpeedPIDOutputLimits,
  optionsPIDSampling,
  optionsSensorSampling,

};

uint8_t numberOfScreens = 7;
screen activeScreen = optionsAdaptivePIDAggressive;
uint8_t activeScreenIndex = 0;

Button upper1Btn(2, false, false, 100);
Button upper2Btn(8, false, false, 100);
Button upper3Btn(9, false, false, 100);
Button upper4Btn(10, false, false, 100);
Button upper5Btn(11, false, false, 100);
Button lower1Btn(12, false, false, 100);
Button lower2Btn(A0, false, false, 100);
Button lower3Btn(A1, false, false, 100);
Button lower4Btn(A2, false, false, 100);
Button lower5Btn(A3, false, false, 100);


void setup()   {
  Serial.begin(9600);
  Serial.println("setup");

  loadConfig();
  initI2C();

  display.begin();

  drawScreen();
}  
void loop() {
  upper1Btn.read();
  upper2Btn.read();
  upper3Btn.read();
  upper4Btn.read();
  upper5Btn.read();
  lower1Btn.read();
  lower2Btn.read();
  lower3Btn.read();
  lower4Btn.read();
  lower5Btn.read();

  // switch screen
  if (upper5Btn.wasReleased()) {
    activeScreen = (screen)((activeScreen+1)%numberOfScreens);
  }
  if (lower5Btn.wasReleased()){
    if (activeScreen == 0)
      activeScreen = (screen)(numberOfScreens-1);
    else
      activeScreen = (screen)(activeScreen-1);
  }

  // parameterIncrement. specifies if fine or rough paramater incrementations are used.
  float parameterIncrement = 10;
  if (lower4Btn.isPressed())
    parameterIncrement = 1;
  else if (upper4Btn.isPressed())
    parameterIncrement = 100;

  // do the button stuff for every scrren
  switch (activeScreen) {
  case optionsAdaptivePIDAggressive:
    if (upper1Btn.wasReleased())
      configuration.anglePIDAggKp += parameterIncrement;
    if (lower1Btn.wasReleased()) 
      configuration.anglePIDAggKp -= parameterIncrement;

    if (upper2Btn.wasReleased())
      configuration.anglePIDAggKi += parameterIncrement;
    if (lower2Btn.wasReleased())
      configuration.anglePIDAggKi -= parameterIncrement;

    if (upper3Btn.wasReleased())
      configuration.anglePIDAggKd += parameterIncrement;
    if (lower3Btn.wasReleased())
      configuration.anglePIDAggKd -= parameterIncrement;
    break;  

  case optionsAdaptivePIDConservative:
    if (upper1Btn.wasReleased())
      configuration.anglePIDConKp += parameterIncrement;
    if (lower1Btn.wasReleased()) 
      configuration.anglePIDConKp -= parameterIncrement;

    if (upper2Btn.wasReleased())
      configuration.anglePIDConKi += parameterIncrement;
    if (lower2Btn.wasReleased())
      configuration.anglePIDConKi -= parameterIncrement;

    if (upper3Btn.wasReleased())
      configuration.anglePIDConKd += parameterIncrement;
    if (lower3Btn.wasReleased())
      configuration.anglePIDConKd -= parameterIncrement;
    break;  

  case optionsAdaptivePIDLimits:
    if (upper1Btn.wasReleased())
      configuration.anglePIDLowerLimit += parameterIncrement;
    if (lower1Btn.wasReleased()) 
      configuration.anglePIDLowerLimit -= parameterIncrement;
    break;

  case optionsSpeedPID:
    if (upper1Btn.wasReleased())
      configuration.speedPIDKp += parameterIncrement;
    if (lower1Btn.wasReleased()) 
      configuration.speedPIDKp -= parameterIncrement;

    if (upper2Btn.wasReleased())
      configuration.speedPIDKi += parameterIncrement;
    if (lower2Btn.wasReleased())
      configuration.speedPIDKi -= parameterIncrement;

    if (upper3Btn.wasReleased())
      configuration.speedPIDKd += parameterIncrement;
    if (lower3Btn.wasReleased())
      configuration.speedPIDKd -= parameterIncrement;
    break;  

  case optionsSpeedPIDOutputLimits:
    if (upper1Btn.wasReleased())
      configuration.speedPidOutputLowerLimit += parameterIncrement;
    if (lower1Btn.wasReleased()) 
      configuration.speedPidOutputLowerLimit -= parameterIncrement;
    if (upper2Btn.wasReleased())
      configuration.speedPidOutputHigherLimit += parameterIncrement;
    if (lower2Btn.wasReleased()) 
      configuration.speedPidOutputHigherLimit -= parameterIncrement;
    break;

  case optionsPIDSampling:
    if (upper1Btn.wasReleased())
      configuration.anglePIDSampling += (int)(parameterIncrement/10);
    if (lower1Btn.wasReleased()) 
      configuration.anglePIDSampling -= (int)(parameterIncrement/10);

    if (upper2Btn.wasReleased())
      configuration.speedPIDSampling += (int)(parameterIncrement/10);
    if (lower2Btn.wasReleased())
      configuration.speedPIDSampling -= (int)(parameterIncrement/10);
    break;  

  case optionsSensorSampling:
    if (upper1Btn.wasReleased())
      configuration.angleSensorSampling += (int)(parameterIncrement/10);
    if (lower1Btn.wasReleased()) 
      configuration.angleSensorSampling -= (int)(parameterIncrement/10);

    if (upper2Btn.wasReleased())
      configuration.motorSpeedSensorSampling += (int)(parameterIncrement/10);
    if (lower2Btn.wasReleased())
      configuration.motorSpeedSensorSampling -= (int)(parameterIncrement/10);
    break;  

  }
  if (upper1Btn.wasReleased() || upper2Btn.wasReleased() || upper3Btn.wasReleased() || upper4Btn.wasReleased() || upper5Btn.wasReleased()
    || lower1Btn.wasReleased() || lower2Btn.wasReleased() || lower3Btn.wasReleased() || lower4Btn.wasReleased() || lower5Btn.wasReleased()) {
    drawScreen();
    saveConfig();
    debugConfiguration();
  }
}
void drawScreen() {
  Serial.print("activeScreen: ");
  Serial.println(activeScreen);

  display.clearDisplay();
  display.setTextSize(0.2);
  display.setTextColor(BLACK);
  display.setCursor(0,0);

  switch (activeScreen) {
  case   optionsAdaptivePIDAggressive:
    display.println("Adapt. PID agg");
    display.print("Kp: ");
    display.println((float)configuration.anglePIDAggKp / 100);
    display.print("Ki: ");
    display.println((float)configuration.anglePIDAggKi / 100);
    display.print("Kd: ");
    display.print((float)configuration.anglePIDAggKd / 100);
    break;
  case   optionsAdaptivePIDConservative:
    display.println("Adapt. PID con");
    display.print("Kp: ");
    display.println((float)configuration.anglePIDConKp / 100);
    display.print("Ki: ");
    display.println((float)configuration.anglePIDConKi / 100);
    display.print("Kd: ");
    display.print((float)configuration.anglePIDConKd / 100);
    break;
  case   optionsAdaptivePIDLimits:
    display.println("Adapt. PID");
    display.println("Cons. limit:\n");
    display.print("angle < ");
    display.print((float)configuration.anglePIDLowerLimit / 100);
    break;
  case   optionsSpeedPID:
    display.println("Speed PID");
    display.print("Kp: ");
    display.println((float)configuration.speedPIDKp / 100);
    display.print("Ki: ");
    display.println((float)configuration.speedPIDKi / 100);
    display.print("Kd: ");
    display.print((float)configuration.speedPIDKd / 100);
    break;
  case   optionsSpeedPIDOutputLimits:
    display.println("Speed PID O L");
    display.print("lo: -");
    display.println((float)configuration.speedPidOutputLowerLimit / 100);
    display.print("hi: ");
    display.print((float)configuration.speedPidOutputHigherLimit / 100);
    break;
  case   optionsPIDSampling:
    display.println("PID sampling");
    display.print("Angle: ");
    display.println(configuration.anglePIDSampling);
    display.print("Speed: ");
    display.println(configuration.speedPIDSampling);
    break;
  case   optionsSensorSampling:
    display.println("Sensor samplin\n");
    display.print("Angle: ");
    display.println(configuration.angleSensorSampling);
    display.print("Motors: ");
    display.println(configuration.motorSpeedSensorSampling);
    break;
  }  

  display.display();
}
void debugConfiguration() {
  Serial.print("anglePIDAggKp: ");
  Serial.println(configuration.anglePIDAggKp);
  Serial.print("anglePIDAggKi: ");
  Serial.println(configuration.anglePIDAggKi);
  Serial.print("anglePIDAggKd: ");
  Serial.println(configuration.anglePIDAggKd);
}



























