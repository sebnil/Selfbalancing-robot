#define SERIALCOMMAND_BUFFER 32
char buffer[SERIALCOMMAND_BUFFER + 1];
byte bufPos;
char *last;

void readSerialCommand() {
  while (Serial1.available() > 0) {
    char inChar = Serial1.read();   // Read single available character, there may be more waiting
    if (inChar == '\r' || inChar == '\n' || inChar == '\0') {     // Check for the terminator (default '\r') meaning end of command
      /*
      Serial1.print("Received: ");
      Serial1.println(buffer);
      */

      char *command = strtok_r(buffer, " ", &last);
      switch (command[0]) {
      case 'r':
        remoteControl();
        break;  
      }
      /*char *param1 = strtok_r(NULL, " ", &last);
       char *param2 = strtok_r(NULL, " ", &last);
       
       Serial1.print("command: ");
       Serial1.print(command);
       Serial1.print("param1: ");
       Serial1.println(param1);*/

      buffer[0] = '\0';
      bufPos = 0;
    }
    else if (isprint(inChar)) {     // Only printable characters into the buffer
      if (bufPos < SERIALCOMMAND_BUFFER) {
        buffer[bufPos++] = inChar;  // Put character into buffer
        buffer[bufPos] = '\0';      // Null terminate
      } 
      else {
        Serial1.println("Buffer full");
      }
    }
  }
}
char* next() {
  return strtok_r(NULL, " ", &last);
}

void remoteControl() {
  char *steeringTmp = strtok_r(NULL, " ", &last);
  char *throttleAndBreakTmp = strtok_r(NULL, " ", &last);
  
  steering = atoi(steeringTmp);
  speedPIDSetpoint = (float)(atoi(throttleAndBreakTmp)/100);
  
  /*Serial1.print("steeringTmp: ");
  Serial1.print(steeringTmp);
  Serial1.print(" throttleAndBreakTmp: ");
  Serial1.println(throttleAndBreakTmp);
  Serial1.print("steering: ");
  Serial1.print(steering);
  Serial1.print(" speedPIDSetpoint: ");
  Serial1.println(speedPIDSetpoint);*/

  // since we got a valid command from remote control, reset the watchdog
  remoteControlWatchdogTimedAction.reset();
}


/*void initSerialCommand() {
 sCmd.addCommand("P",     processCommand);  // Converts two arguments to integers and echos them back
 sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
 Serial1.println("Ready");
 }
 
 void processCommand() {
 int aNumber;
 char *arg;
 
 Serial1.println("We're in processCommand");
 arg = sCmd.next();
 if (arg != NULL) {
 aNumber = atoi(arg);    // Converts a char string to an integer
 Serial1.print("First argument was: ");
 Serial1.println(aNumber);
 }
 else {
 Serial1.println("No arguments");
 }
 
 arg = sCmd.next();
 if (arg != NULL) {
 aNumber = atol(arg);
 Serial1.print("Second argument was: ");
 Serial1.println(aNumber);
 }
 else {
 Serial1.println("No second argument");
 }
 }
 
 // This gets set as the default handler, and gets called when no other command matches.
 void unrecognized(const char *command) {
 Serial1.println("What?");
 }*/





