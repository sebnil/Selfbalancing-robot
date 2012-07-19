// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 32

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  /*if (//EEPROM.read(CONFIG_START + sizeof(settings) - 1) == settings.version_of_program[3] // this is '\0'
      EEPROM.read(CONFIG_START + sizeof(configuration) - 2) == configuration.version_of_program[2] &&
      EEPROM.read(CONFIG_START + sizeof(configuration) - 3) == configuration.version_of_program[1] &&
      EEPROM.read(CONFIG_START + sizeof(configuration) - 4) == configuration.version_of_program[0])
  { // reads settings from EEPROM*/
    for (unsigned int t=0; t<sizeof(configuration); t++)
      *((char*)&configuration + t) = EEPROM.read(CONFIG_START + t);
  /*} else {
    // settings aren't valid! will overwrite with default settings
    saveConfig();
  }*/
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(configuration); t++)
  { // writes to EEPROM
    EEPROM.write(CONFIG_START + t, *((char*)&configuration + t));
    // and verifies the data
    if (EEPROM.read(CONFIG_START + t) != *((char*)&configuration + t))
    {
      // error writing to EEPROM
    }
  }
}
