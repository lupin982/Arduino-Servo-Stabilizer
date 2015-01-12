/*
 * SerialCom.h
 *
 * Created: 12/01/2015 23:41:18
 *  Author: Beatella
 */ 


#ifndef SERIALCOM_H_
#define SERIALCOM_H_

//************************************************************************************
//
// general parameter modification function
//
//************************************************************************************

#include "variables.h"
#include "EPROMAnything.h"
#include "generalFunctions.h"

// serial commands
SerialCommand sCmd;     // The demo SerialCommand object

bool updateYawPIDParams = false;
bool updatePitchPIDParams = false;


// types of config parameters
enum confType {
  BOOL,
  INT8,
  INT16,
  INT32,
  UINT8,
  UINT16,
  UINT32,
  DOUBLE,
  FLOAT
};

#define CONFIGNAME_MAX_LEN 17
typedef struct configDef 
{
  char name[CONFIGNAME_MAX_LEN];  // name of config parameter
  confType type;                  // type of config parameters
  void * address;                 // address of config parameter
  void (* updateFunction)(void);  // function is called when parameter update happens
} t_configDef;

t_configDef configDef;

// alternatively access config decriptor as array of bytes
typedef union {
  t_configDef   c;
  char          bytes[sizeof(t_configDef)];
} t_configUnion;

t_configUnion configUnion;

void readEEPROM();
void writeEEPROM();
void initPitchPIDs();
void initYawPIDs();
//******************************************************************************
//
// list of all config parameters
// to be accessed by par command
//
// descriptor is stored in PROGMEN to preserve RAM space
// see http://www.arduino.cc/en/Reference/PROGMEM
// and http://jeelabs.org/2011/05/23/saving-ram-space/
//
//******************************************************************************
t_configDef PROGMEM configListPGM[] = 
{
  {"configSet", UINT8, &config.configSet,	&readEEPROM}, // select another EEPROM set
  {"PitchKp",	FLOAT, &config.PitchKp,		&initPitchPIDs},
  {"PitchKi",	FLOAT, &config.PitchKi,		&initPitchPIDs},
  {"PitchKd",	FLOAT, &config.PitchKd,		&initPitchPIDs},
  {"YawKp",		FLOAT, &config.YawKp,		&initYawPIDs},
  {"YawKi",		FLOAT, &config.YawKi,		&initYawPIDs},
  {"YawKd",		FLOAT, &config.YawKd,		&initYawPIDs},
  {"", BOOL, NULL, NULL} // terminating NULL required !!
};


// read bytes from program memory
void getPGMstring (PGM_P s, char * d, int numBytes) {
  char c;
  for (int i=0; i<numBytes; i++) {
    *d++ = pgm_read_byte(s++);
  }
}

// find Config Definition for named parameter
t_configDef * getConfigDef(char * name) {

  void * addr = NULL;
  bool found = false;  
  t_configDef * p = (t_configDef *)configListPGM;

  while (true) {
    getPGMstring ((PGM_P)p, configUnion.bytes, sizeof(configDef)); // read structure from program memory
    if (configUnion.c.address == NULL) break;
    if (strncmp(configUnion.c.name, name, CONFIGNAME_MAX_LEN) == 0) {
      addr = configUnion.c.address;
      found = true;
      break;
   }
   p++; 
  }
  if (found) 
      return &configUnion.c;
  else 
      return NULL;
}


// print single parameter value
void printConfig(t_configDef * def) 
{
  if (def != NULL) 
  {
    Serial.print(def->name);
    Serial.print(F(" "));
    switch (def->type) 
	{
      case BOOL   : Serial.print(*(bool *)(def->address)); break;
      case UINT8  : Serial.print(*(uint8_t *)(def->address)); break;
      case UINT16 : Serial.print(*(uint16_t *)(def->address)); break;
      case UINT32 : Serial.print(*(uint32_t *)(def->address)); break;
      case INT8   : Serial.print(*(int8_t *)(def->address)); break;
      case INT16  : Serial.print(*(int16_t *)(def->address)); break;
      case INT32  : Serial.print(*(int32_t *)(def->address)); break;
	  case FLOAT : Serial.print(*(float *)(def->address)); break;
	  case DOUBLE : Serial.print(*(double *)(def->address)); break;
    }
    Serial.println("");
  } 
  else 
  {
     Serial.println("printConfig: illegal parameter");
  }
}

// write single parameter with value
void writeConfig(t_configDef * def, int32_t val) {
  if (def != NULL) 
  {
    switch (def->type) 
	{
      case BOOL   : *(bool *)(def->address)     = val; break;
      case UINT8  : *(uint8_t *)(def->address)  = val; break;
      case UINT16 : *(uint16_t *)(def->address) = val; break;
      case UINT32 : *(uint32_t *)(def->address) = val; break;
      case INT8   : *(int8_t *)(def->address)   = val; break;
      case INT16  : *(int16_t *)(def->address)  = val; break;
      case INT32  : *(int32_t *)(def->address)  = val; break;
      case DOUBLE  : *(double *)(def->address)  = val; break;
      case FLOAT  : *(float *)(def->address)  = val; break;
    }
    // call update function
    if (def->updateFunction != NULL) def->updateFunction();
  } 
  else 
  {
     Serial.println(F("writeConfig: illegal parameter"));  
  }
}


// print all parameters
void printConfigAll(t_configDef * p) {
  while (true) {
    getPGMstring ((PGM_P)p, configUnion.bytes, sizeof(configDef)); // read structure from program memory
    if (configUnion.c.address == NULL) break;
    printConfig(&configUnion.c);
    p++; 
  }
  Serial.println(F("done."));
  
  // show firmware version after parameter output (so it appears at older GUIs as well)
   Serial.println(F(""));

}

//******************************************************************************
// general parameter modification function
//      par                           print all parameters
//      par <parameter_name>          print parameter <parameter_name>
//      par <parameter_name> <value>  set parameter <parameter_name>=<value>
//*****************************************************************************
void parameterMod() 
{
  char * paraName = NULL;
  char * paraValue = NULL;
  
  int32_t val = 0;

  if ((paraName = sCmd.next()) == NULL) 
  {
    // no command parameter, print all config parameters
    printConfigAll((t_configDef *)configListPGM);
  } 
  else if ((paraValue = sCmd.next()) == NULL) 
  {
    // one parameter, print single parameter
    printConfig(getConfigDef(paraName));
  } 
  else 
  {
    // two parameters, set specified parameter
    val = atol(paraValue);
    writeConfig(getConfigDef(paraName), val);
  }
}
//************************************************************************************

//******************************************************************************
// set config to default
//*****************************************************************************
void updateAllParameters() 
{
  initPitchPIDs();
  initYawPIDs();
}

//******************************************************************************
// write current config set number into EEPROM 
//*****************************************************************************
void writeConfigSetNumberToEEPROM(uint8_t configSet)
{
  // write current config set number to EEPROM
  EEPROM_writeAnything(0, configSet);
}
//******************************************************************************
// write config into EEPROM
//*****************************************************************************
void writeEEPROM()
{
  const uint16_t configBlockSize = sizeof(config);
  uint8_t configSet = config.configSet;
  uint16_t configBlockAddr;

  // write current config set number to EEPROM
  writeConfigSetNumberToEEPROM(configSet);
  
  // eeprom address of the selected config set
  configBlockAddr = sizeof(configSet) + configSet*configBlockSize;
        
  config.crc8 = crcSlow((crc *)&config, sizeof(config)-1); // set proper CRC 
  EEPROM_writeAnything(configBlockAddr, config);
  
}

//******************************************************************************
// read EEPROM into config 
//*****************************************************************************
void readEEPROM()
{
  const uint16_t configBlockSize = sizeof(config);
  uint8_t configSet = config.configSet;
  uint16_t configBlockAddr;
  
  // eeprom address of the selected config set
  configBlockAddr = sizeof(configSet) + configSet*configBlockSize;
  
  EEPROM_readAnything(configBlockAddr, config); 
  if (config.crc8 == crcSlow((crc *)&config, sizeof(config)-1))
  { 
    updateAllParameters();
  } else 
  {
    // crc failed intialize directly here, as readEEPROM is void
    //printMessage(MSG_WARNING, F("EEPROM CRC failed, initialize to default"));
    setDefaultParameters();
    writeEEPROM();
  }
  
  // write current config set number to EEPROM
  writeConfigSetNumberToEEPROM(configSet);  
  config.configSet = configSet;
  
}

void sayHello() {
	char *arg;
	arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
	if (arg != NULL) {    // As long as it existed, take it
		Serial.print("Hello ");
		Serial.println(arg);
	}
	else {
		Serial.println("Hello, whoever you are");
	}
}

/*
void processCommand() {
	int aNumber;
	char *arg;

	Serial.println("We're in processCommand");
	arg = sCmd.next();
	if (arg != NULL) {
		aNumber = atoi(arg);    // Converts a char string to an integer
		Serial.print("First argument was: ");
		Serial.println(aNumber);
	}
	else 
	{
		Serial.println("No arguments");
	}

	arg = sCmd.next();
	if (arg != NULL) {
		aNumber = atol(arg);
		Serial.print("Second argument was: ");
		Serial.println(aNumber);
	}
	else {
		Serial.println("No second argument");
	}
}
*/
void printHelpUsage()
{
	Serial.println(F("commands:"));
	Serial.println(F(""));
	Serial.println(F("sd   # set defaults"));
	Serial.println(F("par <parName> <parValue> # parameter read/set command"));
	Serial.println(F(" e.g."));
	Serial.println(F("   par"));
	Serial.println(F("   par PitchKi"));
	Serial.println(F("   par PitchKi 12000"));
	Serial.println(F(""));
	Serial.println(F("he  # print help"));
	Serial.println(F(""));
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
	Serial.println("What?");
}

void initPitchPIDs(void)
{
	updatePitchPIDParams = true;
}

void initYawPIDs(void)
{
	updateYawPIDParams = true;
}


void initSerialCommands()
{
	sCmd.addCommand("HELLO", sayHello);        // Echos the string argument back
	sCmd.addCommand("par",	parameterMod);  // Converts two arguments to integers and echos them back
	sCmd.addCommand("help",	printHelpUsage);
	sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
	Serial.println("Ready");
}

#endif /* SERIALCOM_H_ */