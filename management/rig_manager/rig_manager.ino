#include "Arduino.h"
#include "Vrekrer_scpi_parser.h"

const int RELAYS[] = {2};
const int N_RELAYS = sizeof(RELAYS)/sizeof(RELAYS[0]);
int relayStates[N_RELAYS] = {0,};

const int OUTS[] = {8};
const int N_OUTS = sizeof(OUTS)/sizeof(OUTS[0]);
int outsStates[N_OUTS] = {0,};

SCPI_Parser my_instrument;


void setup() {
  my_instrument.RegisterCommand(F("*IDN?"), &Identify);
  my_instrument.SetCommandTreeBase(F("SYSTem"));
    my_instrument.RegisterCommand(F(":RELAY#"), &SetRelay);
    my_instrument.RegisterCommand(F(":RELAY#?"), &GetRelay);
    my_instrument.RegisterCommand(F(":DO#"), &SetOuts);
    my_instrument.RegisterCommand(F(":DO#?"), &GetOuts);



  Serial.begin(9600);

  for (int i=0; i<N_RELAYS; i++) {
    pinMode(RELAYS[i], OUTPUT);
  }

  for (int i=0; i<N_OUTS; i++) {
    pinMode(OUTS[i], OUTPUT);
  }
}

void Identify(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(F("Bitcraze,Rig Manager,#00," VREKRER_SCPI_VERSION));
  //*IDN? Suggested return string should be in the following format:
  // "<vendor>,<model>,<serial number>,<firmware>"
}

void SetRelay(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(),"%*[RELAY]%u", &suffix);

  if (parameters.Size() > 0 && suffix >= 0 && suffix < N_RELAYS) {
    relayStates[suffix] = constrain(String(parameters[0]).toInt(), 0, 10);
    digitalWrite(RELAYS[suffix], relayStates[suffix]);
  }
}

void GetRelay(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(),"%*[RELAY]%u", &suffix);

  if(suffix >= 0 && suffix < N_RELAYS) {
    interface.println(String(relayStates[suffix], DEC));
  } else {
    interface.println(String(-1, DEC));
  }
  
}

void SetOuts(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(),"%*[DO]%u", &suffix);

  if (parameters.Size() > 0 && suffix >= 0 && suffix < N_OUTS) {
    outsStates[suffix] = constrain(String(parameters[0]).toInt(), 0, 10);
    digitalWrite(OUTS[suffix], outsStates[suffix]);
  }
}

void GetOuts(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(),"%*[DO]%u", &suffix);

  if(suffix >= 0 && suffix < N_OUTS) {
    interface.println(String(outsStates[suffix], DEC));
  } else {
    interface.println(String(-1, DEC));
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  my_instrument.ProcessInput(Serial, "\n");
}
