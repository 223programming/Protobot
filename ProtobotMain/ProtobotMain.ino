#include <Servo.h>
#include <EEPROM.h>
  
  bool Enabled = false;
  
  static const uint8_t AnalogInputCount = 4;
  static const uint8_t DigitalInputCount = 2;
  static const uint8_t MotorCount = AnalogInputCount + DigitalInputCount;

  int16_t AnalogInput_min[AnalogInputCount];
  int16_t AnalogInput_max[AnalogInputCount];

  uint8_t AnalogPin[] = { A0, A1, A3, A4};
  uint8_t FwdPin[] = {8, 10};
  uint8_t RevPin[] = {9, 11};

  Servo motors[MotorCount];                      // Declare all of the motor objects

  String BaseCommand = "";
  String Argument = "";
/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/
  
void setup() {
  pinMode(12, INPUT_PULLUP);                    // Set the enable - switch to be pulled up on the Arduino
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial.println("/nInitializing ProtoBot");
  InitializeProtoBot();
}

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

void loop() {
  ExecuteCommand( GetCommandData( 0xA, 10));    // Process incoming commands. Default termination character is a newline and max readlength is 12
  Enabled = !digitalRead(12);    // Get if switch is in 'enabled' positiont
  
  int8_t output[MotorCount];
  if(Enabled) {                                 // Only output data from user input if switch is set to 'enabled'
    for(uint8_t i=0; i < AnalogInputCount; i++)  {
      output[i] = GetAnalogAxisData ( AnalogInput_min[i], AnalogInput_max[i], AnalogPin[i]);
    }
    for(uint8_t i =0; i < DigitalInputCount; i++) {
      output[i + AnalogInputCount] = GetDigitalAxisData(FwdPin[i], RevPin[i]);
    }
  }
  else {
    for(uint8_t i=0; i < MotorCount; i++) {
      output[i] =0;
    }
  }
  SetMotorsOutput(motors, output);
  delay(100);
}

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

 String GetCommandData(uint8_t termChar, uint8_t MaxCharsToRead) {
  //parses commands out of incoming serial data

  uint8_t ReadLength = 0;
  uint8_t BytesInBuffer = Serial.available();
  uint8_t NextByte = 0;
  String Output;
  if(!Serial){    // Do not bother reading if we are not connected
    return "NoConnection";
  }
  else  {
    if( BytesInBuffer > MaxCharsToRead) {   // Set the amount of characters that we want to read.
      ReadLength = MaxCharsToRead;              // If none are available, then return.
    }
    else if( BytesInBuffer != 0) {
      ReadLength = BytesInBuffer;
    }
    else {
      return "BufferEmpty";
    } 
    for(uint8_t i = 0; i <= ReadLength; i++) {
      NextByte = Serial.read();
      if((NextByte == termChar) || (NextByte == 0)) {
        break;                                // The termination char was reached. Terminate the read.
      }
      Output += char(NextByte);
    }
    return Output;
  }
 }

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

void ExecuteCommand (String CommandData) {
  Serial.println(CommandData);
  if(!(CommandData == "NoConnection" || CommandData == "BufferEmpty")) {
    uint8_t AxisSelector = 0;
    for(uint8_t i=0; i<CommandData.length(); i++) {                                         // Split the command into a base command and an argument
      if(CommandData.charAt(i) == '_')  {
        BaseCommand = CommandData.substring(0, (i-1));
        Argument = CommandData.substring(i+1);
        return;
      }
    }
    if(BaseCommand == "SetMin") {                                                           // Take the current value at the selectedd analog axis and
      AxisSelector = ArgumentStringToAxisNum (Argument);                                    // save it to EEPROM and the global varibles as the new axis minimum
      AnalogInput_min[AxisSelector] = analogRead(AnalogPin[AxisSelector]);
      EEPROM_WriteWord((AxisSelector * AnalogInputCount), AnalogInput_min[AxisSelector]);
    }
    if(BaseCommand == "SetMax") {                                                           // Take the current value at the selectedd analog axis and
      AxisSelector = ArgumentStringToAxisNum (Argument);                                    // save it to EEPROM and the global varibles as the new axis maximum
      AnalogInput_max[AxisSelector] = analogRead(AnalogPin[AxisSelector]);
      EEPROM_WriteWord(((AxisSelector * AnalogInputCount) + 2), AnalogInput_max[AxisSelector]);
    }
  }
  return;
  }

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

void InitializeProtoBot() {
  Serial.println("Initializing ProtoBot");
  for(uint8_t i=0; i < MotorCount; i++)  {                                                  // Create the Servo objects and bind them to their pins
    motors[i].attach(i+2);
  }
  for(uint8_t i=0; i < AnalogInputCount; i++)  {                                                   // Load the analog axis configurations from EEPROM. There are
    AnalogInput_min[i] = EEPROM_ReadWord(AnalogInputCount * i);                                   // only 4 sets being loaded, as we only have 4 analog axes.
    AnalogInput_max[i] = EEPROM_ReadWord((AnalogInputCount * i) + 2);
  }
}
/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

void SetMotorsOutput (Servo motors[], int8_t output[]) {
  //Sets all of the output speeds on the motors
  for(uint8_t i=0; i < MotorCount; i++)  {                                                 // if the output is not between +- 90, coerce it in
    if(output[i] > 90) {
      output[i] = 90;
    }
    else if(output[i] < -90) {
      output[i] = -90;
    }
    motors[i].write(output[i] + 90);      // Write the output to the motor
  }
}

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

int16_t GetAnalogAxisData (int16_t max_input, int16_t min_input,int16_t pin) {
  return map(analogRead(pin), min_input, max_input, -90, 90);
}

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

int16_t GetDigitalAxisData(int16_t FwdPin, int16_t RevPin)  {
  return (90 * (!digitalRead(FwdPin) - !digitalRead(RevPin)));
}

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

void EEPROM_WriteWord(uint16_t address, int16_t value)  {
  EEPROM.write(address, value >> 8);
  EEPROM.write(address++, value & 0xff);
}

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

int16_t EEPROM_ReadWord(uint16_t address)  {
  uint8_t HiByte = EEPROM.read(address);
  uint8_t LowByte = EEPROM.read(address++);
  return word(HiByte, LowByte);
}

/******************************************************************************************************************************************************/
/******************************************************************************************************************************************************/

uint8_t ArgumentStringToAxisNum(String Argument) {
  if(Argument == "x1") {
    return 0;
  }
  else if(Argument == "y1") {
  return 1;
  }
  else if(Argument == "x2") {
    return 2;
  }
  else if(Argument == "y2") {
  return 3;
  }
  else {
    return 0;
  }
}

