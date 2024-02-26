#include <Arduino.h>

//Including libraries for display
#include <U8g2lib.h>
#include <Wire.h>

//Defining display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Define whether to also write output on UART to usb
#define DEBUG_OUTPUT_WRITE false

//Defining analog input pins
const int batteryVoltagePin = A1; 
const int yJoystickPin = A3;
const int xJoystickPin = A2;
// A4 and A5 are used for I2C to display
const int camPotPin = A6;
const int speedPotPin = A7;

//Defining digital input pins
const int potSwitchPin = 4;
const int ledSwitchPin = 7;
const int stabilizeSwitchPin = 8;

//Defining digital output pins
const int waterLedPin = 2;

//Variables for controller
bool ledState = false;
bool stabilizeState = false;
signed int speed = 0;
int camAngle = 0;
double batteryVoltage = 0;
int xJoystick = 0;
int yJoystick = 0;

//Variables for sending
unsigned long long toSend;

void writeData(long long data){
  uint8_t *pointer = (uint8_t *)&data;
  //Length of data to send
  int i = 42;
  int mask = 0b1111111;
  int toSend = 0x00;
  int stopBit = 0b1;
  bool finalByte = false;
  while (true){
    //Get the last 7 bits
    toSend = data & mask;
    //Shift 7 bits
    data >>= 7;
    i -= 7;
    //Shift toSend 1 position to create space for stopbit
    toSend <<= 1;
    //append stop bit if needed
    if (i <= 0){
      toSend = toSend | stopBit;
      finalByte = true;
    }
    if (DEBUG_OUTPUT_WRITE){
      Serial.write(toSend);
    }
    Serial1.write(toSend);
    if (finalByte){
      break;
    }
  }
}

void setup() {
  //Set pin modes
  pinMode(waterLedPin, OUTPUT);
  pinMode(potSwitchPin, INPUT_PULLUP);
  pinMode(ledSwitchPin, INPUT_PULLUP);
  pinMode(stabilizeSwitchPin, INPUT_PULLUP);

  //Init display
  u8g2.begin();

  //Initialize serial logging
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  //Create variable to send message
  toSend = 0x000uLL;
  
  //Read the pins
  ledState = (digitalRead(ledSwitchPin) == HIGH) ? false : true;

  stabilizeState = (digitalRead(stabilizeSwitchPin) == HIGH) ? false : true;

  speed = -1*(analogReadMultiple(speedPotPin) - 512);
  //Correct to zero, because otherwise it is impossible to stop motor
  speed = (speed > -20 && speed < 20) ? 0 : speed;
  //Correct one unit, because max of 9 bits is 511, not 512. Only occurs at positive
  speed = (speed > 511) ? 511 : speed;

  camAngle = 1023 - analogReadMultiple(camPotPin);

  batteryVoltage = (double)analogReadMultiple(batteryVoltagePin) * 5 / 1024;

  xJoystick = -1*(analogReadMultiple(xJoystickPin) - 512);
  xJoystick = (xJoystick > -30 && xJoystick < 30) ? 0 : xJoystick;
  //Correct one unit, because max of 9 bits is 511, not 512. Only occurs at positive
  xJoystick = (xJoystick > 511) ? 511 : xJoystick;

  yJoystick = -1*(analogReadMultiple(yJoystickPin) - 512);
  yJoystick = (yJoystick > -30 && yJoystick < 30) ? 0 : yJoystick;
  //Correct one unit, because max of 9 bits is 511, not 512. Only occurs at positive
  yJoystick = (yJoystick > 511) ? 511 : yJoystick;
  
  //Calculating message to send via serial1
  toSend += ledState;
  toSend <<= 1;
  toSend += stabilizeState;
  toSend <<= 1;
  toSend += (speed < 0);
  toSend <<= 9;
  toSend += abs(speed);
  toSend <<= 10;
  toSend += camAngle;
  toSend <<= 1;
  toSend += (xJoystick < 0);
  toSend <<= 9;
  toSend += abs(xJoystick);
  toSend <<= 1;
  toSend += (yJoystick < 0);
  toSend <<= 9;
  toSend += abs (yJoystick);

  //Sending message
  writeData(toSend);

  u8g2.clearBuffer();
  u8g2_draw();
  u8g2.sendBuffer();
}

//Read multiple times for a better voltage
int analogReadMultiple(int pin){
  int total = 0;
  int n = 0;
  //31 is the max times 1024 (max value of ADC) fits in an int
  while (n<31){
    total += analogRead(pin);
    n++;
    
  }
  return round((float)total/n);
}

//Prepare display for drawing
void u8g2_prepare() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}
void u8g2_draw(){
  //Setting strings to draw
  char speedString[12];
  char angleString[11];
  char batteryVoltageString[5];
  char batteryVoltageStringFormatted[11];
  char xJoystickString[11];
  char yJoystickString[11];

  //Formatting strings
  sprintf(speedString, "SPEED: %d", speed);
  sprintf(angleString, "CAM: %d", camAngle);
  dtostrf(batteryVoltage, 3, 2, batteryVoltageString);
  sprintf(batteryVoltageStringFormatted, "U: %sV", batteryVoltageString);
  sprintf(xJoystickString, "X: %d", xJoystick);
  sprintf(yJoystickString, "Y: %d", yJoystick);

  //Prepare for drawing
  u8g2_prepare();

  //Drawing
  u8g2.drawStr(0, 0, (ledState)?"LED: ON":"LED: OFF");
  u8g2.drawStr(0, 10, (stabilizeState)?"STAB: ON":"STAB: OFF");
  u8g2.drawStr(0, 20, speedString);
  u8g2.drawStr(0, 30, angleString);
  u8g2.drawStr(0, 40, batteryVoltageStringFormatted);
  u8g2.drawStr(75, 0, xJoystickString);
  u8g2.drawStr(75, 10, yJoystickString);
}