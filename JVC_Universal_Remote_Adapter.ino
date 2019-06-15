#include "Arduino.h"
#include "SoftwareSerial.h"

// Define JVC commands
#define POWERNOFF  0x0A
#define VOLUP      0x04
#define VOLDOWN    0x05
#define SOURCE     0x08
#define EQUALIZER  0x0D
#define MUTE       0x0E
#define NEXTTRACK  0x12
#define PREVTRACK  0x13
#define FOLDERFORW 0x14
#define FOLDERBACK 0x15
#define UNKNOWN1   0x37
#define UNKNOWN2   0x58
#define VOICECTRL  0x1A

// Define Remote Controller Inputs
#define KEY1_PIN A1     // A1
#define KEY2_PIN A2     // A2

// Connect optocoupler input through a 1k resistor to this pin
#define OUTPUTPIN 8 // D8

// On-board LED, useful for debugging
#define LEDPIN 13 // D13

// Pulse width in µs
#define PULSEWIDTH 555

// Address that the radio responds to
#define ADDRESS 0x47 // 47

//float remoteVoltages [10] = { 2.51, 4.25, 3.91, 0.46, 3.44, 3.63, 1.16, 4.05, 4.34, 2.83 };
float remoteVoltages [10] = { 0.502, 0.85, 0.782, 0.092, 0.688, 0.726, 0.232, 0.81, 0.868, 0.566 };
unsigned char mappedCommands [10] = { SOURCE, EQUALIZER, FOLDERFORW, FOLDERBACK, VOICECTRL, VOLUP, VOLDOWN, NEXTTRACK, PREVTRACK, MUTE };

unsigned char lastPushed = 0;
unsigned long currentTime, lastPushedTime;
float baseVoltageOne, baseVoltageTwo = 0; // This will be the voltage if there is no button pressed

float ConvertVoltage(int analogValue) {
  return analogValue * (5.0 / 1023.0);
}

bool AroundValue(float expectedVoltage, float voltage) {
  float threshold = 0.1;
  return voltage >= expectedVoltage - threshold && voltage <= expectedVoltage + threshold;
}

unsigned char GetInput(void) {
  currentTime = millis();
  
  float keyOneVoltage = ConvertVoltage(analogRead(KEY1_PIN));
  float keyTwoVoltage = ConvertVoltage(analogRead(KEY2_PIN));

  float voltageThreshold = 4.95;

  // Keep track of the base voltage in case it changed since bootup
  if ( keyOneVoltage > 4.45 ) { baseVoltageOne = keyOneVoltage; }
  if ( keyTwoVoltage > 4.45 ) { baseVoltageTwo = keyTwoVoltage; }

  unsigned char currentButton = 0;
  
  // Only do something if any of the buttons is pushed
  if (keyOneVoltage < baseVoltageOne || keyTwoVoltage < baseVoltageTwo) {
    float currentVoltage = keyOneVoltage;
    float baseVoltage = baseVoltageOne;

    Serial.println(keyOneVoltage);
    Serial.println(keyTwoVoltage);

    // Iterate through known voltage drops
    int i = 0;
    for ( i=0 ; i<10 ; ++i ) {
      // From 5 to 10, we need the voltage from the second pin
      if (i >= 5) { 
        currentVoltage = keyTwoVoltage; 
        baseVoltage = baseVoltageTwo;
      }

      // If the voltage drop is within the threshold
      if ( AroundValue(remoteVoltages[i] * baseVoltage, currentVoltage) ) {
        // Send the command if the pushed button is not last pushed one
        // or if it is the last one but it is at least 300ms later 
        if ( lastPushed != mappedCommands[i] || (lastPushed == mappedCommands[i] && currentTime >= lastPushedTime + 300) ) {
          Serial.println(mappedCommands[i]);
          currentButton = mappedCommands[i];

          // Set this value as the last pushed button
          lastPushed = currentButton;
          // And the current time as the last time the button was pushed
          lastPushedTime = currentTime;
          break;
        }
      }
    }

    return currentButton;
  }
  
  return 0;
}

// Signals to transmit a '0' bit
void SendZero() {
  digitalWrite(OUTPUTPIN, HIGH); // Output HIGH for 1 pulse width
  digitalWrite(LEDPIN, HIGH);    // Turn on on-board LED
  delayMicroseconds(PULSEWIDTH);
  digitalWrite(OUTPUTPIN, LOW); // Output LOW for 1 pulse width
  digitalWrite(LEDPIN, LOW);    // Turn off on-board LED
  delayMicroseconds(PULSEWIDTH);
}

// Signals to transmit a '1' bit
void SendOne() {
  digitalWrite(OUTPUTPIN, HIGH); // Output HIGH for 1 pulse width
  digitalWrite(LEDPIN, HIGH);    // Turn on on-board LED
  delayMicroseconds(PULSEWIDTH);
  digitalWrite(OUTPUTPIN, LOW); // Output LOW for 3 pulse widths
  digitalWrite(LEDPIN, LOW);    // Turn off on-board LED
  delayMicroseconds(PULSEWIDTH * 3);
}

// Signals to precede a command to the radio
void Preamble() {
  // HEADER: always LOW (1 pulse width), HIGH (16 pulse widths), LOW (8 pulse widths)
  digitalWrite(OUTPUTPIN, LOW); // Make sure output is LOW for 1 pulse width, so the header starts with a rising edge
  digitalWrite(LEDPIN, LOW);    // Turn off on-board LED
  delayMicroseconds(PULSEWIDTH * 1);
  digitalWrite(OUTPUTPIN, HIGH); // Start of header, output HIGH for 16 pulse widths
  digitalWrite(LEDPIN, HIGH);    // Turn on on-board LED
  delayMicroseconds(PULSEWIDTH * 16);
  digitalWrite(OUTPUTPIN, LOW); // Second part of header, output LOW 8 pulse widths
  digitalWrite(LEDPIN, LOW);    // Turn off on-board LED
  delayMicroseconds(PULSEWIDTH * 8);

  // START BIT: always 1
  SendOne();
}

// Signals to follow a command to the radio
void Postamble() {
  // STOP BITS: always 1
  SendOne();
  SendOne();
}

// Send a value (7 bits, LSB is sent first, value can be an address or command)
void SendValue(unsigned char value) {
  unsigned char i, tmp = 1;
  for (i = 0; i < sizeof(value) * 8 - 1; i++) {
    if (value & tmp) // Do a bitwise AND on the value and tmp
      SendOne();
    else
      SendZero();

   tmp = tmp << 1; // Bitshift left by 1
  }
}

// Send a command to the radio, including the header, start bit, address and stop bits
void SendCommand(unsigned char value) {
  unsigned char i;
  Preamble();
  for (i = 0; i < 3; i++) {          // Repeat address, command and stop bits three times so radio will pick them up properly
    SendValue(ADDRESS);              // Send the address
    SendValue((unsigned char)value); // Send the command
    Postamble();                     // Send signals to follow a command to the radio
  }
}

// Runs once on startup
void setup() {
  Serial.begin(9600);
  currentTime = millis();
  
  pinMode(OUTPUTPIN, OUTPUT);   // Set the proper pin as output
  digitalWrite(OUTPUTPIN, LOW); // Output LOW to make sure optocoupler is off

  pinMode(LEDPIN, OUTPUT); // Set pin connected to on-board LED as output...
  digitalWrite(LEDPIN, LOW);

  digitalWrite(LEDPIN, HIGH);
  delay(1000);
  digitalWrite(LEDPIN, LOW);
  // ...and turn LED off
}

// Runs repeatedly
void loop() {
  unsigned char Key = GetInput();
  if (Key) {
    SendCommand(Key);
    delay(2);
    SendCommand(Key);
    delay(20);
  }
}
