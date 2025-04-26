
//////////////////////////////////////////////////
//  LIBRARIES  //
//////////////////////////////////////////////////

#include <Arduino.h>
#include <IRremote.hpp>
#include <SparkFun_TB6612.h>

#define DECODE_NEC  // Defines the type of IR transmission to decode based on the remote

//////////////////////////////////////////////////
//  REMOTE CONTROL CODES  //
//////////////////////////////////////////////////
// Define remote control button codes for up and down
#define up 0x18
#define down 0x52

//////////////////////////////////////////////////
//  MOTOR DRIVER PINS & SETTINGS  //
//////////////////////////////////////////////////
// Motor driver configuration

#define BIN1              7
#define BIN2              6
#define PWMB              5
#define STBY              8
#define OFFSET_B          1
#define IR_RECEIVE_PIN    2
#define WHEEL_SPEED       90  // Constant wheel speed

//////////////////////////////////////////////////
//  GLOBAL VARIABLES  //
//////////////////////////////////////////////////
// Drive motors
Motor wheel = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);  // We'll only use one motor (wheel)
bool wheelDirection = true;  // true = forward, false = backward
uint8_t lastCommand = 0;

//////////////////////////////////////////////////
//  SETUP  //
//////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  
  // Initialize motor driver
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR Receiver init");
  // Output debug info
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing IR library version " VERSION_IRREMOTE));
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin "));
  Serial.println(String(IR_RECEIVE_PIN));
  Serial.println(F("Simplified IR Wheel Control initialized"));
  Serial.println(F("UP: Move wheel forward, DOWN: Move wheel backward"));
  
  // Initial state - motor stopped
  //wheel.brake();
}

//////////////////////////////////////////////////
//  MAIN LOOP  //
//////////////////////////////////////////////////

void loop() {
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
      // Repeat code detected
      Serial.println(F("REPEAT detected"));
      // Use last known command
    } else {
      // New command received
      lastCommand = IrReceiver.decodedIRData.command;
      IrReceiver.printIRResultShort(&Serial);
    }

    // Handle command (lastCommand is set or reused)
    switch (lastCommand) {
      case up:
        wheelDirection = true;
        wheel.drive(WHEEL_SPEED);
        Serial.println(F("WHEEL FORWARD"));
        delay(2000);
        wheel.brake();
        break;
      case down:
        wheelDirection = false;
        wheel.drive(-WHEEL_SPEED);
        Serial.println(F("WHEEL BACKWARD"));
        delay(2000);
        wheel.brake();
        break;
      default:
        Serial.println(F("Unknown command"));
        break;
    }
    Serial.println("Resuming IR receiver...");
    IrReceiver.resume();  // Ready for next input
  }

  delay(5);
}

