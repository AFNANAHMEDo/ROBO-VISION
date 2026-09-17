#include <ESP32Servo.h>

Servo panServo;
Servo tiltServo;

#define PAN_PIN 18
#define TILT_PIN 19

// ====================================================
// SERVO CALIBRATION
// ====================================================

// Actual STOP values
#define PAN_STOP 1500
#define TILT_STOP 1500

// ====================================================
// DIRECTION-SPECIFIC SPEED
// ====================================================

// PAN
// Positive command  -> one direction
// Negative command  -> opposite direction
//
// Give each direction its own value because MG996R
// continuous servos often have different deadbands.

#define PAN_POS_STEP 70
#define PAN_NEG_STEP 55

// TILT
#define TILT_POS_STEP 70
#define TILT_NEG_STEP 55

// ====================================================
// SHORT MOVEMENT PULSE
// ====================================================

#define MOVE_PULSE_MS 55

// Safety watchdog
#define COMMAND_TIMEOUT 400

unsigned long lastCommandTime = 0;
unsigned long movementStopTime = 0;


// ====================================================
// STOP
// ====================================================

void stopServos() {

  panServo.writeMicroseconds(PAN_STOP);
  tiltServo.writeMicroseconds(TILT_STOP);
}


// ====================================================
// PAN PWM
// ====================================================

int getPanPWM(int command) {

  if (command == 0) {
    return PAN_STOP;
  }

  if (command > 0) {
    return PAN_STOP + PAN_POS_STEP;
  }

  return PAN_STOP - PAN_NEG_STEP;
}


// ====================================================
// TILT PWM
// ====================================================

int getTiltPWM(int command) {

  if (command == 0) {
    return TILT_STOP;
  }

  if (command > 0) {
    return TILT_STOP + TILT_POS_STEP;
  }

  return TILT_STOP - TILT_NEG_STEP;
}


// ====================================================
// SETUP
// ====================================================

void setup() {

  Serial.begin(115200);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  panServo.attach(
    PAN_PIN,
    500,
    2500
  );

  tiltServo.attach(
    TILT_PIN,
    500,
    2500
  );

  stopServos();

  lastCommandTime = millis();
  movementStopTime = millis();

  Serial.println(
    "ESP32 SERVO CONTROLLER READY"
  );
}


// ====================================================
// LOOP
// ====================================================

void loop() {

  // ==================================================
  // SERIAL COMMAND
  // ==================================================

  if (Serial.available()) {

    String command =
      Serial.readStringUntil('\n');

    command.trim();


    // =================================================
    // STOP
    // =================================================

    if (
      command == "S" ||
      command == "C"
    ) {

      stopServos();

      lastCommandTime = millis();
      movementStopTime = millis();

      Serial.println("STOP");
    }


    // =================================================
    // MOVEMENT
    //
    // D,pan,tilt
    //
    // D,-2,1
    // D,3,-1
    // =================================================

    else if (
      command.startsWith("D,")
    ) {

      int firstComma =
        command.indexOf(',');

      int secondComma =
        command.indexOf(
          ',',
          firstComma + 1
        );


      if (secondComma > 0) {

        int panCommand =
          command.substring(
            firstComma + 1,
            secondComma
          ).toInt();

        int tiltCommand =
          command.substring(
            secondComma + 1
          ).toInt();


        // ---------------------------------------------
        // LIMIT COMMANDS
        // ---------------------------------------------

        panCommand = constrain(
          panCommand,
          -5,
          5
        );

        tiltCommand = constrain(
          tiltCommand,
          -5,
          5
        );


        // ---------------------------------------------
        // PAN
        // ---------------------------------------------

        panServo.writeMicroseconds(
          getPanPWM(panCommand)
        );


        // ---------------------------------------------
        // TILT
        // ---------------------------------------------

        tiltServo.writeMicroseconds(
          getTiltPWM(tiltCommand)
        );


        // ---------------------------------------------
        // SHORT PULSE
        // ---------------------------------------------

        movementStopTime =
          millis() + MOVE_PULSE_MS;


        // ---------------------------------------------
        // WATCHDOG
        // ---------------------------------------------

        lastCommandTime = millis();


        // ---------------------------------------------
        // DEBUG
        // ---------------------------------------------

        Serial.print("D: ");
        Serial.print(panCommand);
        Serial.print(",");
        Serial.println(tiltCommand);
      }
    }
  }


  // ==================================================
  // STOP AFTER MOVEMENT PULSE
  // ==================================================

  if (
    millis() >= movementStopTime
  ) {

    stopServos();
  }


  // ==================================================
  // SAFETY WATCHDOG
  // ==================================================

  if (
    millis() - lastCommandTime
    > COMMAND_TIMEOUT
  ) {

    stopServos();

    movementStopTime = millis();
  }
}