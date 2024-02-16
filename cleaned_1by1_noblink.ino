#include <Arduino.h>
#include <FastLED.h>
#include <elapsedMillis.h>
#include <Servo.h>

// Pin Definitions
#define LED_PIN 11
#define NUM_LEDS 15
#define IR_PIN 13
#define DIR_PIN 45
#define PULL_PIN 46
#define SERVO_PWM 44
#define LED_PIN_CUBE 12
#define NUM_LEDS_CUBE 32
#define DRIVER_OUT1 82
#define DRIVER_OUT2 83

// Global Variables
CRGB leds[NUM_LEDS];
CRGB ledscube[NUM_LEDS_CUBE];
elapsedMillis elapsedTime;
unsigned long sensorBlockedStartTime = 0;
unsigned long defaultSpeed = 130;
unsigned long lastToggleTime = 0;
int8_t startIndex = 0;
uint8_t hue = 0;
bool moveRight = true;
bool ledStripOn = false;

const int motorPins[] = {54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83};
const int numPumps = sizeof(motorPins) / sizeof(motorPins[0]);
bool isReverse = true;
unsigned long pumpStartTimes[numPumps] = {0};
unsigned long pumpRunTimes[numPumps] = {0};
unsigned long lastPumpEndTime = 0;
int maxRunTime = 0;
String currentMotorType;

enum MotorState { IDLE, RUNNING_PUMPS };
MotorState motorState = IDLE;
Servo myServo;

// Function Prototypes
void runPumps(int pumpNumber, int runTime);
void roundStrip(int speed, int runTime);
void movingRainbowEffect(unsigned long speed, int ledStripState);
void ledStrip(int speed, int runTime);
void blinkLEDs();
void runServo(int angle, int runSpeed);
void runStepper(int angle, int speed);
void processCommand(String command);

// Setup Function
void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, LED_PIN_CUBE, GRB>(ledscube, NUM_LEDS_CUBE);

  for (int i = 0; i < numPumps; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PULL_PIN, OUTPUT);
  pinMode(SERVO_PWM, OUTPUT);
  pinMode(DRIVER_OUT1, OUTPUT);
  pinMode(DRIVER_OUT2, OUTPUT);
  digitalWrite(DRIVER_OUT1, LOW);
  digitalWrite(DRIVER_OUT2, LOW);
  
  Serial.begin(115200);
}

// Main Loop
void loop() {
  if (digitalRead(IR_PIN) == LOW) {
    if (sensorBlockedStartTime == 0) {
      sensorBlockedStartTime = millis();
    } else if (millis() - sensorBlockedStartTime >= 2000) {
      movingRainbowEffect(defaultSpeed, ledStripOn ? 0 : 1);
      sensorBlockedStartTime = 0;
    }
  } else {
    sensorBlockedStartTime = 0;
  }

  if (motorState == IDLE) {
    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');
      Serial.println(data);
      processCommand(data);
    }
  } else if (motorState == RUNNING_PUMPS) {
    unsigned long currentTime = millis();
    for (int i = 0; i < numPumps; i++) {
      if (currentTime >= pumpStartTimes[i] + pumpRunTimes[i] && digitalRead(motorPins[i]) == HIGH) {
        digitalWrite(motorPins[i], LOW);
        break;
      }
    }

    bool allPumpsOff = true;
    for (int i = 0; i < numPumps; i++) {
      if (digitalRead(motorPins[i]) == HIGH) {
        allPumpsOff = false;
        break;
      }
    }

    if (allPumpsOff) {
      motorState = IDLE;
      digitalWrite(isReverse ? DRIVER_OUT2 : DRIVER_OUT1, LOW);
      delay(2000);
      runStepper(5000, 2000);
    }
  }
}

// Function to Run Pumps
void runPumps(int pumpNumber, int runTime) {
  digitalWrite(isReverse ? DRIVER_OUT2 : DRIVER_OUT1, HIGH);
  digitalWrite(isReverse ? DRIVER_OUT1 : DRIVER_OUT2, LOW);
  digitalWrite(motorPins[pumpNumber - 54], HIGH);
  pumpStartTimes[pumpNumber - 54] = millis();
  pumpRunTimes[pumpNumber - 54] = runTime;
  lastPumpEndTime = millis();
  for (int i = 0; i < numPumps; i++) {
    if (pumpStartTimes[i] + pumpRunTimes[i] > lastPumpEndTime) {
      lastPumpEndTime = pumpStartTimes[i] + pumpRunTimes[i];
    }
  }
  Serial.print("Running pump: ");
  Serial.println(pumpNumber);
}

// Function to Run Stepper Motor
void runStepper(int angle, int speed) {
  const float STEP_ANGLE = 1.8; 
  int direction = (angle >= 0) ? HIGH : LOW;
  angle = abs(angle);

  digitalWrite(DIR_PIN, direction);
  int stepTargetPosition = int(2 * (angle / STEP_ANGLE));

  int i = 0;
  while (i < stepTargetPosition && digitalRead(IR_PIN) == HIGH) {
    digitalWrite(PULL_PIN, HIGH);
    delayMicroseconds(speed);
    digitalWrite(PULL_PIN, LOW);
    delayMicroseconds(speed);
    i++;
  }

  digitalWrite(PULL_PIN, LOW);
  blinkLEDs();

  if (digitalRead(IR_PIN) == LOW || i < stepTargetPosition) {
    blinkLEDs();
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
  }
}

// Function to Process Serial Commands
void processCommand(String command) {
  char separator = ',';
  int firstBracketIndex = command.indexOf('(');
  int secondBracketIndex = command.indexOf(')', firstBracketIndex + 1);

  if (firstBracketIndex != -1 && secondBracketIndex != -1) {
    String motorTypeAndSID = command.substring(firstBracketIndex + 1, secondBracketIndex);
    int spaceIndex = motorTypeAndSID.indexOf(' ');

    if (spaceIndex != -1) {
      String motorType = motorTypeAndSID.substring(0, spaceIndex);
      String SID = motorTypeAndSID.substring(spaceIndex + 1);
      currentMotorType = motorType;
      int index = secondBracketIndex + 1;

      while (index < command.length()) {
        int nextBracketIndex = command.indexOf('(', index);
        int endIndex = command.indexOf(')', nextBracketIndex + 1);

        if (nextBracketIndex != -1 && endIndex != -1) {
          String inputData = command.substring(nextBracketIndex + 1, endIndex);
          int commaIndex = inputData.indexOf(separator);

          if (commaIndex != -1) {
            int param1 = inputData.substring(0, commaIndex).toInt();
            int param2 = inputData.substring(commaIndex + 1).toInt();
            Serial.print(currentMotorType);
            Serial.print(": Param1: ");
            Serial.print(param1);
            Serial.print(", Param2: ");
            Serial.println(param2);

            if (currentMotorType == "PUMPMOTOR_OPERATION" || currentMotorType == "REVERSE_PUMPMOTOR_OPERATION") {
              maxRunTime = max(maxRunTime, param2);
              Serial.println("maxRunTime: " + String(maxRunTime));
              ledStrip(defaultSpeed, maxRunTime);
              runPumps(param1, param2);
              motorState = RUNNING_PUMPS;
            } else if (currentMotorType == "SERVOMOTOR_OPERATION") {
              runServo(param1, param2);
            } else if (currentMotorType == "STEPPERMOTOR_OPERATION") {
              runStepper(param1, param2);
            } else if (currentMotorType == "LED_STRIP_OPERATION") {
              ledStrip(param1, param2);
            } else {
              Serial.println("Unknown motor type");
            }
          } else {
            Serial.println("Invalid pump data format");
          }

          index = endIndex + 1;
        } else {
          break;
        }
      }
    } else {
      Serial.println("Invalid motor type and SID format");
    }
  } else {
    Serial.println("Invalid command format");
  }
}


// Function to Blink LEDs
void blinkLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::White;
  }
  FastLED.show();
  delay(100);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  delay(100);
}

// Function to Implement Moving Rainbow Effect
void movingRainbowEffect(unsigned long speed, int ledStripState) {
  if (elapsedTime > speed) {
    if (ledStripState == 1) {
      startIndex++;
      fill_rainbow(leds, NUM_LEDS, startIndex, 7);
      FastLED.show();
    }
    elapsedTime = 0;
  }
}

// Function to Implement LED Strip
void ledStrip(int speed, int runTime) {
  if (!ledStripOn) {
    ledStripOn = true;
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
    delay(2000);
  }

  unsigned long startTime = millis();
  while (millis() - startTime < runTime) {
    movingRainbowEffect(speed, 1);
  }

  ledStripOn = false;
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

// Function to Run Servo
void runServo(int angle, int runSpeed) {
  myServo.attach(SERVO_PWM);
  myServo.write(angle);
  delay(runSpeed);
  myServo.detach();
}


//"(REVERSE_PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,1000),(55,3500),(56,2600),(57,1000),(58,2500),(59,4000),(59,1000),(60,5500),(61,500),(62,3600),(64,1000),(65,2500),(66,4000),(67,1000),(68,5500),(69,5000),(70,3600),(71,2000),(75,2500),(80,4000),(83,1000),(78,5500)"
//"(PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,1000),(55,3500),(56,2600),(57,1000),(58,2500),(59,4000),(59,1000),(60,5500),(61,500),(62,3600),(64,1000),(65,2500),(66,4000),(67,1000),(68,5500),(69,5000),(70,3600),(71,2000),(75,2500),(80,4000),(83,1000),(78,5500)"
