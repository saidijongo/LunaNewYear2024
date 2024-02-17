
#include <Arduino.h>
#include <FastLED.h>
#include <elapsedMillis.h>
#include <Servo.h>

#define LED_PIN 11
#define NUM_LEDS 32

CRGB leds[NUM_LEDS];

elapsedMillis elapsedTime; // Declare elapsedTime as a static variable

const int motorPins[] = {54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83};
const int dirPin = 45;
const int pullPin = 46;
const int servoPWM = 44;
#define IR_PIN 13

//cube led strip
#define NUM_LEDS_CUBE 32
#define LED_PIN_CUBE 12
CRGB ledscube[NUM_LEDS_CUBE];
bool ledStripOn = false;
unsigned long sensorBlockedStartTime = 0;
unsigned long defaultSpeed = 130;
unsigned long lastToggleTime = 0;
int8_t startIndex = 0; // Changed to signed type for handling direction
uint8_t hue = 0;
bool moveRight = true; // Flag to determine direction of movement


const int driverOut1 = 82;
const int driverOut2 = 83;

const int numPumps = sizeof(motorPins) / sizeof(motorPins[0]);

bool isReverse = true;

unsigned long pumpStartTimes[numPumps] = {0};
unsigned long pumpRunTimes[numPumps] = {0}; // run times store for each pump
unsigned long lastPumpEndTime = 0; // Variable to store the end time of the last pump
// Find the maximum runTime for param2 values.
int maxRunTime = 0;

String currentMotorType;
Servo myServo;

enum MotorState
{
  IDLE,
  RUNNING_PUMPS,
  RUNNING_STEPPER
};

MotorState motorState = IDLE;

void setup()
{
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, LED_PIN_CUBE, GRB>(ledscube, NUM_LEDS_CUBE);


  for (int i = 0; i < numPumps; i++)
  {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  pinMode(dirPin, OUTPUT);
  pinMode(pullPin, OUTPUT);
  pinMode(servoPWM, OUTPUT);

  pinMode(driverOut1, OUTPUT);
  pinMode(driverOut2, OUTPUT);

  digitalWrite(driverOut1, LOW); // Ensure driverOut1 is initially low
  digitalWrite(driverOut2, LOW); // Ensure driverOut2 is initially low

  Serial.begin(115200);
}

void runPumps(int pumpNumber, int runTime)
{
  digitalWrite(isReverse ? driverOut2 : driverOut1, HIGH);
  digitalWrite(isReverse ? driverOut1 : driverOut2, LOW); // Turn off the other driver output
  digitalWrite(motorPins[pumpNumber - 54], HIGH);
  pumpStartTimes[pumpNumber - 54] = millis();
  pumpRunTimes[pumpNumber - 54] = runTime; // Store individual run time for this pump

  
  // Calculate last pump end time considering all pump run times
  lastPumpEndTime = millis();
  for (int i = 0; i < numPumps; i++)
  {
    if (pumpStartTimes[i] + pumpRunTimes[i] > lastPumpEndTime)
    {
      lastPumpEndTime = pumpStartTimes[i] + pumpRunTimes[i];
    }
  }

  Serial.print("Running pump: ");
  Serial.println(pumpNumber);
}


void roundStrip(int speed, int runTime) {
  bool stripRunning = true;
  static uint8_t startIndex = 0;
  static uint8_t hue = 0;
  unsigned long startTime = millis();

  while (stripRunning && (millis() - startTime < runTime)) {
    // Fill the entire LED strip with a rainbow gradient
    fill_rainbow(leds, NUM_LEDS, hue, 4);

    // Move the rainbow effect from left to right
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = leds[(i + startIndex) % NUM_LEDS];
    }

    FastLED.show();
    FastLED.delay(speed);

    // Increment the rainbow hue to change colors
    hue++;

    // Move the rainbow gradient index
    startIndex++;
    if (startIndex >= NUM_LEDS) {
      startIndex = 0;
    }
  }

  // Turn off LEDs after the runTime
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Set all LEDs to black (off)
  FastLED.show();
  stripRunning = false; //stop iterations
}



void movingRainbowEffect(unsigned long speed, int ledStripState) {
  if (ledStripState == 1) {
    // Turn on the LED strip
    if (!ledStripOn) {
      ledStripOn = true;
    }

    // Calculate the delay based on the received speed or use the default speed
    unsigned long delayTime = (speed > 0) ? speed : defaultSpeed;

    if (millis() - lastToggleTime >= delayTime) {
      // Update the hue for the rainbow effect
      hue++;

      // Fill the LEDs with the rainbow pattern
      for (int i = 0; i < NUM_LEDS_CUBE; i++) {
        int pixelHue = hue + (i * 255 / NUM_LEDS_CUBE);
        ledscube[(startIndex + i) % NUM_LEDS_CUBE] = CHSV(pixelHue, 255, 255);
      }

      FastLED.show();
      lastToggleTime = millis();

      // Update the startIndex based on the direction of movement
      if (moveRight) {
        startIndex++; // Move right
        if (startIndex >= NUM_LEDS_CUBE) {
          startIndex = 0; // Wrap around
        }
      } else {
        startIndex--; // Move left
        if (startIndex < 0) {
          startIndex = NUM_LEDS_CUBE - 1; // Wrap around
        }
      }
    }
  } else {
    // Turn off the LED strip
    if (ledStripOn) {
      ledStripOn = false;
      FastLED.clear();
      FastLED.show();
    }
  }
}

void ledStrip(int speed, int runTime) {
  unsigned long startTime = millis();
  unsigned long currentTime;

  // Set the background color to blue
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  while (elapsedTime < runTime) {
    currentTime = millis(); // Update currentTime
    // Move the group of 5 green LEDs from left to right
    for (int i = 0; i <= NUM_LEDS - 5; ++i) {
      fill_solid(leds + i, 5, CRGB::Green); // Set a group of 5 LEDs to green
      FastLED.show();
      delay(speed);
      fill_solid(leds + i, 5, CRGB::Blue); // Set the same group back to blue
    }

    // Move the group of 5 green LEDs from right to left
    for (int i = NUM_LEDS - 5; i >= 0; --i) {
      fill_solid(leds + i, 5, CRGB::Green); // Set a group of 5 LEDs to green
      FastLED.show();
      delay(speed);
      fill_solid(leds + i, 5, CRGB::Blue); // Set the same group back to blue
    }

    // Check if runtime exceeded
    if (currentTime - startTime >= runTime) {
      break;
    }
  }

  // Turn off LEDs after the runTime
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Set all LEDs to black (off)
  FastLED.show();
}

void blinkLEDs() {
  const int blinkDuration = 20000;  // 20 seconds
  const int blinkInterval = 500;   // 500 milliseconds (0.5 seconds)

  static elapsedMillis elapsedTime; // Track elapsed time

  while (elapsedTime < blinkDuration) {
    // Turn on all LEDs
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
    delay(blinkInterval);

    // Turn off all LEDs
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(blinkInterval);
  }

  // Turn off LEDs after the specified duration
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void runServo(int angle, int runSpeed)
{
  int mappedSpeed = map(runSpeed, 0, 2000, 0, 180);

  if (angle >= 0)
  {
    for (int i = 0; i <= angle; ++i)
    {
      myServo.write(i);
      delay(mappedSpeed);
    }
  }
  else
  {
    for (int i = 0; i >= angle; --i)
    {
      myServo.write(i);
      delay(mappedSpeed);
    }
  }
}

void runStepper(int angle, int speed) {
  const float STEP_ANGLE = 1.8; 
  int direction = (angle >= 0) ? HIGH : LOW;
  angle = abs(angle);

  digitalWrite(dirPin, direction);
  int step_target_position = int(2 * (angle / STEP_ANGLE));

  int i = 0;
  while (i < step_target_position && digitalRead(IR_PIN) == HIGH) {
    digitalWrite(pullPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(pullPin, LOW);
    delayMicroseconds(speed);

    i++;
  }

  digitalWrite(pullPin, LOW);
  blinkLEDs();

  if (digitalRead(IR_PIN) == LOW || i < step_target_position) {
    blinkLEDs();
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
  }
}

void processCommand(String command)
{
  char separator = ',';

  int firstBracketIndex = command.indexOf('(');
  int secondBracketIndex = command.indexOf(')', firstBracketIndex + 1);

  if (firstBracketIndex != -1 && secondBracketIndex != -1)
  {
    String motorTypeAndSID = command.substring(firstBracketIndex + 1, secondBracketIndex);
    int spaceIndex = motorTypeAndSID.indexOf(' ');

    if (spaceIndex != -1)
    {
      String motorType = motorTypeAndSID.substring(0, spaceIndex);
      String SID = motorTypeAndSID.substring(spaceIndex + 1);

      currentMotorType = motorType;

      int index = secondBracketIndex + 1;
      while (index < command.length())
      {
        int nextBracketIndex = command.indexOf('(', index);
        int endIndex = command.indexOf(')', nextBracketIndex + 1);

        if (nextBracketIndex != -1 && endIndex != -1)
        {
          String inputData = command.substring(nextBracketIndex + 1, endIndex);

          int commaIndex = inputData.indexOf(separator);
          if (commaIndex != -1)
          {
            int param1 = inputData.substring(0, commaIndex).toInt();
            int param2 = inputData.substring(commaIndex + 1).toInt();

            Serial.print(currentMotorType);
            Serial.print(": Param1: ");
            Serial.print(param1);
            Serial.print(", Param2: ");
            Serial.println(param2);

            if (currentMotorType == "PUMPMOTOR_OPERATION" || currentMotorType == "REVERSE_PUMPMOTOR_OPERATION")
            {
              maxRunTime = max(maxRunTime, param2);
              Serial.println("maxRunTime: " + String(maxRunTime));
              
              ledStrip(defaultSpeed, maxRunTime);
              runPumps(param1, param2);
              motorState = RUNNING_PUMPS;
            }
            else if (currentMotorType == "SERVOMOTOR_OPERATION")
            {
              runServo(param1, param2);
            }
            else if (currentMotorType == "STEPPERMOTOR_OPERATION")
            {
              runStepper(param1, param2);
            }
            else if (currentMotorType == "LED_STRIP_OPERATION")
            {
              ledStrip(param1, param2);
            }
            else
            {
              Serial.println("Unknown motor type");
            }
          }
          else
          {
            Serial.println("Invalid pump data format");
          }

          index = endIndex + 1;
        }
        else
        {
          break;
        }
      }
    }
    else
    {
      Serial.println("Invalid motor type and SID format");
    }
  }
  else
  {
    Serial.println("Invalid command format");
  }
}

void loop()
{
  // Check if the IR sensor is interrupted
  if (digitalRead(IR_PIN) == LOW) {
    if (sensorBlockedStartTime == 0) {
      // Initialize the timer when the sensor is first blocked
      sensorBlockedStartTime = millis();
    } else if (millis() - sensorBlockedStartTime >= 2000) {
      // Toggle the LED strip state after 3 seconds of continuous blocking
      movingRainbowEffect(defaultSpeed, ledStripOn ? 0 : 1);
      sensorBlockedStartTime = 0;  // Reset the timer
    }
  } else {
    sensorBlockedStartTime = 0;  // Reset the timer if the sensor is not blocked
  }


  if (motorState == IDLE)
  {
    if (Serial.available() > 0)
    {
      String data = Serial.readStringUntil('\n');
      Serial.println(data);
      processCommand(data);
    }
  }
  else if (motorState == RUNNING_PUMPS)
  {
    unsigned long currentTime = millis();
    for (int i = 0; i < numPumps; i++)
    {
      // Check if the current pump has finished its run time
      if (currentTime >= pumpStartTimes[i] + pumpRunTimes[i] && digitalRead(motorPins[i]) == HIGH)
      {
        digitalWrite(motorPins[i], LOW); // Turn off the current pump
        break; // Exit the loop after turning off one pump
      }
    }

    // Check if all pumps have finished running
    bool allPumpsOff = true;
    for (int i = 0; i < numPumps; i++)
    {
      if (digitalRead(motorPins[i]) == HIGH)
      {
        allPumpsOff = false;
        break;
      }
    }

    if (allPumpsOff)
    {
      motorState = IDLE; // Set motor state to idle when all pumps are turned off
      digitalWrite(isReverse ? driverOut2 : driverOut1, LOW); // Turn off the corresponding driverOut
      delay(2000);
      // Run the stepper motor with appropriate values
      runStepper(5000,2000);
    }
  }
}




//"(STEPPERMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(-360,2000)"
////"(REVERSE_PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,2000),(55,3250),(56,3000),(60,1500),(70,1000)"
////"(PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,2000),(55,3250),(56,3000),(60,1500),(70,1000)"
//"(PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,2000),(55,3250),(56,3000),(60,1500),(70,1000)"
