
#include <Arduino.h>
#include <FastLED.h>
#include <elapsedMillis.h>
#include <Servo.h>

#define LED_PIN 11
#define NUM_LEDS 15
#define IR_PIN 13

CRGB leds[NUM_LEDS];
elapsedMillis elapsedTime; 

const int motorPins[] = {54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83};
const int dirPin = 45;
const int pullPin = 46;
const int servoPWM = 44;
const int driverOut1 = 84;
const int driverOut2 = 85;

#define MOTOR_ON HIGH
#define MOTOR_OFF LOW

const int numPumps = sizeof(motorPins) / sizeof(motorPins[0]);

bool isReverse = true;
int maxRunTime = 0;
unsigned long pumpStartTimes[numPumps] = {0};
unsigned long lastPumpEndTime = 0;
int runningPumpsCount = 0;
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

  Serial.begin(115200);
}

void roundStrip(int speed, int runTime)
{
  bool stripRunning = true;
  static uint8_t startIndex = 0;
  static uint8_t hue = 0;
  unsigned long startTime = millis();

  while (stripRunning && (millis() - startTime < runTime))
  {
    fill_rainbow(leds, NUM_LEDS, hue, 4);

    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = leds[(i + startIndex) % NUM_LEDS];
    }

    FastLED.show();
    FastLED.delay(speed);

    hue++;
    startIndex++;
    if (startIndex >= NUM_LEDS)
    {
      startIndex = 0;
    }
  }

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void ledStrip(int speed, int runTime)
{
  unsigned long startTime = millis();

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  while (elapsedTime < runTime)
  {
    for (int i = 0; i <= NUM_LEDS - 5; ++i)
    {
      fill_solid(leds + i, 5, CRGB::Green);
      FastLED.show();
      delay(speed);
      fill_solid(leds + i, 5, CRGB::Black);
    }

    for (int i = NUM_LEDS - 5; i >= 0; --i)
    {
      fill_solid(leds + i, 5, CRGB::Green);
      FastLED.show();
      delay(speed);
      fill_solid(leds + i, 5, CRGB::Black);
    }

    if (millis() - startTime >= runTime)
    {
      break;
    }
  }

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void blinkLEDs()
{
  const int blinkDuration = 20000; 
  const int blinkInterval = 500;

  static elapsedMillis elapsedTime;

  while (elapsedTime < blinkDuration)
  {
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
    delay(blinkInterval);

    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(blinkInterval);
  }

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void runPumps(int pumpNumber, int runTime)
{
  digitalWrite(isReverse ? driverOut2 : driverOut1, HIGH);
  digitalWrite(motorPins[pumpNumber - 54], HIGH);
  pumpStartTimes[pumpNumber - 54] = millis();
  runningPumpsCount++;

  if (millis() + runTime > lastPumpEndTime)
  {
    lastPumpEndTime = millis() + runTime;
  }

  Serial.print("Running pump: ");
  Serial.println(pumpNumber);
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

void runStepper(int angle, int speed)
{
  const float STEP_ANGLE = 1.8;
  int direction = (angle >= 0) ? HIGH : LOW;
  angle = abs(angle);

  digitalWrite(dirPin, direction);
  int step_target_position = int(2 * (angle / STEP_ANGLE));

  int i = 0;
  while (i < step_target_position && digitalRead(IR_PIN) == HIGH)
  {
    digitalWrite(pullPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(pullPin, LOW);
    delayMicroseconds(speed);

    i++;
  }

  digitalWrite(pullPin, LOW);

  if (digitalRead(IR_PIN) == LOW || i < step_target_position)
  {
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
              runPumps(param1, param2);
              maxRunTime = max(maxRunTime, param2);
              motorState = RUNNING_PUMPS;
              Serial.println("maxRunTime: " + String(maxRunTime));
            }
            else if (currentMotorType == "SERVOMOTOR_OPERATION")
            {
              runServo(param1, param2);
            }
            else if (currentMotorType == "STEPPERMOTOR_OPERATION")
            {
              runStepper(param1, param2);
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

  // Check and deactivate pumps if the run time has elapsed
  if (millis() >= lastPumpEndTime)
  {
    if (currentMotorType == "PUMPMOTOR_OPERATION" && !isReverse)
    {
      digitalWrite(driverOut1, MOTOR_OFF);
    }
    else if (currentMotorType == "REVERSE_PUMPMOTOR_OPERATION" && isReverse)
    {
      digitalWrite(driverOut2, MOTOR_OFF);
    }

    motorState = IDLE;
    runningPumpsCount = 0;
  }
  else if (runningPumpsCount == 0)
  {
    if (currentMotorType == "PUMPMOTOR_OPERATION" && !isReverse)
    {
      digitalWrite(driverOut1, MOTOR_ON);
    }
    else if (currentMotorType == "REVERSE_PUMPMOTOR_OPERATION" && isReverse)
    {
      digitalWrite(driverOut2, MOTOR_ON);
    }
  }
}

void loop()
{
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
    for (int i = 0; i < numPumps; i++)
    {
      if (millis() >= pumpStartTimes[i] + lastPumpEndTime && digitalRead(motorPins[i]) == HIGH)
      {
        digitalWrite(motorPins[i], MOTOR_OFF);
        runningPumpsCount--;
      }
    }
  }
  else if (motorState == RUNNING_STEPPER)
  {
    runStepper(-5000, 2000);
    motorState = IDLE;
  }
}

////"(PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,2000),(55,3250),(56,3000),(60,1500),(70,1000)"
//"(PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,2000),(55,3250),(56,3000),(60,1500),(70,1000)"
