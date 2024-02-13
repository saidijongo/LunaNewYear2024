#include <Arduino.h>
#include <FastLED.h>
#include <elapsedMillis.h>
#include <Servo.h>

#define LED_PIN 11
#define NUM_LEDS 15

CRGB leds[NUM_LEDS];

elapsedMillis elapsedTime; // Declare elapsedTime as a static variable

const int motorPins[] = {54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83};
const int dirPin = 45;
const int pullPin = 46;
const int servoPWM = 44;
#define IR_PIN 13

const int driverOut1 = 84;
const int driverOut2 = 85;

const int numPumps = sizeof(motorPins) / sizeof(motorPins[0]);

bool isReverse = true;

unsigned long pumpStartTimes[numPumps] = {0};
unsigned long pumpRunTimes[numPumps] = {0}; // Store individual run times for each pump
unsigned long lastPumpEndTime = 0; // Variable to store the end time of the last pump

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

  digitalWrite(driverOut1, LOW); // Ensure driverOut1 is initially low
  digitalWrite(driverOut2, LOW); // Ensure driverOut2 is initially low

  Serial.begin(115200);
}

void runPumps(int pumpNumber, int runTime)
{
  digitalWrite(isReverse ? driverOut2 : driverOut1, HIGH);
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
              motorState = RUNNING_PUMPS;
            }
            else if (currentMotorType == "SERVOMOTOR_OPERATION")
            {
              // 
            }
            else if (currentMotorType == "STEPPERMOTOR_OPERATION")
            {
              //
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

    // If all pumps are off, update motor state to IDLE
    if (allPumpsOff)
    {
      motorState = IDLE;
      digitalWrite(isReverse ? driverOut2 : driverOut1, LOW); // Turn off driverOut1 or driverOut2
    }
  }
}



////"(PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,2000),(55,3250),(56,3000),(60,1500),(70,1000)"
//"(PUMPMOTOR_OPERATION 1647eba3-a6b0-42a7-8a08-ffef8ab07065),(54,2000),(55,3250),(56,3000),(60,1500),(70,1000)"
