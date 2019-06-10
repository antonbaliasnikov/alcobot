/**
 * Main program for Alcobot control
 * 
 * It includes following parts:
 *  - Servo control
 *  - Max72xx display digits
 *  - Light distance sensor VL6180
 *  - Algorithms to identify glass positions
 *  
 * 
 * Some libraries hints used in program:
 * 
 * _BV means 1 << BIT => set BIT in register
 * 
 * ***Using Arduino timers***
 * TCCR => Timer/Counter Control Register (there could be 2 registers A and B)
 * 
 * WGM2 - Waveform Generation Mode bits for Timer 2
 * 011 to choose Fast PWM Mode: _BV(WGM21) | _BV(WGM20
 * 
 * 111 to use OCRA as controlling the top limit to get necessary frequency.
 * 
 * Compare Match Output A and B (COM2A and COM2B)
 * enable/disable/invert outputs A or B
 * _BV(COM2A1) | _BV(COM2B1) => non-inverted PWM outputs for A and B
 * 
 * OCR2A => "A" Output Compare Register for Timer 2, used as top value for the timer
 * OCR2B => "B" Output Compare Register for Timer 2
 * 
 * OCR2A's mode is set to "Toggle on Compare Match" by setting the COM2A bits to 01.
 * 
 * More information could be found here:
 * https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
 * 
 */

/*
   Globals section
*/

#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>

/**
 * Pinout section
 */
const int FAST_PWM_SERVO_PIN = 9;  // Fast PWM Timer1 PIN to control servo
const int ANALOG_VOLTAGE_PIN = A0; // Pin to measure analog voltage
const int PUMP_PIN = 3;            // Pump control pin
const int SOLENOID_PIN = 5;        // Solenoid control pin
const int MAX72_PANEL_CS_PIN = 10; // Chip Select pin for diode panel

/**
 * Serial host communication
 */
const int SERIAL_BAUD_RATE = 115200;

/**
 * Light distance sensor
 */
Adafruit_VL6180X vl = Adafruit_VL6180X();

/* Measurements in micro seconds for real servo */
const int MIN_ANGLE_DUTY = 1900;
const int MAX_ANGLE_DUTY = 4240;
const int SERVO_PERIOD = 6080;
/* Servo MIN and MAX turn angles */
const int MAX_ANGLE = 180;
const int MIN_ANGLE = 0;
/* Number of pulses to check servo is finished its turn */
const int SERVO_CONSTANT_VOLTAGE_PULSES = 15;
/* Current angle of servo */
int CURRENT_ANGLE = 0;
/* Pulse duty step */
int DUTY_STEP = (MAX_ANGLE_DUTY - MIN_ANGLE_DUTY) / (MAX_ANGLE - MIN_ANGLE);

/** 
 *  Liquid flow sensor
 * 
 * liquidVolume: volume of liquid passed through sensor in ml
 */
volatile double liquidVolume;

/**
 * Display setup
 * 
 * horizontalDisplaysNum: number of horizonally placed displays
 * verticalDisplaysNum: number of vertically placed displays
 * diodeIntensity: intensity of Max72xx panel, range: [0...15]
 * matrix: Max72xx diode panel object
 */
const uint8_t horizontalDisplaysNum = 1;
const uint8_t verticalDisplaysNum = 4;
const uint8_t diodeIntensity = 0;
Max72xxPanel matrix = Max72xxPanel(MAX72_PANEL_CS_PIN,
                                   horizontalDisplaysNum,
                                   verticalDisplaysNum);

/**
 * Search algorithms variables
 */
const int MAX_GLASSES = 10;
static int glassArray[MAX_GLASSES];
static int glassArrayInd = 0;
static bool pumpIsRunning = false;
const int horizontalEdgeThreshold = 150;
int range = 0;
int startCupAngle = 0;
bool glassStartFound = false;
bool glassBottomFound = false;
bool glassEndFound = false;
int glassAngle = 0;
int verticalEdgeThreshold = 20;
int glassStartRange;

/**
 *  Setup function
 */
void setup()
{
  /**
   * Start serial communication with host 
   */
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(FAST_PWM_SERVO_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(ANALOG_VOLTAGE_PIN, INPUT);

  /*  Set-up Timer1 control registers to control servo
      Enabling Fast PWM mode (mode 14) for Timer 1
      PWM period is 16 MHz / prescaler / ICR1
      Duty = OCR1A / ICR1
      Set pre-scaler to 8
      1 tick is 0.5 microS
      3040 microS = 6080 ticks, setting OCR1 to 6080
      Range in microS for servo: 950 to 2120
      middle position is 1535
  */
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS11);
  ICR1 = SERVO_PERIOD;                        // Period
  OCR1A = angleToPulse(MIN_ANGLE); // Switching value

  if (!vl.begin())
  {
    Serial.println("Failed to find VL6180x sensor");
    while (1)
      ;
  }

  /**
   * Attach external interrupt from liquid flow sensor
   */
  attachInterrupt(0, liquidSensorISR, RISING); // DIGITAL Pin 2: Interrupt 0

  /**
   * Diode matrix set-up
   * 
   * Set intensity and rotation.
   * Rotation can be:
   *   0: no rotation
   *   1: 90 degrees clockwise
   *   2: 180 degrees
   *   3: 90 degrees counter clockwise
   * 
   *  For our display we need option #3.
   */
  matrix.setIntensity(diodeIntensity);
  matrix.setRotation(3);
}

/**
 * Displays a number from 0 to 99999 on the diode matrix
 * 
 * number: integer number to be displayed
 */
void displayNumber(int number)
{
  matrix.fillScreen(LOW);
  char displayStr[] = "     ";
  String numberString = String(number);
  if (numberString.length() > 5)
  {
    displayStr[0] = 'O';
    displayStr[1] = 'f';
    displayStr[2] = 'l';
    displayStr[3] = 'o';
    displayStr[4] = 'w';
  }
  int bias = sizeof(displayStr) - 1;
  for (int i = numberString.length() - 1; i >= 0; --i)
  {
    displayStr[--bias] = numberString[i];
  }
  for (int i = 0; i < sizeof(displayStr) - 1; ++i)
  {
    matrix.drawChar(i * 6, 0, displayStr[i], HIGH, LOW, 1);
  }
  matrix.write();
}

/**
 * Hall sensor interrupt from liquid sensor
 * 
 * ~0.2 ml of liquid is flowing through the sensor for each interrupt
 * count them when pumpIsRunnins
 */ 
void liquidSensorISR() //measure the quantity of square wave
{
  if (pumpIsRunning)
    liquidVolume += 0.2;
  else
    displayNumber(liquidVolume);
}

/**
 * Runs pump using HW PWM from Arduino
 * 
 * It runs pump and opens solenoid in 200 ms after
 * 
 * duty: PWM duty cycles
 */ 
void runPump(int duty)
{
  pumpIsRunning = true;
  if (duty > 255)
  {
    duty = 255;
  }
  else if (duty < 0)
  {
    duty = 0;
  }
  analogWrite(PUMP_PIN, duty);
  delay(200);
  analogWrite(SOLENOID_PIN, 255);
}

void stopPump()
{
  pumpIsRunning = false;
  analogWrite(SOLENOID_PIN, 0);
  analogWrite(PUMP_PIN, 0);
  delay(300);
}

int angleToPulse(int angle)
{
  /*
      Converts angle to the number of pulses for servo

      Angle is the integer from 0 to 180 in degrees,
      where 0 is the most left position
      and 180 is the most right position.

      In real life the most left position could be more than 0,
      and the most right position could be less than 180 degrees.
  */
  if (angle > MAX_ANGLE)
  {
    Serial.println("WARNING: angle is more than MAX_ANGLE. Using MAX_ANGLE instead.");
    angle = MAX_ANGLE;
  }
  else if (angle < MIN_ANGLE)
  {
    Serial.println("WARNING: angle is less than MIN_ANGLE. Using MIN_ANGLE instead.");
    angle = MIN_ANGLE;
  }
  return MIN_ANGLE_DUTY + (DUTY_STEP * angle);
}

/**
 * Turns servo on specified angle
 * 
 * angle: angle to turn servo [0...180]
 * 
 * Servo is turning by generating PWM
 * signal with SERVO_PERIOD period.
 * Pulse duration is calculated from angle.
 * 
 * When servo is turning, voltage on it is changing.
 * In this function we calculate a number of pulses
 * where servo input voltage is constant.
 * 
 * If the voltage still constant during
 * SERVO_CONSTANT_VOLTAGE_PULSES steps,
 * we could conclude that servo is finished its turn.
 * 
 * 10 first pulses are always generated without interrupts
 * for consistency.
 * 
 */
void turnServo(int angle)
{
  // Counter for constant voltage
  int counter = 0;

  // Convert angle to pulse_duration
  OCR1A = angleToPulse(angle);

  // Small delay
  delay(3);

  // Wait until turn is finished
  while (true)
  {
    if (analogRead(ANALOG_VOLTAGE_PIN) == 1023)
    {
      counter++;
      if (counter % SERVO_CONSTANT_VOLTAGE_PULSES == 0)
      {
        CURRENT_ANGLE = angle;
        break;
      }
    }
    else
    {
      counter = 0;
    }
  }
}

/**
 * Gently returns servo to the starting position
 */
void returnServoToStart()
{
  int final_angle = CURRENT_ANGLE;
  for (int pos = final_angle; pos > MIN_ANGLE; pos -= final_angle / 40)
  {
    turnServo(pos);
    delay(1);
  }
  turnServo(MIN_ANGLE);
}

/**
 * Searches for glasses in horizontal plan
 */

void glassSearchHorizontal(int steps)
{
  glassArrayInd = 0;
  int angle_step = int((MAX_ANGLE - MIN_ANGLE) / steps);
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += angle_step)
  {
    turnServo(angle);
    range = vl.readRange();
#ifdef VERBOSE
    Serial.println(range);
#endif
    if ((range < horizontalEdgeThreshold) && !glassStartFound)
    {
      startCupAngle = CURRENT_ANGLE;
      glassStartFound = true;
    }
    if ((range > horizontalEdgeThreshold) && glassStartFound)
    {
      glassAngle = (CURRENT_ANGLE + startCupAngle) / 2;
#ifdef VERBOSE
      Serial.print("Target angle found: ");
      Serial.println(glassAngle);
#endif
      if ((CURRENT_ANGLE - startCupAngle) > 5)
      {
        glassArray[glassArrayInd++] = glassAngle;
      }
      glassStartFound = false;
    }
  }
}

/**
 * Fills glasses with a liquid
 * 
 * passes: number of passes
 */
void fillGlasses(int passes, int ml)
{
  for (int k = 0; k < passes; ++k)
  {
    for (int i = 0; i < glassArrayInd; ++i)
    {
      turnServo(glassArray[i]);
      // Delay here until we add a feedback for servo
      delay(200);
      runPump(255);
      while (liquidVolume < ml)
      {
        //delay(1);
        displayNumber(liquidVolume);
      }
      //displayNumber(liquidVolume);
      stopPump();
      //displayNumber(liquidVolume + 2);
#ifdef VERBOSE
      Serial.print("Flow, ml: ");
      Serial.println(liquidVolume);
#endif
      liquidVolume = 0;
    }
  }
}

/**
 * Main loop function which repeats by Arduino
 */
void loop()
{
  //analogWrite(SOLENOID_PIN, 255);
  int steps = 45;
  int passes = 1;
  int ml = 20;
  glassSearchHorizontal(steps);
  fillGlasses(passes, ml);
  returnServoToStart();
  while (true)
    ;
}



/***********************************************************************/
/*                     TEMPORARY UNUSED FUNCTIONS                      */
/***********************************************************************/

/**
 * Returns ground range for vertical glass search alg
 */
int getGroundRange()
{
  turnServo(0);
  delay(1000);
  return vl.readRange();
}

/**
 * Searches for glasses in vertical plan
 */
void glassSearchVertical(int steps)
{
  int ground = getGroundRange();
  int angle_step = int((MAX_ANGLE - MIN_ANGLE) / steps);
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += angle_step)
  {
    turnServo(angle);
    range = vl.readRange();
#ifdef VERBOSE
    Serial.println(range);
#endif

    if ((range < (ground - verticalEdgeThreshold)) && !glassStartFound)
    {
      startCupAngle = CURRENT_ANGLE;
      glassStartFound = true;
      glassStartRange = range;
#ifdef VERBOSE
      Serial.print("glassStartFound found!: ");
#endif
      continue;
    }

#ifdef VERBOSE
    Serial.print("glassStartRange - verticalEdgeThreshold : ");
    Serial.println((glassStartRange - verticalEdgeThreshold));
#endif

    if (glassStartFound && !glassBottomFound)
    {
      if (range < (glassStartRange + verticalEdgeThreshold))
      {
#ifdef VERBOSE
        Serial.println("continue!");
#endif
        continue;
      }
      else
      {
#ifdef VERBOSE
        Serial.println("glassBottomFound = true!");
#endif
        glassBottomFound = true;
      }
    }

    if ((range < (glassStartRange + verticalEdgeThreshold) && (range > (glassStartRange - verticalEdgeThreshold))) && glassStartFound && glassBottomFound && !glassEndFound)
    {
      glassAngle = (CURRENT_ANGLE + startCupAngle) / 2 + 5;
#ifdef VERBOSE
      Serial.print("Cup angle found: ");
      Serial.println(glassAngle);
      Serial.print("At range: ");
      Serial.println(range);
#endif
      glassArray[glassArrayInd++] = glassAngle;
      glassEndFound = true;
    }

    if (glassEndFound)
    {
      if (range < (glassStartRange + verticalEdgeThreshold))
      {
        Serial.println("continue on the end edge!");
        continue;
      }
      else
      {
        Serial.println("finish!");
        glassEndFound = false;
        glassStartFound = false;
        glassBottomFound = false;
      }
    }
  }
}
