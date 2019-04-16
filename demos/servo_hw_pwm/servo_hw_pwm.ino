// Using Timer2
/*
   TCCR => Timer/Counter Control Register for Timer2 (there are 2 registers A and B)

   WGM2 - Waveform Generation Mode bits for Timer 2
   011 to choose Fast PWM Mode: _BV(WGM21) | _BV(WGM20

   111 to use OCRA as controlling the top limit to get necessary frequency.

   Compare Match Output A and B (COM2A and COM2B)
   enable/disable/invert outputs A or B
   _BV(COM2A1) | _BV(COM2B1) => non-inverted PWM outputs for A and B

   OCR2A => "A" Output Compare Register for Timer 2, used as top value for the timer
   OCR2B => "B" Output Compare Register for Timer 2

   OCR2A's mode is set to "Toggle on Compare Match" by setting the COM2A bits to 01.

*/
/*
   Globals section
*/

#include <Wire.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();

/* Verbose output */
bool VERBOSE = true;
/* Measurements in micro seconds for real servo */
int MIN_ANGLE_DUTY = 1900;
int MAX_ANGLE_DUTY = 4240;
int SERVO_PERIOD = 6080;
/* Servo MIN and MAX turn angles */
int MAX_ANGLE = 180;
int MIN_ANGLE = 0;
/* Current angle of servo */
int CURRENT_ANGLE = 0;
/* Number of pulses to check servo is finished its turn */
int SERVO_CONSTANT_VOLTAGE_PULSES = 15;

/* Calculate pulse step */
int DUTY_STEP = (MAX_ANGLE_DUTY - MIN_ANGLE_DUTY) / (MAX_ANGLE - MIN_ANGLE);

/*
   Pinout section
*/
int FAST_PWM_SERVO_PIN = 9;  // Fast PWM Timer1 PIN to control servo
int ANALOG_VOLTAGE_PIN = A0; // Pin to measure analog voltage

int range = 0;
int counter = 0;
bool not_turned = true;
bool finished = false;
int measurements = 0;

unsigned long time;

/* Setup function. Executes once when program starts. */
void setup() {

  // _BV means 1 << BIT => set BIT in register

  /* Using mode 14 fast PWM of timer 1
    PWM period is 16 MHz / prescaler / ICR1
    Duty = OCR1A / ICR1
  */

  pinMode(FAST_PWM_SERVO_PIN, OUTPUT);

  /*  Set-up Timer1 control registers
      Enabling Fast PWM mode (mode 14) for Timer 1
      Set pre-scaler to 8
      1 tick is 0.5 microS
      3040 microS = 6080 ticks, setting OCR1 to 6080
      Range in microS for servo: 950 to 2120
      middle position is 1535
  */
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS11);

  /* Period */
  ICR1 = SERVO_PERIOD;

  /* Switching value */
  OCR1A = angle_to_pulse_duration(0);
  delay(700);

  if (VERBOSE) {
    // Open serial for communication with host
    Serial.begin(115200);
  }

  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}

int angle_to_pulse_duration(int angle) {
  /*
      Converts angle to the number of pulses for servo

      Angle is the integer from 0 to 180 in degrees,
      where 0 is the most left position
      and 180 is the most right position.

      In real life the most left position could be more than 0,
      and the most right position could be less than 180 degrees.
  */
  if (angle > MAX_ANGLE) {
    Serial.println("WARNING: angle is more than MAX_ANGLE. Using MAX_ANGLE instead.");
    angle = MAX_ANGLE;
  } else if (angle < MIN_ANGLE) {
    Serial.println("WARNING: angle is less than MIN_ANGLE. Using MIN_ANGLE instead.");
    angle = MIN_ANGLE;
  }
  return MIN_ANGLE_DUTY + (DUTY_STEP * angle);
}

void turn_servo(int angle) {
  /*
     Turns servo on specified angle

     Servo is turning by generating PWM
     signal with SERVO_PERIOD period.
     Pulse duration is calculated from angle.

     When servo is turning, voltage on it is changing.
     In this function we calculate a number of pulses
     where servo input voltage is constant.

     If the voltage still constant during
     SERVO_CONSTANT_VOLTAGE_PULSES steps,
     we could conclude that servo is finished its turn.

     10 first pulses are always generated without interrupts
     for consistency.
  */
  counter = 0;

  // Convert angle to pulse_duration
  OCR1A = angle_to_pulse_duration(angle);

  // Small delay
  delay(1);

  while (true) {
    if (analogRead(ANALOG_VOLTAGE_PIN) == 1023) {
      counter++;
      if (counter % 10 == 0) {
        CURRENT_ANGLE = angle;
        break;
      }
    } else {
      counter = 0;
    }
  }
}

void disable_pwm() {
  TCCR1A |= ~_BV(COM1A1);
}

void enable_pwm() {
  TCCR1A |= _BV(COM1A1);
}

int threshold = 150;

int local_angle = 0;
bool start_found = false;
int target_angle = 0;

int pos_array[10];
int pos_array_ind = 0;

/* Main loop function which repeats by Arduino */
void loop() {
  //time = micros();
  //Serial.println(micros() - time);
  int steps = 180;
  int angle_step = int((MAX_ANGLE - MIN_ANGLE) / steps);
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += angle_step) {
    turn_servo(angle);
    //Serial.println(CURRENT_ANGLE);
    range = vl.readRange();
    if ((range < threshold) && !start_found) {
      local_angle = CURRENT_ANGLE;
      start_found = true;
    }      
    Serial.println(range);

    if ((range > threshold) && start_found) {
      target_angle = (CURRENT_ANGLE + local_angle) / 2 - 5;
      Serial.print("Target angle found: ");
      Serial.println(target_angle);
      //Serial.print("At range: ");
      //Serial.println(range);
      if ((CURRENT_ANGLE - local_angle) > 5) {
        pos_array[pos_array_ind++] = target_angle; 
      }
      start_found = false;
    }
  }
  for (int i = 0; i<pos_array_ind; ++i) {
    turn_servo(pos_array[i]);
    delay(1500);
  }
  turn_servo(0);
  while(true);
}














