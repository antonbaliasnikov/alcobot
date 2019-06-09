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
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>

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

/* Pump inputs */
int PUMP_PIN = 3;

int SOLENOID_PIN = 5;

int range = 0;
int counter = 0;
bool not_turned = true;
bool finished = false;
int measurements = 0;

unsigned long time;

/* 
 *  Waterflow sensor
*/
volatile double waterFlow;

/**
 * Display setup
 */
int pinCS = 10;
int numberOfHorizontalDisplays = 1;
int numberOfVerticalDisplays = 4;
Max72xxPanel matrix = Max72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);

/* Setup function. Executes once when program starts. */
void setup() {

  // _BV means 1 << BIT => set BIT in register

  /* Using mode 14 fast PWM of timer 1
    PWM period is 16 MHz / prescaler / ICR1
    Duty = OCR1A / ICR1
  */

  pinMode(FAST_PWM_SERVO_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);

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

  attachInterrupt(0, pulse, RISING); //DIGITAL Pin 2: Interrupt 0

  matrix.setIntensity(0); // яркость
  matrix.setRotation(3);
}

void displayNumber(int number) {
  matrix.fillScreen(LOW);
  char displayStr[] = "     ";
  String numberString = String(number);
  if (numberString.length() > 5) {
    displayStr[0] = 'O';
    displayStr[1] = 'f';
    displayStr[2] = 'l';
    displayStr[3] = 'o';
    displayStr[4] = 'w';
  }
  int bias = sizeof(displayStr) - 1;
  for(int i = numberString.length() - 1; i >=0 ; --i) {
    displayStr[--bias] = numberString[i];
  }
  for(int i = 0; i < sizeof(displayStr) - 1; ++i) {
    matrix.drawChar(i*6, 0, displayStr[i], HIGH, LOW, 1);
  }
  matrix.write();
}

bool pump_is_run = false;

void pulse() //measure the quantity of square wave
{
  // 5000 is the measured constant
  if (pump_is_run) waterFlow += 0.2;
}

void run_pump(int duty) {
  pump_is_run = true;
  if (duty > 255) {
    duty = 255;
  } else if (duty < 0) {
    duty = 0;
  }
  analogWrite(PUMP_PIN, duty);
  delay(200);
  analogWrite(SOLENOID_PIN, 255);
}

void stop_pump() {
  pump_is_run = false;
  analogWrite(SOLENOID_PIN, 0);
  analogWrite(PUMP_PIN, 0);
  delay(1500);
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
  delay(3);

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

int start_cup_angle = 0;
bool start_found = false;
bool bottom_found = false;
bool end_found = false;
int cup_angle = 0;
int edge_threshold_vertical = 20;
int edge_threshold = 150;
int start_cup_range;

int pos_array[10];
int pos_array_ind = 0;

int ml = 80;

int get_ground_range(void) {
  turn_servo(0);
  delay(1000);
  return vl.readRange();
}

void horizontal_search_alg() {
  //time = micros();
  //Serial.println(micros() - time);
  int steps = 30;
  int angle_step = int((MAX_ANGLE - MIN_ANGLE) / steps);
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += angle_step) {
    turn_servo(angle);
    //Serial.println(CURRENT_ANGLE);
    range = vl.readRange();
    if ((range < edge_threshold) && !start_found) {
      start_cup_angle = CURRENT_ANGLE;
      start_found = true;
    }
    Serial.println(range);

    if ((range > edge_threshold) && start_found) {
      cup_angle = (CURRENT_ANGLE + start_cup_angle) / 2;
      Serial.print("Target angle found: ");
      Serial.println(cup_angle);
      //Serial.print("At range: ");
      //Serial.println(range);
      if ((CURRENT_ANGLE - start_cup_angle) > 5) {
        pos_array[pos_array_ind++] = cup_angle;
      }
      start_found = false;
    }
  }
  long flow_time = 0;
  for ( int k = 0; k < 1; ++k ) {
    for (int i = 0; i < pos_array_ind; ++i) {
      turn_servo(pos_array[i]);
      // Delay here until we add a feedback for servo
      delay(200);
      run_pump(255);
      // -2 is inertial constant
      while(waterFlow < ml - 2) {
        //delay(1);
        displayNumber(waterFlow);
        //flow_time += 1;
      }
      displayNumber(waterFlow);
      stop_pump();
      displayNumber(waterFlow + 2);
      //Serial.print("Flow time, iterations: ");
      //Serial.println(flow_time);
      //Serial.print("Flow, ml: ");
      //Serial.println(waterFlow);
      waterFlow = 0;
      flow_time = 0;
    }
  }
  int final_angle = CURRENT_ANGLE;
  for (int pos = final_angle; pos > 0; pos -= final_angle/40){
    turn_servo(pos);
    delay(1);
  }
  turn_servo(0);
  while (true);
}

void vertical_search_alg() {
  int ground = get_ground_range();

  int steps = 60;
  int angle_step = int((MAX_ANGLE - MIN_ANGLE) / steps);
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += angle_step) {
    turn_servo(angle);
    //Serial.println(CURRENT_ANGLE);
    range = vl.readRange();

    //Serial.print("range : ");
    Serial.println(range);
    //continue;

    if ((range < (ground - edge_threshold_vertical)) && !start_found) {
      start_cup_angle = CURRENT_ANGLE;
      start_found = true;
      start_cup_range = range;
      Serial.print("start_found found!: ");
      continue;
    }

    Serial.print("start_cup_range - edge_threshold_vertical : ");
    //Serial.println((start_cup_range - edge_threshold_vertical));

    if (start_found && !bottom_found) {
      if (range < (start_cup_range + edge_threshold_vertical)) {
        Serial.println("continue!");
        continue;
      } else {
        Serial.println("bottom_found = true!");
        bottom_found = true;
      }
    }

    if ((range < (start_cup_range + edge_threshold_vertical) && (range > (start_cup_range - edge_threshold_vertical))) && start_found && bottom_found && !end_found) {
      cup_angle = (CURRENT_ANGLE + start_cup_angle) / 2 + 5;
      Serial.print("Cup angle found: ");
      //Serial.println(cup_angle);
      //Serial.print("At range: ");
      //Serial.println(range);
      //if ((CURRENT_ANGLE - cup_angle) > 5) {
      pos_array[pos_array_ind++] = cup_angle;
      //}
      end_found = true;
    }

    if (end_found) {
      if (range < (start_cup_range + edge_threshold_vertical)) {
        Serial.println("continue on the end edge!");
        continue;
      } else {
        Serial.println("finish!");
        end_found = false;
        start_found = false;
        bottom_found = false;
      }
    }

  }
  long flow_time = 0;
  for ( int k = 0; k < 1; ++k ) {
    for (int i = 0; i < pos_array_ind; ++i) {
      turn_servo(pos_array[i]);
      run_pump(255);
      while(waterFlow < ml - 2) {
        delay(1);
        flow_time += 1;
      }
      stop_pump();
      Serial.print("Flow time, iterations: ");
      Serial.println(flow_time);
      Serial.print("Flow, ml: ");
      Serial.println(waterFlow);
      waterFlow = 0;
      flow_time = 0;
    }
  }
  int final_angle = CURRENT_ANGLE;
  for (int pos = final_angle; pos > 0; pos -= final_angle/40){
    turn_servo(pos);
    delay(1);
  }
  turn_servo(0);
  while (true);
}

/* Main loop function which repeats by Arduino */
void loop() {
  //analogWrite(SOLENOID_PIN, 255);
  horizontal_search_alg();
}









