
/*
 * Servo EMAX ES09MD
 * Simple control program.
 * 
 * Characteristics:
 *  Pulse frequency = 3040 microS
 *  Min angle = 950 microS
 *  Max angle = 2120 microS
 * 
 * Connections:
 *  Red: +5V
 *  Brown: GND
 *  Yellow: digital pin 2 and analog pin A0
 *  (should be connected in one point)
 *  
 * Additionals:
 *  Arduino servo library is working slowly
 *  with this digital servo.  
 *  That's why it's implemented manually here.
*/

/*
 * Globals section
*/

/* Verbose output */
bool VERBOSE = true;
/* Measurements in micro seconds for real servo */
int MIN_ANGLE_PULSE = 950;
int MAX_ANGLE_PULSE = 2120;
int SERVO_PERIOD = 3040;
/* Servo MIN and MAX turn angles */
int MAX_ANGLE = 180;
int MIN_ANGLE = 0;
/* Calculate pulse step */
int PULSE_STEP = (MAX_ANGLE_PULSE - MIN_ANGLE_PULSE) / (MAX_ANGLE - MIN_ANGLE);
/* Current angle of servo */
int CURRENT_ANGLE = 0;
/* Max pulse number for servo */
int MAX_PULSE_NUMBER = 10000;
/* Number of pulses to check servo is finished its turn */
int SERVO_CONSTANT_VOLTAGE_PULSES = 15;


/*
 * Pinout section
*/
int SERVO_PIN = 2;            // Servo PIN on Arduino
int ANALOG_VOLTAGE_PIN = A0; // Pin to measure analog voltage


/* Setup function. Executes once when program starts. */
void setup() {
  if (VERBOSE) {
    // Open serial for communication with host
    Serial.begin(115200); 
  }
  // Output pin initialization
  pinMode(SERVO_PIN, OUTPUT);
}

int angle_to_pulse_duration(int angle) {
  /* 
   *  Converts angle to the number of pulses for servo
   *  
   *  Angle is the integer from 0 to 180 in degrees,
   *  where 0 is the most left position
   *  and 180 is the most right position.
   *  
   *  In real life the most left position could be more than 0,
   *  and the most right position could be less than 180 degrees.
  */
  if (angle > MAX_ANGLE) {
    Serial.println("WARNING: angle is more than MAX_ANGLE. Using MAX_ANGLE instead.");
    angle = MAX_ANGLE;
  } else if (angle < MIN_ANGLE) {
    Serial.println("WARNING: angle is less than MIN_ANGLE. Using MIN_ANGLE instead.");
    angle = MIN_ANGLE;
  }
  return MIN_ANGLE_PULSE + (PULSE_STEP * angle); 
}

void turn_servo(int angle) {
  /*
   * Turns servo on specified angle
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
  */

  // Convert angle to pulse_duration
  int pulse_duration = angle_to_pulse_duration(angle);
  
  int counter = 0;
  int i;
  for (i = 0; i < MAX_PULSE_NUMBER; ++i) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulse_duration);
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(SERVO_PERIOD - pulse_duration);
    if (i < 10) {
      continue;
    } else {
      if (analogRead(ANALOG_VOLTAGE_PIN) == 1023) {
        counter++;
        if(counter % SERVO_CONSTANT_VOLTAGE_PULSES == 0) {
          break;
        }
      } else {
        counter = 0;
      }
    }
  }
  /* Set-up global angle variable */
  CURRENT_ANGLE = angle;

  /* Verbose output */
  if (VERBOSE) {
    Serial.println("Turn finished!");
    Serial.println("Current pulse: ");
    Serial.print(i);
    Serial.println("\nCurrent angle: ");
    Serial.print(CURRENT_ANGLE);
    Serial.println();
    delay(1000); 
  }
}


/* 
 *  TESTS SECTION
*/

void servo_minmax_test() {
  /*
   * Min/max turn test for servo
   * 
   * Turns servo to MIN_ANGLE and then to MAX_ANGLE
  */
  turn_servo(MIN_ANGLE);
  turn_servo(MAX_ANGLE);
  if (VERBOSE) {
    Serial.println("Servo minmax test SUCCESS!");
  }
}

void servo_steps_test(int steps) {
  /*
   * Steps test for servo
   * 
   * Turns servo from MIN_ANGLE to MAX_ANGLE and back
   * with fixed number of equal steps passed via steps parameter
   * 
  */
  int angle_step = int((MAX_ANGLE - MIN_ANGLE) / steps);
  for (int angle=MIN_ANGLE; angle<=MAX_ANGLE; angle+=angle_step){
    turn_servo(angle);
  }
  for (int angle = MAX_ANGLE; angle>=MIN_ANGLE; angle-=angle_step){
    turn_servo(angle);
  }
  if (VERBOSE) {
    Serial.println("Servo steps test SUCCESS!");
  }
}

void servo_random_test(int steps) {
  /*
   * Random test for servo
   * 
   * Turns servo to the random angle
   * from MIN_ANGLE to MAX_ANGLE
   * several times specified by steps parameter
   * 
  */
  int randomAngle;
  for (int i=0; i<steps; ++i) {
    randomAngle = random(MIN_ANGLE, MAX_ANGLE);
    turn_servo(randomAngle);
  }
  if (VERBOSE) {
    Serial.println("Servo random test SUCCESS!");
  }
}

/* Main loop function which repeats by Arduino */
void loop() {
  /* Run minmax test */
  servo_minmax_test();

  /* Run steps test */
  int equalSteps = 4;
  servo_steps_test(equalSteps);

  /* Run random test */
  int randomSteps = 3;
  servo_random_test(randomSteps);
}
