volatile double waterFlow;

int PUMP_PIN = 5;

void setup() {
  Serial.begin(115200); //baudrate
  waterFlow = 0;
  pinMode(PUMP_PIN, OUTPUT);
  attachInterrupt(0, pulse, RISING); //DIGITAL Pin 2: Interrupt 0
}

void pulse() //measure the quantity of square wave
{
  // 5000 is the measured constant
  waterFlow += 1.0 / 5000.0 * 1000;
}

void run_pump(int duty) {
  if (duty > 255) {
    duty = 255;
  } else if (duty < 0) {
    duty = 0;
  }
  analogWrite(PUMP_PIN, duty);
}

void stop_pump() {
  analogWrite(PUMP_PIN, 0);
}

void loop() {
  //delay(1000);
  run_pump(255);
  delay(1000);
  stop_pump();
  delay(1000);
  //Serial.println(waterFlow);
  //while(1);
}

