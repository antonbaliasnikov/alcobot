//#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>

int pinCS = 10;
int numberOfHorizontalDisplays = 1;
int numberOfVerticalDisplays = 4;

Max72xxPanel matrix = Max72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);

String tape = "0123456789"; // текст, который будет плыть
int number = 12345;
int wait = 1000; // время между крайними перемещениями букв

int spacer = 1; // расстояние между буквами
int width = 5 + spacer; // размер шрифта

void setup() {
    matrix.setIntensity(7); // яркость
//    matrix.setPosition(0, 0, 0);
//    matrix.setPosition(1, 0, 1);
//    matrix.setPosition(2, 0, 2);
//    matrix.setPosition(3, 0, 3);
    /*
   * Define if and how the displays are rotated. The first display
   * (0) is the one closest to the Arduino. rotation can be:
   *   0: no rotation
   *   1: 90 degrees clockwise
   *   2: 180 degrees
   *   3: 90 degrees counter clockwise
   */
    matrix.setRotation(3);
    Serial.begin(115200);
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

void loop() {
  displayNumber(432);
  while(true);
    /*
    for ( int i = 0 ; i < width * tape.length() + matrix.width() - 1 - spacer; i++ ) {
        matrix.fillScreen(LOW);

        int letter = i / width;
        int x = (matrix.width() - 1) - i % width;
        int y = (matrix.height() - 8) / 2; // center the text vertically

        while ( x + width - spacer >= 0 && letter >= 0 ) {
            if ( letter < tape.length() ) {
                matrix.drawChar(x, y, tape[letter], HIGH, LOW, 1);
            }
            letter--;
            x -= width;
        }

        matrix.write();
        delay(wait);
    }
    */
}
