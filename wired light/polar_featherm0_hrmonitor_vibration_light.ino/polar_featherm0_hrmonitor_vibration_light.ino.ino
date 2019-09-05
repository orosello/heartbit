//#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

//Definitions
const int HR_RX = 6;
const int motorPin = 12;
byte oldSample, sample;
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

static const uint8_t PROGMEM
  disp_on[] =
  { B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111 },
  disp_small[] =
  { B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000,
    B00000000 },
  disp_large[] =
  { B00000000,
    B00000000,
    B00111100,
    B00111100,
    B00111100,
    B00111100,
    B00000000,
    B00000000 },
   disp_off[] =
  { B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000 };

void setup() {
  Serial.begin(9600);
  while(!Serial);
  pinMode (HR_RX, INPUT);  //Signal pin to input
  pinMode(motorPin, OUTPUT);
  Serial.println("Waiting for heart beat...");

  //Wait until a heart beat is detected
  while (!digitalRead(HR_RX));
  Serial.println ("Heart beat detected!");

  matrix.begin(0x70);  // pass in the address
}

static void draw_bitmap(const uint8_t bitmap[]){
    matrix.clear();
    matrix.drawBitmap(0, 0, bitmap, 8, 8, LED_ON);
    matrix.writeDisplay();
}

void loop() {
  //detection code
  sample = digitalRead(HR_RX); //Store signal output
  if (sample && (oldSample != sample)) {
    Serial.println("Beat");
    digitalWrite(13, HIGH);
    digitalWrite(motorPin, HIGH);
    draw_bitmap(disp_on);
    delay(100);

    digitalWrite(13, LOW);
    digitalWrite(motorPin, LOW);
    draw_bitmap(disp_large);
    delay(50);

    digitalWrite(13, HIGH);
    digitalWrite(motorPin, HIGH);
    draw_bitmap(disp_small);
    delay(50);

    digitalWrite(13, LOW);
    digitalWrite(motorPin, LOW);
    draw_bitmap(disp_off);
    delay(25);     
  } 
  oldSample = sample;           //Store last signal received

}
 
