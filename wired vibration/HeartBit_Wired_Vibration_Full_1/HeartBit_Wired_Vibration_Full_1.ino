#include <Wire.h>
#include <SD.h>

//Definitions
enum {
  HR_RX = 12,
  MOTOR_PIN = 13,
  SECOND_MS = 1000,
  MINUTE_MS = 60 * SECOND_MS,
  PERIOD_MS = 15 * SECOND_MS,
  REFRESH_PERIOD = 1 * SECOND_MS,
  BUFFER_SIZE = 200,
  
  FIRST_BUZZ_MS = 100,
  FIRST_BUZZ_PAUSE_MS = 50,
  SECOND_BUZZ_MS = 50,
};

typedef enum {
    BUZZ_STAGE_FIRST_BUZZ, 
    BUZZ_STAGE_FIRST_PAUSE, 
    BUZZ_STAGE_SECOND_BUZZ, 
    BUZZ_OFF
} buzz_state_t;

byte last_sample;
unsigned long last_bpm_time;
unsigned long beatBuffer[BUFFER_SIZE];
int beatIndex = 0;
bool initialized = false;
unsigned int minTimeBetweenBeats = 0;
unsigned long last_beat_time=0;
unsigned long time_offset=0;
buzz_state_t buzz_state = BUZZ_OFF;
unsigned long buzz_last_updated = 0;
bool non_blocking_delay = true;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  pinMode (HR_RX, INPUT);  //Signal pin to input
  pinMode(MOTOR_PIN, OUTPUT);
  // Serial.println("Waiting for heart beat...");

  // Wait until a heart beat is detected
  while (!digitalRead(HR_RX));
  // Serial.println(666);
  // Serial.println ("Heart beat detected!");
  for(int k = 0; k < BUFFER_SIZE; k++) {
    beatBuffer[k] = -1;
    beatBuffer[k] = -1;
  }
}

void loop() {
  // detection code
  unsigned long time = millis();
  if (time_offset == 0) { // should only set this value the first time
    time_offset = time;
  }
  time -= time_offset;
  
  byte sample = digitalRead(HR_RX); //Store signal output
  if (sample && last_sample != sample && time - last_beat_time > minTimeBetweenBeats) {
    last_beat_time = time;
    Serial.println("Beat at " + String(time));
//    Serial.println(beatIndex);
//    Serial.println(time);
//    Serial.println("-------");
    if (beatIndex >= BUFFER_SIZE) {
//      Serial.println("fuuuuuck");
      for (int i = 0; i < BUFFER_SIZE / 2; i++) {
        beatBuffer[i] = beatBuffer[BUFFER_SIZE / 2 + i];
        beatBuffer[BUFFER_SIZE / 2 + i] = -1;
      }
      beatIndex = BUFFER_SIZE / 2;
    }
    beatBuffer[beatIndex++] = time;
//    
//    beatCounter++;
////    Serial.println("-------");
////    Serial.println(beatCounter);
////    Serial.println("-------");
//    time = millis();
//    if (time - last_bpm_time >= PERIOD_MS) {
//      last_bpm_time = time;
//      Serial.println(beatCounter);
//      double bpm = beatCounter * (MINUTE_MS / PERIOD_MS);
//      Serial.println(bpm);
//      beatCounter = 0;
//    }

//      Serial.println("Beat");
      if (non_blocking_delay) {
        // NON BLOCKING DELAY
        buzz_state = BUZZ_STAGE_FIRST_BUZZ;
        buzz_last_updated = time;
//        digitalWrite(MOTOR_PIN, HIGH);//comente
      } //else {
//        //digitalWrite(13, HIGH);
//        digitalWrite(MOTOR_PIN, HIGH);
//        delay(100);
//
//        //digitalWrite(13, LOW);
//        digitalWrite(MOTOR_PIN, LOW);
//  //    delay(50);
//  //
//  //    digitalWrite(13, HIGH);
//  //    digitalWrite(MOTOR_PIN, HIGH);
//  //    delay(50);
//  //
//  //    digitalWrite(13, LOW);
//  //    digitalWrite(MOTOR_PIN, LOW);
//  //    delay(25);     
//      }
  }

    if (non_blocking_delay) {
      switch(buzz_state) {
        case BUZZ_STAGE_FIRST_BUZZ:
          if (time - buzz_last_updated > FIRST_BUZZ_MS) {
            digitalWrite(MOTOR_PIN, LOW);
            buzz_last_updated = time;
            buzz_state = BUZZ_STAGE_FIRST_PAUSE;
          }       
          break;
        case BUZZ_STAGE_FIRST_PAUSE:
          if (time - buzz_last_updated > FIRST_BUZZ_PAUSE_MS) {
            digitalWrite(MOTOR_PIN, HIGH);
            buzz_last_updated = time;
            buzz_state = BUZZ_STAGE_SECOND_BUZZ;
          }
          break;
        case BUZZ_STAGE_SECOND_BUZZ:
          if (time - buzz_last_updated > SECOND_BUZZ_MS) {
            digitalWrite(MOTOR_PIN, LOW);
            buzz_last_updated = time;
            buzz_state = BUZZ_OFF;
          }       
          break;
        case BUZZ_OFF:
        default:
          break;
      }
    }
  
  last_sample = sample;           //Store last signal received

  if (time >= PERIOD_MS && time - last_bpm_time >= REFRESH_PERIOD) {
    last_bpm_time = time;
    int beats = 0;
    for (int j = beatIndex - 1; j >= 0; j--) {
      if (beatBuffer[j] == -1) {
        continue;
      }
      unsigned long diff = time - beatBuffer[j];
      if (diff < PERIOD_MS) {
        beats++;
      }
    }
    int bpm = beats * MINUTE_MS / PERIOD_MS;
    Serial.println("Bpm: " + String(bpm));
//    Serial.println(bpm);
  }

}
 
