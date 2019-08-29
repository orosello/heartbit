#include <Wire.h>
#include <SD.h>

//Definitions
enum {
  HR_RX = 12,
  MOTOR_PIN = 13,
  
  BUFFER_SIZE = 100, // How many beat timestamps we store at once
  
  SECOND_MS = 1000,
  MINUTE_MS = 60 * SECOND_MS,
  BPM_PERIOD_MS = 5 * SECOND_MS, // Window size for calculating BPM over
  BPM_REFRESH_PERIOD = 1 * SECOND_MS, // How frequently BPM is re-calculated
  MIN_TIME_BETWEEN_BEATS_MS = 100, // How long to wait between valid beat timestamps
  
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

byte last_sample=0;
unsigned long last_bpm_time=0;
unsigned long beatBuffer[BUFFER_SIZE];
int beatIndex = BUFFER_SIZE - 1;
bool initialized = false;
unsigned long last_beat_time=0;
unsigned long time_offset=0;
buzz_state_t buzz_state = BUZZ_OFF;
unsigned long buzz_last_updated = 0;

int calculate_bpm(const unsigned long* buffer, const unsigned last_beat_index, const unsigned now) {  
  int beats_in_last_period = 1;
  int i;
  // Start from last valid beat and search backwards for more
  for (i = last_beat_index - 1; i >= 0; i--) {
    const long diff = now - buffer[i];
    if (diff > BPM_PERIOD_MS) {
      // We're at beat timestamps that are too old, quit counting
      break;
    }
    beats_in_last_period++;
  }
  // Start from end of buffer and keep searching backwards
  for (i = BUFFER_SIZE - 1; i > last_beat_index; i--) {
    const long diff =  now - buffer[i];
    if (diff > BPM_PERIOD_MS) {
      // Bail on encountering too-old timestamps
      break;
    }
    beats_in_last_period++;
  }
  const int bpm = beats_in_last_period * MINUTE_MS / BPM_PERIOD_MS;
  return bpm;
}

unsigned long time_since_start(void){
  const unsigned long now = millis();
  if (time_offset == 0) { // should only set this value the first time
    time_offset = now;
  }
  return now - time_offset;
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  pinMode (HR_RX, INPUT);  //Signal pin to input
  pinMode(MOTOR_PIN, OUTPUT);

  // Wait until a heart beat is detected
  while (!digitalRead(HR_RX));
  for(int i = 0; i < BUFFER_SIZE; i++) {
    beatBuffer[i] = 0;
    beatBuffer[i] = 0;
  }
}

void execute_buzz(const unsigned long time){
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
      break;
    default:
      break;
  }
}

void loop() {
  // detection code
  const unsigned long time = time_since_start();
  const byte sample = digitalRead(HR_RX); // Store signal input
  if (sample && last_sample != sample && time - last_beat_time > MIN_TIME_BETWEEN_BEATS_MS) {
    last_beat_time = time;
    Serial.println("Beat at " + String(time));
    beatIndex = (beatIndex + 1) % BUFFER_SIZE;
    beatBuffer[beatIndex] = time;

    buzz_state = BUZZ_STAGE_FIRST_BUZZ;
    buzz_last_updated = time;
  }
  last_sample = sample; //Store last signal received

  if (time > BPM_PERIOD_MS && time - last_bpm_time >= BPM_REFRESH_PERIOD) {
    const int bpm = calculate_bpm(beatBuffer, beatIndex, time);
    Serial.println("Bpm: " + String(bpm));
    last_bpm_time = time;
  }

  execute_buzz(time);
}
 
