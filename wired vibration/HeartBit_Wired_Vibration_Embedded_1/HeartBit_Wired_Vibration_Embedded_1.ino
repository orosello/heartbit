#include <Wire.h>
#include <SD.h>
//#include <cmath>

//Definitions
enum {
  HR_RX = 12,
  MOTOR_PIN = 13,
  
  BUFFER_SIZE = 100, // How many beat timestamps we store at once
  
  SECOND_MS = 1000,
  MINUTE_MS = 60 * SECOND_MS,
  BPM_LONG_PERIOD_MS = 15 * SECOND_MS, // Bigger window size for calculating BPM over
  BPM_SHORT_PERIOD_MS = 5 * SECOND_MS, // Smaller window size for calculating BPM over
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

long time_offset=0;
byte last_sample=0;

long last_bpm_time=0;
unsigned int current_bpm_long = 0;
unsigned int current_bpm_short = 0;
float current_hrv = 0.0f;
long beatBuffer[BUFFER_SIZE];
int beatIndex = BUFFER_SIZE - 1;
long last_beat_time=0;

buzz_state_t buzz_state = BUZZ_OFF;
long buzz_last_updated = 0;

int index_of_first_beat_in_window(const long buffer[BUFFER_SIZE], 
                                  const unsigned last_beat_index,
                                  const unsigned now,
                                  const unsigned period_ms) {
  // Start from previous beat and search backwards for more
  for (int i = last_beat_index - 1; i >= 0; i--) {
    const long unsigned diff = now - buffer[i];
    if (diff > period_ms || buffer[i] == 0) {
      // We're at beat timestamps that are too old, return last valid index
      return i + 1;
    }
  }
  // Start from end of buffer and keep searching backwards
  for (int i = BUFFER_SIZE - 1; i > last_beat_index; i--) {
    const long unsigned diff =  now - buffer[i];
    if (diff > period_ms || buffer[i] == 0) {
      // Bail on encountering too-old, or unfilled timestamps
      return (i + 1) % BUFFER_SIZE;
    }
  }
  // We made it all the way around and all values were within window
  return last_beat_index + 1;
}

int calculate_bpm(const long buffer[BUFFER_SIZE], const unsigned last_beat_index,
                  const unsigned now, const unsigned period_ms) {
  const int first_in_window = index_of_first_beat_in_window(buffer, last_beat_index,
                                                            now, period_ms);
  const int beats_in_last_period = (last_beat_index + BUFFER_SIZE - first_in_window + 1) % BUFFER_SIZE;
  const int bpm = beats_in_last_period * MINUTE_MS / period_ms;
  return bpm;
}

float calculate_hrv_time(const long buffer[BUFFER_SIZE], const int last_beat_index,
                       const unsigned now, const unsigned period_ms) {
  const int first_in_window = index_of_first_beat_in_window(buffer, last_beat_index,
                                                            now, period_ms);
  if (first_in_window == last_beat_index) {
    return 0.0f; // Zero intervals
  }
  if ((first_in_window +1) % BUFFER_SIZE == last_beat_index) {
    // Only one interval present, leave
    return 0.0f;
  }
  float current_average = 0.0f;
  unsigned n = 0;
  for (int i=first_in_window; ((i + 1) % BUFFER_SIZE) != last_beat_index; i++) {
    const long interval_1 = buffer[(i+1) % BUFFER_SIZE] - buffer[(i) % BUFFER_SIZE];
    const long interval_2 = buffer[(i+2) % BUFFER_SIZE] - buffer[(i+1) % BUFFER_SIZE];
    const long interval_diff = interval_2 - interval_1;
    const long interval_diff_squared = interval_diff * interval_diff;
    n++;
    current_average += (interval_diff_squared - current_average) / (float) n;
  }

  return sqrt(current_average);
}

long time_since_start(void){
  const long now = millis();
  return now - time_offset;
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  pinMode (HR_RX, INPUT);  //Signal pin to input
  pinMode(MOTOR_PIN, OUTPUT);

  for(int i = 0; i < BUFFER_SIZE; i++) {
    beatBuffer[i] = 0;
    beatBuffer[i] = 0;
  }
  // Wait until a heart beat is detected
  while (!digitalRead(HR_RX));
  
  const long now = millis();
  time_offset = now;
}

void execute_buzz(const long time){
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
  const long time = time_since_start();
  const byte sample = digitalRead(HR_RX); // Store signal input
  if (sample && last_sample != sample && time - last_beat_time > MIN_TIME_BETWEEN_BEATS_MS) {
    last_beat_time = time;
    Serial.println(String(time) + "," + String(current_bpm_short) + "," + String(current_bpm_long) + "," + String(current_hrv));
    beatIndex = (beatIndex + 1) % BUFFER_SIZE;
    beatBuffer[beatIndex] = time;

    buzz_state = BUZZ_STAGE_FIRST_BUZZ;
    buzz_last_updated = time;
  }
  last_sample = sample; //Store last signal received

  if (time - last_bpm_time >= BPM_REFRESH_PERIOD) {
    current_bpm_long = calculate_bpm(beatBuffer, beatIndex, time, BPM_LONG_PERIOD_MS);
    current_bpm_short = calculate_bpm(beatBuffer, beatIndex, time, BPM_SHORT_PERIOD_MS);
    current_hrv = calculate_hrv_time(beatBuffer, beatIndex, time, BPM_LONG_PERIOD_MS);
    last_bpm_time = time;
  }

  execute_buzz(time);
}
 
