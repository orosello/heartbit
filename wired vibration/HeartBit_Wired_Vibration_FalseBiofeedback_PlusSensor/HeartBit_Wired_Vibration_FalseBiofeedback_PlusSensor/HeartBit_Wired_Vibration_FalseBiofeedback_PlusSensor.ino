#include <Wire.h>
#include <SD.h>
#include <cmath>

//Definitions
enum {
  HR_RX = 6,
  MOTOR_PIN = 13,

  BUFFER_SIZE = 200, // How many beat timestamps we store at once

  SECOND_MS = 1000,
  MINUTE_SEC = 60,
  MINUTE_MS = MINUTE_SEC * SECOND_MS,

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

long time_offset = 0;

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

const int TIMESTAMPS[] = {0,790,1604,2459,3339,4250,5144,6035,6909,
7751,8573,9372,10162,10958,11795,12623,13470,14291,15079,15851,16620,
17407,18242,19079,19936,20772,21635,22484,23293,24056,24803,25545,26331,
27178,28062,28944,29809,30657,31464,32284,33100,33848,34546,35199,35845,
36498,37174,37880,38638,39433,40225,41007,41755,42479,43188,43884,44556,45241,
45964,46765,47601,48397,49236,50095,50958,51781,52634,53472,54279,54622,55041,
55796,56577,57379,58179,58959,59708,60467,61231,61985,62729,63457,64174,64895,
65618,66334,67064,67810,68558,69314,70060,70799,71522,72246,72982,73731,74470,
75215,75907,76566,77206,77824,78431,79039,79654,80302,81032,81854,82666,83449,
84262,85041,85780,86521,87280,88012,88750,89489,90229,90933,91592,92254,92924,
93597,94321,95093,95912,96781,97640,98509,99359,100173,100938,101697,102455,
103254,104043,104877,105814,106767,107655,108578,109483,110310,111094,111857,
112614,113397,114210,114983,115778,116603,117477,118315,119152,119971,120776,
121562,122327,123066,123769,124445,125109,125750,126382,127007,127637,128297,
128978,129657,130058,130336,131037,131732,132394,133039,133665,134274,134863,
135439,135715,135993,136547,137087,137439,137941,138659,139178,139705,140244,
140816,141377,141935,142486,143032,144106,144637,145162,145679,146089,146688,
147183,147452,148161,148651,149134,149614,150095,150577,151056,151530,152008,
152485,152964,153444,153926,154408,154891,155370,155849,156331,156818,157344,
157965,158596,159291,160005,160716,161411,162108,162769,163383,163991,164611,
165283,165971,166639,167304,167953,168576,169175,169754,170313,170858,171392,
171924,172457,173003,173542,174077,174608,175132,175655,176178,176732,177666,
178461,179277,180083,180939,181842,182739,183665,184526,185309,186053,186723,
187346,187953,188550,189209,189888,190597,191385,192140,192865,193556,194199,
194816,195405,195973,196533,197090,197660,198238,198845,199503,200388,201243,
202191,203139,204038,204918,205786,206577,207392,208177,209042,209961,210880,
211770,212654,213528,214359,215133,215860,216524,217153,217757,218344,218924,
219495,220043,220585,221117,221643,222160,222672,223177,223682,224178,224670,
225164,225650,226153,226667,227202,227794,228428,229071,229673,230278,230858,
231422,231972,232515,233043,233568,234091,234606,235020,235632,236145,236656,
237169,237679,238194,238702,239206,239709,240213,240716,241222,241723,242220,
242719,243217,243718,244217,244737,245258,245777,246316,246852,247390,247929,
248471,249000,249579,250264,250572,250988,252393,253043,253690,254373,255125,
255949,256805,257619,258396,259137,259911,260692,261525,262405,263323,264214,
265136,266024,266802,267521,268185,268802,269402,269978,270362,271106,271655,
272197,272731,273257,273777,274291,274804,275317,275825,276334,278358};

const unsigned int TIMESTAMP_COUNT = sizeof(TIMESTAMPS) / sizeof(TIMESTAMPS[0]);
const int LAST_TIMESTAMP = TIMESTAMPS[TIMESTAMP_COUNT - 1];
unsigned int timestamp_index = 0;

//////// Time functions

// Returns current time since program start
long time_since_start(void){
  const long now = millis();
  return now - time_offset;
}

// Returns current time since program start, within recorded timestamp bounds
long wrapped_time_since_start(void){
  return time_since_start() % LAST_TIMESTAMP;
}

//////// Heartrate calculations

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

/////////// Buzz functions

void execute_buzz(long time){
  time = time % LAST_TIMESTAMP; // Make sure time is within expected bounds
  const long time_since_last_buzz = time < buzz_last_updated ? 
                                    ((time + LAST_TIMESTAMP) - buzz_last_updated) :
                                    time - buzz_last_updated;
  switch(buzz_state) {
    case BUZZ_STAGE_FIRST_BUZZ:
      if (time_since_last_buzz > FIRST_BUZZ_MS) {
        digitalWrite(MOTOR_PIN, LOW);
        buzz_last_updated = time;
        buzz_state = BUZZ_STAGE_FIRST_PAUSE;
      }       
      break;
    case BUZZ_STAGE_FIRST_PAUSE:
      if (time_since_last_buzz > FIRST_BUZZ_PAUSE_MS) {
        digitalWrite(MOTOR_PIN, HIGH);
        buzz_last_updated = time;
        buzz_state = BUZZ_STAGE_SECOND_BUZZ;
      }
      break;
    case BUZZ_STAGE_SECOND_BUZZ:
      if (time_since_last_buzz > SECOND_BUZZ_MS) {
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

//////////// Main functions

void setup() {
  pinMode (HR_RX, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  Serial.begin(9600);
  while(!Serial);

  // Initialize buffer for recording beats
  for(int i = 0; i < BUFFER_SIZE; i++) {
    beatBuffer[i] = 0;
    beatBuffer[i] = 0;
  }
  // Wait until a heart beat is detected
  while (!digitalRead(HR_RX));

  // Mark this time as 0 ms
  const long now = millis();
  time_offset = now;
}

void loop() {
  // Detect heartbeats, calculate statistics, log to serial
  const long time = time_since_start();
  const byte sample = digitalRead(HR_RX); // Store signal input
  if (sample && last_sample != sample && time - last_beat_time > MIN_TIME_BETWEEN_BEATS_MS) {
    last_beat_time = time;
    const float time_in_sec = ((float) time) / SECOND_MS;
    Serial.println("P," + String(time_in_sec) + "," + String(current_bpm_short) + "," + String(current_bpm_long) + "," + String(current_hrv));
    beatIndex = (beatIndex + 1) % BUFFER_SIZE;
    beatBuffer[beatIndex] = time;
  }
  last_sample = sample; //Store last signal received

  if (time - last_bpm_time >= BPM_REFRESH_PERIOD) {
    current_bpm_long = calculate_bpm(beatBuffer, beatIndex, time, BPM_LONG_PERIOD_MS);
    current_bpm_short = calculate_bpm(beatBuffer, beatIndex, time, BPM_SHORT_PERIOD_MS);
    current_hrv = calculate_hrv_time(beatBuffer, beatIndex, time, BPM_LONG_PERIOD_MS);
    last_bpm_time = time;
  }

  ///// Play back recorded beats based on timestamps
  
  // Get current time within looping window
  const long wrapped_time = wrapped_time_since_start();
  // Update buzzing as needed
  execute_buzz(wrapped_time);
  // Check if we've reached the next timestamp or not
  if (wrapped_time < TIMESTAMPS[timestamp_index]) {
    return;
  }
  // If we're back at the first timestamp, wait for time to wrap around to 0
  if (timestamp_index == 0 && wrapped_time >= TIMESTAMPS[TIMESTAMP_COUNT - 2]) {
    return;
  }
  // Display clock time of recorded beat being played
  const int seconds = (wrapped_time / SECOND_MS) % MINUTE_SEC;
  const int minutes = wrapped_time / MINUTE_MS;
  // Display this pre-recorded heartbeat's time since the program started
  const int current_time_ms = (time - wrapped_time) + TIMESTAMPS[timestamp_index];
  const float current_time_s = ((float) current_time_ms) / SECOND_MS;
  Serial.printf("R,%02d:%02d,%.3f\n", minutes, seconds, current_time_s);
  
  // Time for next beat in list (excluding very last timestamp)
  timestamp_index = (timestamp_index + 1) % (TIMESTAMP_COUNT - 1);
  // Kick off first phase of buzz
  buzz_state = BUZZ_STAGE_FIRST_BUZZ;
  buzz_last_updated = wrapped_time;
  digitalWrite(MOTOR_PIN, HIGH);
}
 
