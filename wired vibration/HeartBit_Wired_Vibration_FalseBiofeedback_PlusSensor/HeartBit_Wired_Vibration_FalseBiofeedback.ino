#include <Wire.h>
#include <SD.h>
#include <cmath>

//Definitions
enum {
  MOTOR_PIN = 13,
  
  SECOND_MS = 1000,
  MINUTE_SEC = 60,
  MINUTE_MS = MINUTE_SEC * SECOND_MS,
  
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
buzz_state_t buzz_state;
long buzz_last_updated = 0;

const int timestamps[] = {0,790,1604,2459,3339,4250,5144,6035,6909,
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


const unsigned int timestamp_count = sizeof(timestamps) / sizeof(timestamps[0]);
const int last_timestamp = timestamps[timestamp_count - 1];
unsigned int timestamp_index = 0;

long time_since_start(void){
  const long now = millis();
  return (now - time_offset) % last_timestamp;
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  pinMode(MOTOR_PIN, OUTPUT);
  
  const long now = millis();
  time_offset = now;
}

void execute_buzz(const long time){
  const long time_since_last_buzz = time < buzz_last_updated ? 
                                    ((time + last_timestamp) - buzz_last_updated) :
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

void loop() {
  // Get current time within looping window
  const long time = time_since_start();
  // Update buzzing as needed
  execute_buzz(time);
  // Check if we've passed the next timestamp or not
  if (time < timestamps[timestamp_index]) {
    return;
  }
  // If we're back at the first timestamp, wait for time to wrap around to 0
  if (timestamp_index == 0 && time >= timestamps[timestamp_count - 2]) {
    return;
  }
  const int seconds = (time / SECOND_MS) % MINUTE_SEC;
  const int minutes = time / MINUTE_MS;
  Serial.printf("%02d:%02d\n", minutes, seconds);
  
  // Time for next beat in list (excluding very last timestamp)
  timestamp_index = (timestamp_index + 1) % (timestamp_count - 1);
  // Kick off first phase of buzz
  buzz_state = BUZZ_STAGE_FIRST_BUZZ;
  buzz_last_updated = time;
  digitalWrite(MOTOR_PIN, HIGH);
}
 