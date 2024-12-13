#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_IR    A0

// Event interval parameters
#define _INTERVAL_DIST    20 // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO   20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL  100  // serial interval (unit: ms)

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.7    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter

// Servo adjustment - Set _DUTY_MAX, _NEU, _MIN with your own numbers
#define _DUTY_MAX 2400 // 2000
#define _DUTY_NEU 1600 // 1500 // NEU를 줄이면 레일이 올라간다
#define _DUTY_MIN 600 // 1000

#define _SERVO_ANGLE_DIFF  80  // Replace with |D - E| degree
#define _SERVO_SPEED       1000  // servo speed

// Target Distance
#define _DIST_TARGET    155 // Center of the rail (unit:mm) 수정하지 말 것

// PID parameters
#define _KP 3  // proportional gain
#define _KD 230  // derivative gain
// #define _KI 0.0   // integral gain

const int numSamples = 11; // 샘플 크기
float samples[numSamples];  // 샘플 배열
float tempSamples[numSamples];
int sampleIndex = 0;

// global variables

Servo myservo;      // Servo instance

float dist_ema;     // filtered distance

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial; // event triggered?

// Servo speed control
int duty_chg_per_interval;     // maximum duty difference per interval

// Servo position
float duty_target, duty_current;

// PID variables
float error_current, error_prev;
float pterm, dterm, iterm;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  
  myservo.attach(PIN_SERVO);
  duty_target = duty_current = _DUTY_NEU;
  myservo.writeMicroseconds(duty_current);

  // convert angle speed into duty change per interval
  duty_chg_per_interval = 
    (float)(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0); 

  // initialize serial port
  Serial.begin(1000000);

  dist_ema = volt_to_distance(ir_sensor_filtered(99, 0.5, 0));

  for (int i = 0; i < numSamples; i++){
    samples[i] = dist_ema;
  }
  
  error_current = error_prev = _DIST_TARGET - dist_ema;
}
  
void loop() {
  unsigned long time_current = millis();
  
  // wait until next event time
  if (time_current >= (last_sampling_time_dist + _INTERVAL_DIST)) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_current >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_current >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  if (event_dist) {
    float dist_filtered; // unit: mm
    float control;
    
    event_dist = false;

    // get a distance reading from the distance sensor
    dist_filtered = volt_to_distance(ir_sensor_filtered(99, 0.5, 0));
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

    // 1. 새로운 센서 값 읽기
    float newSample = dist_ema;
  
    // 2. 샘플 배열 업데이트
    samples[sampleIndex] = newSample;
    sampleIndex = (sampleIndex + 1) % numSamples; // 순환 인덱스
  
    // 3. 중간값 필터 적용
//    float filteredValue = medianFilter(samples, numSamples);
//    dist_ema = filteredValue;

    for (int i = 0; i < numSamples; i++){
      tempSamples[i] = samples[i];
    }

    for (int i = 0; i < numSamples - 1; i++){
      for (int j = 0; j < numSamples - 1; j++){
        if (tempSamples[j] > tempSamples[j+1]){
          float temp = tempSamples[j];
          tempSamples[j] = tempSamples[j+1];
          tempSamples[j+1] = temp;
        }
      }
    }
    dist_ema = (tempSamples[numSamples / 2 - 1] + tempSamples[numSamples / 2] + tempSamples[numSamples / 2 + 1]) / 3;

    // PID control logic
    error_current = _DIST_TARGET - dist_ema;
    
    pterm = _KP * error_current;
    dterm = _KD * (error_current - error_prev);

    control = pterm + dterm /* + iterm*/;
    error_prev = error_current;

    duty_target = _DUTY_NEU + control;

    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN)
      duty_target = _DUTY_MIN; // lower limit
    if (duty_target > _DUTY_MAX)
      duty_target = _DUTY_MAX; // upper limit
  }
 
  if (event_servo) {
    event_servo = false;

    // adjust duty_current toward duty_target by duty_chg_per_interval
    if (duty_target > duty_current) {
      duty_current += duty_chg_per_interval;
      if (duty_current > duty_target)
        duty_current = duty_target;
    } else {
      duty_current -= duty_chg_per_interval;
      if (duty_current < duty_target)
        duty_current = duty_target;
    }
    
    // update servo position
    myservo.writeMicroseconds(duty_current);    
  }
 
  if (event_serial) {
    event_serial = false;
  
    // use for debugging
    if (0) {
      Serial.print(",ERROR:"); Serial.print(error_current); 
      Serial.print(",pterm:"); Serial.print(pterm);
      Serial.print(",dterm:"); Serial.print(dterm);
      Serial.print(",duty_target:"); Serial.print(duty_target);
      Serial.print(",duty_current:"); Serial.print(duty_current);
    }
    Serial.print("MIN:0,MAX:300,TARGET:155,TG_LO:128,TG_HI:182,DIST:"); 
    Serial.println(dist_ema);
  }
}

//float medianFilter(float* samples, int numSamples) {
//  // 1. 배열 복사
//  float sorted[numSamples];
//  memcpy(sorted, samples, numSamples * sizeof(int));
//
//  Serial.print("samples: "); 
//  for (int i = 0; i < numSamples; i++){
//    Serial.println(samples[i]);
//  }
//
//  // 2. 정렬
//  for (int i = 0; i < numSamples - 1; i++) {
//    for (int j = 0; j < numSamples - i - 1; j++) {
//      if (sorted[j] > sorted[j + 1]) {
//        float temp = sorted[j];
//        sorted[j] = sorted[j + 1];
//        sorted[j + 1] = temp;
//      }
//    }
//  }
//
//  // 3. 중앙값 반환
//  return sorted[numSamples / 2];
//}

float volt_to_distance(int a_value)
{
  // Replace next line into your own equation
  // return (6762.0 / (a_value - 9) - 4.0) * 10.0; 
  return 628 + -2.12 * a_value + 1.79E-03 * a_value * a_value;
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
  // n: number of measurement repetition
  // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)
  // verbose: 0 - normal operation, 1 - observing the internal procedures, and 2 - calculating elapsed time.
  // Example 1: ir_sensor_filtered(n, 0.5, 0) => return the median value among the n measured samples.
  // Example 2: ir_sensor_filtered(n, 0.0, 0) => return the smallest value among the n measured samples.
  // Example 3: ir_sensor_filtered(n, 1.0, 0) => return the largest value among the n measured samples.

  // The output of Sharp infrared sensor includes lots of spiky noise.
  // To eliminate such a spike, ir_sensor_filtered() performs the following two steps:
  // Step 1. Repeat measurement n times and collect n * position smallest samples, where 0 <= postion <= 1.
  // Step 2. Return the position'th sample after sorting the collected samples.

  // returns 0, if any error occurs

  unsigned int *ir_val, ret_val;
  unsigned int start_time;
 
  if (verbose >= 2)
    start_time = millis(); 

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;
    
  if (position == 1.0)
    position = 0.999;

  if (verbose == 1) {
    Serial.print("n: "); Serial.print(n);
    Serial.print(", position: "); Serial.print(position); 
    Serial.print(", ret_idx: ");  Serial.println((unsigned int)(n * position)); 
  }

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  if (verbose == 1)
    Serial.print("IR:");
  
  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    if (verbose == 1) {
        Serial.print(" ");
        Serial.print(ir_val[i]);
    }
  }

  if (verbose == 1)
    Serial.print  ("  => ");

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];

  if (verbose == 1) {
    for (int i = 0; i < n; i++) {
        Serial.print(" ");
        Serial.print(ir_val[i]);
    }
    Serial.print(" :: ");
    Serial.println(ret_val);
  }
  free(ir_val);

  if (verbose >= 2) {
    Serial.print("Elapsed time: "); Serial.print(millis() - start_time); Serial.println("ms");
  }

  return ret_val;
}
