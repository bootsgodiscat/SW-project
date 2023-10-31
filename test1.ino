#include <Servo.h>

#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

#define _EMA_ALPHA 0.4    // EMA weight of new sample (range: 0 to 1)
#define _TARGET_LOW  180.0
#define _TARGET_HIGH 360.0  // Adjusted the high target to 360 degrees

Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(map(_TARGET_HIGH, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX)); // Set initial servo position
  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;
    digitalWrite(PIN_LED, 1);
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;
    digitalWrite(PIN_LED, 1);
  } else {
    digitalWrite(PIN_LED, 0);
  }

  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  int servo_position = map(dist_ema, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX);

  myservo.writeMicroseconds(servo_position);

  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",Low:");   Serial.print(_TARGET_LOW);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",Servo:"); Serial.print(servo_position);
  Serial.print(",High:");  Serial.print(_TARGET_HIGH);
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
