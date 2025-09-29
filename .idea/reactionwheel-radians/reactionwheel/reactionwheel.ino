// === Motor Pins ===
const int forward = 5;
const int backward = 6;

// === Encoder ===
const int THRESHOLD = 150;
int32_t prev_steps = 0;
float start_time = millis();
int sensor_val_1;
int sensor_val_2;
int differance_steps;
bool state = false;
int32_t steps = 0;

// === Gyro (MPU-6050) ===
#include <Wire.h>
#define MPU_ADDRESS 0x68
float yawAngle = 0; // now in radians
float filteredYawAngle = 0; // in radians
float GYRO_ERROR = 0; // in raw units
const int angleBufferSize = 5;
float angleBuffer[angleBufferSize];
int angleIndex = 0;

// === Timing ===
long timeCur = 0;
long timePrev = 0;

// === PID Controllers ===
float desiredYawAngle = 0.0; // radians

// Outer PI (Yaw Angle -> Desired Velocity)
float kp_outer = 14000; // already tuned for radians
float ki_outer = 0; 
float angle_integral = 0;

// Inner PID (Velocity -> PWM)
float kp_inner = 0.063928;
float ki_inner = 22.2279;
float kd_inner = 0.000015777;
float velocity_integral = 0;
float previous_velocity_error = 0;

void setup() {
  pinMode(forward, OUTPUT);
  pinMode(backward, OUTPUT);
  digitalWrite(forward, HIGH);
  digitalWrite(backward, HIGH);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);

  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);

  calibrateMPU();

  timeCur = millis();
  timePrev = timeCur;
}

// === Encoder Functions ===
int32_t countSteps(int sensor_val_1, int sensor_val_2, int32_t steps) {
  if (sensor_val_2 > THRESHOLD) steps++;
  else steps--;
  return steps;
}

int32_t counterRisingEdge(int sensor_val_1, int sensor_val_2, int32_t steps) {
  if (sensor_val_1 > THRESHOLD && !state) {
    state = true;
    return countSteps(sensor_val_1, sensor_val_2, steps);
  }
  if (sensor_val_1 <= THRESHOLD) state = false;
  return steps;
}

int32_t differanceSteps(int32_t prev_steps, int32_t current_steps) {
  return current_steps - prev_steps;
}

float calculateVelocity(int differance_steps, float start_time) {
  float differance_time = (millis() - start_time) / 60000.0;
  return (differance_steps / differance_time) / 9.0;
}

float getVelocity(float &start_time) {
  float velocity = 0.0;
  if (millis() >= start_time + 1000.0) {
    differance_steps = differanceSteps(prev_steps, steps);
    velocity = calculateVelocity(differance_steps, start_time);
    start_time = millis();
    prev_steps = steps;
  }
  return velocity;
}

// === Gyro Functions ===
int16_t readMPU() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 2, true);
  return Wire.read() << 8 | Wire.read();
}

void calibrateMPU() {
  Serial.println("Calibrating... Keep still.");
  GYRO_ERROR = 0;
  for (int i = 0; i < 200; i++) {
    GYRO_ERROR += readMPU();
    delay(20);
  }
  GYRO_ERROR /= 200.0;
  Serial.print("Gyro Error: ");
  Serial.println(GYRO_ERROR);
}

void updateYaw() {
  timePrev = timeCur;
  timeCur = millis();
  float yawAngularSpeed = ((float)readMPU() - GYRO_ERROR) / 32.8 * PI / 180.0; // rad/s
  yawAngle += yawAngularSpeed * (timeCur - timePrev) / 1000.0; // rad

  // Wrap angle to [-π, π]
  while (yawAngle < -PI) yawAngle += 2 * PI;
  while (yawAngle > PI) yawAngle -= 2 * PI;

  angleBuffer[angleIndex] = yawAngle;
  angleIndex = (angleIndex + 1) % angleBufferSize;

  float total = 0;
  for (int i = 0; i < angleBufferSize; i++) total += angleBuffer[i];
  filteredYawAngle = total / angleBufferSize;
}

float getFilteredYawAngle() {
  return filteredYawAngle; // radians
}

// === Motor Control ===
void applyMotorPWM(float pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(forward, pwm);
    digitalWrite(backward, HIGH);
  } else {
    digitalWrite(forward, HIGH);
    analogWrite(backward, -pwm);
  }
}

void loop() {
  updateYaw();
  sensor_val_1 = analogRead(A5);
  sensor_val_2 = analogRead(A6);
  steps = counterRisingEdge(sensor_val_1, sensor_val_2, steps);

  float current_velocity = getVelocity(start_time);

  // Outer PI Loop (Yaw angle to desired velocity)
  float angle_error = desiredYawAngle - getFilteredYawAngle(); // rad
  angle_integral += angle_error * 0.01;
  float desired_velocity = kp_outer * angle_error + ki_outer * angle_integral;

  // Inner PID Loop (Velocity to PWM)
  float velocity_error = desired_velocity - current_velocity;
  velocity_integral += velocity_error * 0.01;
  float derivative = (velocity_error - previous_velocity_error) / 0.01;
  previous_velocity_error = velocity_error;
  float control_signal = kp_inner * velocity_error + ki_inner * velocity_integral + kd_inner * derivative;

  applyMotorPWM(control_signal);

  // Print in degrees for human readability
  Serial.print("Yaw: "); Serial.print(getFilteredYawAngle() * 180.0 / PI);
  Serial.print("° | Desired Vel: "); Serial.print(desired_velocity);
  Serial.print(" | Actual Vel: "); Serial.print(current_velocity);
  Serial.print(" | PWM: "); Serial.println(control_signal);

  delay(10);
}
