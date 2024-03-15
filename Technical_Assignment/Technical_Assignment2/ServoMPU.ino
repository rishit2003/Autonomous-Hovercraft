#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

#define LED_PIN 13 // Assuming built-in LED pin
#define LED_D3 11 
#define SERVO_PIN 9 // Assuming servo is connected to pin 9
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  servo.attach(SERVO_PIN);
}
float yaw;
float roll;
float pitch;
int16_t rawGyroX, rawGyroY, rawGyroZ;
int16_t accX, accY, accZ;
int16_t rawAccelX, rawAccelY, rawAccelZ;

void loop() {
  float ;
  // Read raw gyroscope data
  
  // Define variables to store accelerometer data
  
  // Read accelerometer offsets
int16_t accelOffsetX = mpu.getXAccelOffset();
int16_t accelOffsetY = mpu.getYAccelOffset();
int16_t accelOffsetZ = mpu.getZAccelOffset();

  mpu.getRotation(&rawGyroX, &rawGyroY, &rawGyroZ);

  // Read raw accelerometer data

mpu.getAcceleration(&rawAccelX, &rawAccelY, &rawAccelZ);

// Apply offsets to raw data to get calibrated accelerometer data
accX = rawAccelX - accelOffsetX;
accY = rawAccelY - accelOffsetY;
accZ = rawAccelZ - accelOffsetZ;

  // Calculate yaw angle based on gyroscope data
  float gyroScale = mpu.get_gyro_resolution();
  float gyroX_deg_per_s = rawGyroX * gyroScale;
  float gyroY_deg_per_s = rawGyroY * gyroScale;
  float gyroZ_deg_per_s = rawGyroZ * gyroScale;

  // Now, accX, accY, and accZ variables should contain the calibrated accelerometer data

   float accelScale = mpu.get_acce_resolution();
  float accX_G = accX * accelScale;
  float accY_G = accY * accelScale;
  float accZ_G = accZ * accelScale;
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  yaw += gyroZ_deg_per_s; // Integration over time (0.01 is the time interval in seconds)

  // Control servo based on yaw
  if (yaw > 85) yaw = 100;
  else if (yaw < -85) yaw = -100;
  servo.write(map(yaw, -85, 85, 0, 170));

  // Control LED based on yaw range
  if (yaw > 85 || yaw < -85) digitalWrite(LED_PIN, HIGH);
  else digitalWrite(LED_PIN, LOW);

  // Control D3 brightness based on X-axis acceleration
  float accelMagnitude = abs(accX) / 16384.0; // Conversion factor from raw data to g
  int brightness = map(accelMagnitude, 0.08, 1.08, 255, 0);
  analogWrite(LED_D3, brightness); // Assuming D3 is connected to PWM pin 11

  // Output values on serial monitor
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("Yaw: "); Serial.print(yaw);
  Serial.println("AccX: "); Serial.println(accX);
  Serial.print(" AccY: "); Serial.println(accY);
  Serial.print(" AccZ: "); Serial.println(accZ);
  // Serial.print(" GyroX: "); Serial.print(gyroX_deg_per_s);
  // Serial.print(" GyroY: "); Serial.print(gyroY_deg_per_s);
  Serial.print(" GyroZ: "); Serial.println(gyroZ_deg_per_s);
  
  delay(100); // Adjust as necessary for your application
}
