#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

#define LED_PIN 13 // Assuming built-in LED pin
#define SERVO_PIN 9 // Assuming servo is connected to pin 9
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  servo.attach(SERVO_PIN);
}

void loop() {
  
  float roll, pitch, yaw;
  
  // Define variables to store accelerometer data
int16_t accX, accY, accZ;

// Read accelerometer offsets
int16_t accelOffsetX = mpu.getXAccelOffset();
int16_t accelOffsetY = mpu.getYAccelOffset();
int16_t accelOffsetZ = mpu.getZAccelOffset();

// Read raw accelerometer data
int16_t rawAccelX, rawAccelY, rawAccelZ;
mpu.getAcceleration(&rawAccelX, &rawAccelY, &rawAccelZ);

// Apply offsets to raw data to get calibrated accelerometer data
accX = rawAccelX - accelOffsetX;
accY = rawAccelY - accelOffsetY;
accZ = rawAccelZ - accelOffsetZ;

// Now, accX, accY, and accZ variables should contain the calibrated accelerometer data

   float accelScale = mpu.get_acce_resolution();
  float accX_G = accX * accelScale;
  float accY_G = accY * accelScale;
  float accZ_G = accZ * accelScale;
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  yaw = atan2(sqrt(accX * accX + accZ * accZ), accY) * RAD_TO_DEG;

  // Control servo based on yaw
  if (yaw > 90) yaw = 90;
  else if (yaw < -90) yaw = -90;
  servo.write(map(yaw, -90, 90, 0, 180));

  // Control LED based on yaw range
  if (yaw > 90 || yaw < -90) digitalWrite(LED_PIN, HIGH);
  else digitalWrite(LED_PIN, LOW);

  // Control D3 brightness based on X-axis acceleration
  float accelMagnitude = abs(accX) / 16384.0; // Conversion factor from raw data to g
  int brightness = map(accelMagnitude, 0.01, 1.00, 0, 255);
  analogWrite(3, brightness); // Assuming D3 is connected to PWM pin 3

  // Output values on serial monitor
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: "); Serial.print(yaw);
  Serial.print(" AccX: "); Serial.println(accX);
  
  delay(1000); // Output once per second
}
