#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
#include <math.h>  // Pentru funcțiile trigonometrice

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // I2C addr 0x40
VL53L0X sensor;                                            // I2C addr 0x29

#define SERVOMIN  100
#define SERVOMAX  550

void setServoAngle(uint8_t servo, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulse);
}

int middle = 0;
int offsetrange = 10;

void setup() {
  Serial.begin(115200);
  Wire.begin(22, 23);  // Pinii I2C ESP32

  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz pentru servomotoare

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("VL53L0X nu a fost detectat!");
    while (1);
  }
  sensor.startContinuous();

  setServoAngle(0, offsetrange);
  setServoAngle(1, 180 - offsetrange);
  setServoAngle(3, middle);
  delay(1000);
}

void loop() {
  while (middle != 180 - offsetrange) {
    for (int thetaDeg = offsetrange; thetaDeg <= 180 - offsetrange; thetaDeg += 5) {
      setServoAngle(0, thetaDeg);
      setServoAngle(1, 180 - thetaDeg);

      uint16_t distance = sensor.readRangeContinuousMillimeters();
      if (sensor.timeoutOccurred()) {
        Serial.println("Timeout");
        distance = 0;
      }

      float d = distance / 10.0; // Convertim mm in cm
      float delta = 19 - d;
      float theta = radians(thetaDeg);
      float phi = radians(middle);

      float x = delta * cos(phi) * sin(theta);
      float y = delta * sin(phi);
      float z = delta * cos(phi) * cos(theta);


      Serial.print(x, 2);
      Serial.print(",");
      Serial.print(y, 2);
      Serial.print(",");
      Serial.println(z, 2);

      delay(300);
    }

    setServoAngle(3, middle + 5);
    middle += 5;

    for (int thetaDeg = 180 - offsetrange; thetaDeg >= offsetrange; thetaDeg -= 5) {
      setServoAngle(0, thetaDeg);
      setServoAngle(1, 180 - thetaDeg);

      uint16_t distance = sensor.readRangeContinuousMillimeters();
      if (sensor.timeoutOccurred()) {
        Serial.println("Timeout");
        distance = 0;
      }

      float d = distance / 10.0; // mm -> cm
      delta = 19 - d;
      float theta = radians(thetaDeg);
      float phi = radians(middle);

      float x = delta * cos(phi) * sin(theta);
      float y = delta * sin(phi);
      float z = delta * cos(phi) * cos(theta);



      Serial.print(x, 2);
      Serial.print(",");
      Serial.print(y, 2);
      Serial.print(",");
      Serial.println(z, 2);

      delay(300);
    }

    setServoAngle(3, middle + 5);
    middle += 5;
  }
  Serial.println("END");
  while (true);
}
