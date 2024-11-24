#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> // Include the Servo library
#include <LiquidCrystal_I2C.h> // Include the LCD library

// Pin definitions for LEDs
int L1 = 13;
int L2 = 12;
int L3 = 11;
int L4 = 10;
int L5 = 9;
int L6 = 8;
int L7 = 7;
int L8 = 6;
int L9 = 5;
int L10 = 4;
int L11 = 3;
int L12 = 2;
int L13 = A0;
int L14 = A1;
int L15 = A2;

// Servo pin definition
int servoPin = 9; 

Adafruit_MPU6050 mpu;
Servo myServo; // Create a servo object
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD I2C address (usually 0x27) and size (16x2)

void setup(void) {
  Serial.begin(115200);

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer and gyro ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  // Set pinMode for LEDs
  for (int pin = L1; pin <= L15; pin++) {
    pinMode(pin, OUTPUT);
  }

  // Attach the servo to the pin
  myServo.attach(servoPin);

  // Initialize the LCD
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 Status:");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("Ready");
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Update LEDs based on Y-axis acceleration
  updateLEDs(a.acceleration.y);

  // Control the servo based on Y-axis acceleration
  controlServo(a.acceleration.y);

  // Update LCD display based on Y-axis acceleration
  updateLCD(a.acceleration.y);
}

// Update LED states based on Y-axis acceleration
void updateLEDs(float accelY) {
  for (int i = 1; i <= 15; i++) {
    float thresholdLow = (i - 1) * 0.5;
    float thresholdHigh = i * 0.5;

    if (accelY > -thresholdHigh && accelY <= -thresholdLow) {
      digitalWrite(i + L1 - 1, HIGH);  // LEDs from L1 to L15
    } else {
      digitalWrite(i + L1 - 1, LOW);
    }
  }
}

// Control the servo based on Y-axis acceleration
void controlServo(float accelY) {
  static int lastServoAngle = 90;
  int servoAngle;

  // Smooth servo movement for better control
  if (accelY == 0) {
    servoAngle = 90; // Center position
  } else if (accelY < 0) {
    servoAngle = 90 + (accelY * -10); // Tilt to the left (clockwise)
  } else {
    servoAngle = 90 - (accelY * 10); // Tilt to the right (anticlockwise)
  }

  // Constrain servo angle to valid range [0, 180]
  servoAngle = constrain(servoAngle, 0, 180);

  // Smooth transition for servo movement
  int smoothAngle = lastServoAngle + (servoAngle - lastServoAngle) / 10;  // Smoothing effect
  myServo.write(smoothAngle);
  lastServoAngle = smoothAngle;
}

// Update the LCD display based on Y-axis acceleration
void updateLCD(float accelY) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Position:");

  lcd.setCursor(0, 1);
  if (accelY == 0) {
    lcd.print("Center");
  } else if (accelY < 0) {
    lcd.print("Left Side");
  } else {
    lcd.print("Right Side");
  }
}
