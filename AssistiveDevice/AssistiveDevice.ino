#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Servo.h>

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */

SimpleKalmanFilter simpleKalmanFilter_x(22.5, 45, 0.1);
SimpleKalmanFilter simpleKalmanFilter_y(22.5, 45, 0.1);
SimpleKalmanFilter simpleKalmanFilter_z(22.5, 45, 0.1);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 50;
long refresh_time;

Servo servo_x_axis;
Servo servo_y_axis;

int init_x = 45, init_y = 45, init_z = 45;
int mapped_x = 0, mapped_y = 0, mapped_z = 0;


float x, y, z;
const int MPU_ADDR = 0x68;                                  // I2C address of the MPU6050. If AD0 pin is set to HIGH, the I2C address will be 0x69
int16_t accelerometer_x, accelerometer_y, accelerometer_z;  // Variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z;                             // Variables for gyro raw data
int16_t temperature;                                        // Variable for temperature

void setup() {
  Serial.begin(9600);
  //Setup Servo
  servo_x_axis.attach(6);
  servo_y_axis.attach(7);
  //initiliazlize postion of servo
  servo_x_axis.write(0);
  servo_y_axis.write(0);
  delay(1000);
  servo_x_axis.write(90);
  servo_y_axis.write(90);
  delay(1000);
  servo_x_axis.write(init_x);
  servo_y_axis.write(init_y);
  delay(100);
  // put your setup code here, to run once:
  setup_gyro();
}

void loop() {
  // put your main code here, to run repeatedly:
  fetch_gyro();
  // Clamp values to a specific range
  if (x > 100) {
    x = 100;
  }
  if (x < -100) {
    x = -100;
  }
  if (y > 100) {
    y = 100;
  }
  if (y < -100) {
    y = -100;
  }
  if (z > 100) {
    z = 100;
  }
  if (z < -100) {
    z = -100;
  }
  
  mapped_x = map(x, -100, 100, 0, 90);
  mapped_y = map(y, -100, 100, 0, 90);
  mapped_z = map(z, -100, 100, 0, 90);

 // add a noise to the reference value and use as the measured value
  float calculated_x = mapped_x + random(-100, 100) / 100.0;
  float calculated_y = mapped_y + random(-100, 100) / 100.0;
  float calculated_z = mapped_z + random(-100, 100) / 100.0;

  // calculate the estimated value with Kalman Filter
  float filtered_x = simpleKalmanFilter_x.updateEstimate(calculated_x);
  float filtered_y = simpleKalmanFilter_y.updateEstimate(calculated_y);
  float filtered_z = simpleKalmanFilter_z.updateEstimate(calculated_z);

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization

  if (millis() > refresh_time) {
    Serial.print("X:");
    Serial.print(mapped_x);
    Serial.print(",");
    
    Serial.print("EX:");
    Serial.print(filtered_x, 4);
    Serial.print(",");

    Serial.print("Y:");
    Serial.print(mapped_y);
    Serial.print(",");

    Serial.print("EY:");
    Serial.print(filtered_y, 4);
    Serial.print(",");

    Serial.print("Z:");
    Serial.print(mapped_z);
    Serial.print(",");

    Serial.print("EZ:");
    Serial.println(filtered_z, 4);

    refresh_time = millis() + SERIAL_REFRESH_TIME;

  }
  servo_x_axis.write(filtered_x);
  servo_y_axis.write(filtered_y);
}




//Setting up initialize
void setup_gyro() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to 0 (wakes up the MPU6050)
  Wire.endTransmission(true);
}

//Fetching Values From Sensor
void fetch_gyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7 * 2, true);

  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  x = accelerometer_x / 100.0;
  y = accelerometer_y / 100.0;
  z = accelerometer_z / 100.0;
}



  // Print out data
  // Serial.print("x:");
  // Serial.print(x);
  // Serial.print(",");
  // Serial.print("y:");
  // Serial.print(y);
  // Serial.print(",");
  // Serial.print("z:");
  // Serial.println(z);
  // Serial.print("X : ");
  // Serial.print(x);
  // Serial.print("\t Y :");
  // Serial.print(y);
  // Serial.print("\t Z : ");
  // Serial.println(z);

  // Serial.print("\t Value-1 : ");
  // Serial.print(mapped_x);
  // Serial.print("\t Value-2 : ");
  // Serial.println(mapped_y);

  // Serial.print("Ax");
  // Serial.print(accelerometer_x);
  // Serial.print("\t Ay :");
  // Serial.print(accelerometer_y);
  // Serial.print("\t Az :");
  // Serial.println(accelerometer_z);

  // Serial.print("Gx");
  // Serial.print(gyro_x);
  // Serial.print("\t Gy :");
  // Serial.print(gyro_y);
  // Serial.print("\t Gz :");
  // Serial.println(gyro_z);