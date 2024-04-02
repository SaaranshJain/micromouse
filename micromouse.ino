// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// #include <PID_v1.h>

#define IN1 D4
#define IN2 D5
#define IN3 D6
#define IN4 D7

float kp = 0.1;
float kd = 0.025;
float ki = 0.001;

double angleX;
double error_prior;
double error;
double integral;
double pid_inp;

const int timeStepMs = 50;

Adafruit_MPU6050 mpu;

void setup(void) {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit MPU6050 test!");

    if (!mpu.begin()) {
            Serial.println("Failed to find MPU6050 chip");

            while (1) {
                delay(10);
            }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    delay(100);
}

void loop() {
    analogWrite(IN1, 255);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 255);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (abs(g.gyro.x) >= 0.05) {
        angleX += (((double) (g.gyro.x * timeStepMs)) / 1000.0) * (180 / M_PI);
    }

    // pid
    error = 0 - angleX;
    integral += ((double) (error * timeStepMs)/1000.0) ;
    double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
    pid_inp = kp*error + ki*integral + kd*derivative;

    error_prior = error;

    Serial.println(pid_inp);
    delay(timeStepMs);
}
